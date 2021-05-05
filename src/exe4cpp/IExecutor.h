/*
 * Copyright (c) 2018, Automatak LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EXE4CPP_IEXECUTOR_H
#define EXE4CPP_IEXECUTOR_H

#include "exe4cpp/Timer.h"
#include "exe4cpp/ISteadyTimeSource.h"

#include <future>

/**
* @brief exe4cpp header-only library namespace
*/
namespace exe4cpp
{

/**
* Provides thread-safe access to a value that can be set once.
*/
template <class T>
class Synchronized
{
public:

    Synchronized() : value(), isSet(false)
    {}

    T WaitForValue()
    {
        std::unique_lock<std::mutex> lock(mutex);
        while (!isSet)
        {
            auto complete = [this]()
            {
                return isSet;
            };
            condition.wait(lock, complete);
        }
        return value;
    }

    void SetValue(T value_)
    {
        std::unique_lock<std::mutex> lock(mutex);
        this->value = value_;
        isSet = true;
        condition.notify_all();
    }

private:

    T value;
    bool isSet;

    std::mutex mutex;
    std::condition_variable condition;
};

/**
 * Interface that abstracts an event loop.
 *
 * Events can be posted for to execute immediately or some time in the future.  Events
 * are processed in the order they are posted.
 *
 */
class IExecutor : public ISteadyTimeSource
{
public:

    virtual ~IExecutor() {}

    /// @return start a new timer based on a relative time duration
    virtual Timer start(const duration_t& duration, const action_t& action) = 0;

    /// @return start a new timer based on an absolute timestamp of the steady clock
    virtual Timer start(const steady_time_t& expiration, const action_t& action) = 0;

    /// @return start a new timer based on an absolute timestamp of the system clock
    virtual Timer start(const system_time_t& expiration, const action_t& action) = 0;

    /// @return Thread-safe way to post an event to be handled asynchronously
    virtual void post(const action_t& action) = 0;

    // Helper functions
    template<class T> T return_from(const std::function<T()>& action)
    {
        if (is_running_in_this_thread())
        {
            return action();
        } else {
            Synchronized<T> sync;
            auto pointer = &sync;
            auto lambda = [action, pointer]()
            {
                  T tmp = action();
                  pointer->SetValue(tmp);
            };
            strand.post(lambda);
            return sync.WaitForValue();
        }

        // std::promise<T> ready;

        // auto future = ready.get_future();

        // auto run = [&] { ready.set_value(action()); };

        // post(run);

        // future.wait();

        // return future.get();
    }

    void block_until(const std::function<void()>& action)
    {
        if (is_running_in_this_thread())
        {
            action();
            return;
        } else {
            Synchronized<bool> sync;
            auto pointer = &sync;
            auto lambda = [action, pointer]()
            {
                  action();
                  pointer->SetValue(true);
            };
            strand.post(lambda);
            sync.WaitForValue();
        }

        // std::promise<bool> ready;

        // auto future = ready.get_future();

        // auto run = [&] {
        //     action();
        //     ready.set_value(true);
        // };

        // post(run);

        // future.wait();
    }

    void block_until_and_flush(const std::function<void()>& action)
    {
        this->block_until(action);
        this->block_until([]() {});
    }

protected:
    virtual bool is_running_in_this_thread() = 0;
};

}

#endif
