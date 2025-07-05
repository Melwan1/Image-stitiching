#pragma once

#include <chrono>
#include <iostream>

namespace tifo::config
{

    class FunctionTimer
    {
        /**
         * When timing a function, one should call the function timer this way:
         *
         * void some_namespace::do_something() {
         *      FunctionTimer timer("some_namespace", "do_something");
         *      // really do something
         *      // ...
         * }
         *
         * The destructor will have the responsibility to log the time taken by
         * the function
         *
         */

    public:
        FunctionTimer(const std::string& namespace_name,
                      const std::string& function_name,
                      std::ostream& ostr = std::cout);
        ~FunctionTimer();

    private:
        std::string namespace_name_;
        std::string function_name_;
        std::ostream& ostr_;
        std::chrono::steady_clock::time_point start_;
    };

} // namespace tifo::config