#include <config/function-timer.hh>

namespace tifo::config
{

    FunctionTimer::FunctionTimer(const std::string& namespace_name,
                                 const std::string& function_name,
                                 std::ostream& ostr)
        : namespace_name_(namespace_name)
        , function_name_(function_name)
        , ostr_(ostr)
        , start_(std::chrono::steady_clock::now())
    {}

    FunctionTimer::~FunctionTimer()
    {
        auto end = std::chrono::steady_clock::now();
        ostr_ << "\033[0;36m" << "namespace " << namespace_name_ << "\033[0;32m"
              << " function " << function_name_ << "\033[0;33m" << " time "
              << (end - start_).count() / 1E6 << "ms" << "\033[0m" << "\n";
    }

} // namespace tifo::config