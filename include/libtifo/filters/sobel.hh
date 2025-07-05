#pragma once

#include "filter.hh"

namespace tifo::filter
{

    class SobelX : public Filter<int, 3>
    {
    public:
        SobelX();
    };

    class SobelY : public Filter<int, 3>
    {
    public:
        SobelY();
    };

} // namespace tifo::filter