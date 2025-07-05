#pragma once

#include "bidirectional-filter.hh"
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

    class Sobel : public BidirectionalFilter<int, 3>
    {
    public:
        Sobel();
    };

} // namespace tifo::filter