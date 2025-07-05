#pragma once

#include <filters/filter.hh>
#include <filters/laplacian.hh>

namespace tifo::filter
{

    class Laplacian : public Filter<int, 3>
    {
    public:
        Laplacian();
    };
} // namespace tifo::filter