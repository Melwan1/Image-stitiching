#pragma once

#include <filters/filter.hh>
#include <math/matrix.hh>

namespace tifo::filter
{

    template <unsigned size>
    class GaussianFilter : public Filter<float, 2 * size + 1>
    {
    public:
        GaussianFilter(float sigma);

        template <unsigned filter_size>
        static const math::SquaredMatrix<float, 2 * filter_size + 1>
        generate_gaussian_filter_matrix(float sigma);
    };

} // namespace tifo::filter

#include "gaussian.hxx"