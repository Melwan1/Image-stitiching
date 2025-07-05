#pragma once

#include <filters/filter.hh>
#include <filters/gaussian.hh>
#include <numbers>

namespace tifo::filter
{

    template <unsigned size>
    template <unsigned filter_size>
    const math::SquaredMatrix<float, 2 * filter_size + 1>
    GaussianFilter<size>::generate_gaussian_filter_matrix(float sigma)
    {
        math::SquaredMatrix<float, 2 * filter_size + 1> matrix;

        float denominator = 1 / (std::sqrt(2 * std::numbers::pi) * sigma);

        for (int y = -size; y <= static_cast<int>(size); y++)
        {
            for (int x = -size; x <= static_cast<int>(size); x++)
            {
                matrix(x + size, y + size) = denominator
                    * std::exp(-(x * x + y * y) / (2 * sigma * sigma));
            }
        }
        return matrix;
    }

    template <unsigned size>
    GaussianFilter<size>::GaussianFilter(float sigma)
        : Filter<float, 2 * size + 1>(
              generate_gaussian_filter_matrix<size>(sigma))
    {}

} // namespace tifo::filter