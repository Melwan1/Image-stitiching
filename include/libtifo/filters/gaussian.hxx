#pragma once

#include <filters/filter.hh>
#include <filters/gaussian.hh>
#include <numbers>

namespace tifo::filter
{

    template <unsigned unused_size>
    template <unsigned filter_size>
    const math::SquaredMatrix<float, 2 * filter_size + 1>
    GaussianFilter<unused_size>::generate_gaussian_filter_matrix(float sigma)
    {
        math::SquaredMatrix<float, 2 * filter_size + 1> matrix;

        float denominator = 1.0 / (2.0 * std::numbers::pi * sigma * sigma);

        float sum = 0;
        for (int y = -filter_size; y <= static_cast<int>(filter_size); y++)
        {
            for (int x = -filter_size; x <= static_cast<int>(filter_size); x++)
            {
                matrix(x + filter_size, y + filter_size) = denominator
                    * std::exp(-(x * x + y * y) / (2 * sigma * sigma));
                sum += matrix(x + filter_size, y + filter_size);
            }
        }

        return matrix * (1 / sum);
    }

    template <unsigned size>
    GaussianFilter<size>::GaussianFilter(float sigma)
        : Filter<float, 2 * size + 1>(
              generate_gaussian_filter_matrix<size>(sigma))
    {}

} // namespace tifo::filter