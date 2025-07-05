#pragma once

#include <images/ppm-image.hh>
#include <math/matrix.hh>

#include "filter.hh"

namespace tifo::filter
{

    template <typename ElementType, unsigned size>
    Filter<ElementType, size>::Filter(
        const math::SquaredMatrix<ElementType, size>& matrix)
        : matrix_(matrix)
    {}

    template <typename ElementType, unsigned size>
    image::Image*
    Filter<ElementType, size>::apply_on_image(const image::Image* input_image)
    {
        if (!input_image)
        {
            return nullptr;
        }
        image::PPMImage* result_image = new image::PPMImage(
            input_image->get_width(), input_image->get_height());
        int half_size = size / 2;
        for (int y = 0; y < input_image->get_height(); y++)
        {
            for (int x = 0; x < input_image->get_width(); x++)
            {
                for (int channel_index = 0; channel_index < 3; channel_index++)
                {
                    float sum = 0;
                    for (int dy = -half_size; dy <= half_size; dy++)
                    {
                        for (int dx = -half_size; dx <= half_size; dx++)
                        {
                            sum += input_image->get_pixels()
                                       [(y + dy) * input_image->get_width() + x
                                        + dx][channel_index]
                                * matrix_(dy + half_size, dx + half_size);
                        }
                    }
                    result_image->get_pixels()[y * result_image->get_width()
                                               + x][channel_index] = sum;
                }
            }
        }
        return result_image;
    }

} // namespace tifo::filter