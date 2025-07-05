#pragma once

#include <images/grayscale-ppm-image.hh>
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
    image::GrayscaleImage* Filter<ElementType, size>::apply_on_image(
        const image::GrayscaleImage* input_image)
    {
        std::cout << matrix_ << "\n";
        if (!input_image)
        {
            return nullptr;
        }
        input_image->write("grayscale.ppm");
        image::GrayscalePPMImage* result_image = new image::GrayscalePPMImage(
            input_image->get_width(), input_image->get_height());
        int half_size = size / 2;
        for (int y = 0; y < input_image->get_height(); y++)
        {
            for (int x = 0; x < input_image->get_width(); x++)
            {
                if (y < half_size || y >= input_image->get_height() - half_size
                    || x < half_size || x >= input_image->get_width())
                {
                    result_image
                        ->get_pixels()[y * result_image->get_width() + x] = 0;
                    continue;
                }
                float sum = 0;
                for (int dy = -half_size; dy <= half_size; dy++)
                {
                    for (int dx = -half_size; dx <= half_size; dx++)
                    {
                        sum +=
                            input_image->get_pixels()
                                [(y + dy) * input_image->get_width() + x + dx]
                            * matrix_(dy + half_size, dx + half_size);
                    }
                }
                if (sum < 0)
                {
                    sum *= -1;
                }
                result_image->get_pixels()[y * result_image->get_width() + x] =
                    sum / (size * size);
            }
        }
        return result_image;
    }

    template <typename ElementType, unsigned size>
    const math::SquaredMatrix<ElementType, size>
    Filter<ElementType, size>::get_matrix() const
    {
        return matrix_;
    }

} // namespace tifo::filter