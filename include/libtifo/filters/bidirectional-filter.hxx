#pragma once

#include <cmath>
#include <filters/filter.hh>
#include <images/grayscale-image.hh>
#include <images/grayscale-ppm-image.hh>

#include "bidirectional-filter.hh"

namespace tifo::filter
{

    template <typename ElementType, unsigned size>
    BidirectionalFilter<ElementType, size>::BidirectionalFilter(
        const Filter<ElementType, size>& horizontal_filter,
        const Filter<ElementType, size>& vertical_filter)
        : Filter<ElementType, size>(horizontal_filter.get_matrix())
        , horizontal_filter_(horizontal_filter)
        , vertical_filter_(vertical_filter)
    {}

    template <typename ElementType, unsigned size>
    image::GrayscaleImage*
    BidirectionalFilter<ElementType, size>::apply_on_image(
        const image::GrayscaleImage* input_image)
    {
        image::GrayscaleImage* horizontally_filtered_image =
            horizontal_filter_.apply_on_image(input_image);
        horizontally_filtered_image->write("sobel_x.ppm");
        image::GrayscaleImage* vertically_filtered_image =
            vertical_filter_.apply_on_image(input_image);
        vertically_filtered_image->write("sobel_y.ppm");
        image::GrayscalePPMImage* result_image = new image::GrayscalePPMImage(
            horizontally_filtered_image->get_width(),
            horizontally_filtered_image->get_height());

        for (int y = 0; y < result_image->get_height(); y++)
        {
            for (int x = 0; x < result_image->get_width(); x++)
            {
                int vector_index = y * result_image->get_width() + x;
                {
                    float horizontal =
                        horizontally_filtered_image->get_pixels()[vector_index];
                    float vertical =
                        vertically_filtered_image->get_pixels()[vector_index];

                    result_image->get_pixels()[vector_index] = std::sqrt(
                        horizontal * horizontal + vertical * vertical);
                }
            }
        }

        return result_image;
    }

} // namespace tifo::filter