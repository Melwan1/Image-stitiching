#pragma once

#include <cmath>

#include "bidirectional-filter.hh"

namespace tifo::filter
{

    template <typename ElementType, unsigned size>
    BidirectionalFilter<ElementType, size>::BidirectionalFilter(
        const Filter<ElementType, size>& horizontal_filter,
        const Filter<ElementType, size> vertical_filter)
        : horizontal_filter_(horizontal_filter)
        , vertical_filter_(vertical_filter)
        , matrix_(horizontal_filter.matrix_)
    {}

    template <typename ElementType, unsigned size>
    image::Image* BidirectionalFilter<ElementType, size>::apply_on_image(
        const image::Image* input_image)
    {
        image::Image* horizontally_filtered_image =
            horizontal_filter_.apply_on_image(input_image);
        image::Image* vertically_filtered_image =
            vertical_filter_.apply_on_image(input_image);
        image::PPMImage* result_image =
            new image::PPMImage(horizontally_filtered_image->get_width(),
                                horizontally_filtered_image->get_height());

        for (int y = 0; y < result_image->get_height(); y++)
        {
            for (int x = 0; x < result_image->get_width(); x++)
            {
                int vector_index = y * result_image->get_width() + x;
                for (int channel_index = 0; < channel_index < 3;
                     channel_index++)
                {
                    int horizontal_channel =
                        horizontally_filtered_image
                            ->get_pixels()[vector_index][channel_index];
                    int vertical_channel =
                        vertically_filtered_image
                            ->get_pixels()[vector_index][channel_index];

                    result_image->get_pixels()[vector_index][channel_index] =
                        std::sqrt(horizontal_channel * horizontal_channel
                                  + vertical_channel * vertical_channel);
                }
            }
        }

        return result_image;
    }

} // namespace tifo::filter