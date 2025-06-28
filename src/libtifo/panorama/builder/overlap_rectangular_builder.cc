#include <panorama/builder/overlap_rectangular_builder.hh>

namespace tifo::panorama::builder
{

    OverlapRectangularBuilder&
    OverlapRectangularBuilder::set_horizontal_slices(int horizontal_slices)
    {
        if (horizontal_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::OverlapRectangularBuilder - cannot set "
                "nonpositive horizontal slices count.");
        }
        horizontal_slices_ = horizontal_slices;
        return *this;
    }

    OverlapRectangularBuilder&
    OverlapRectangularBuilder::set_vertical_slices(int vertical_slices)
    {
        if (vertical_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::OverlapRectangularBuilder - cannot set "
                "nonpositive horizontal slices count.");
        }
        vertical_slices_ = vertical_slices;
        return *this;
    }

    image::Image* OverlapRectangularBuilder::build()
    {
        /** for now we will consider that the images are well placed with
         * respect to each other. We therefore just need to compute the overlap
         * size and glue the images back together. For simplicity, we will also
         * consider that the overlap is constant among the images, so we only
         * calculate the overlap for images at position (0, 0) and (0, 1) on one
         * hand and (0, 0) and (1, 0) on the other hand
         */
        int horizontal_overlap_size = 0;
        int vertical_overlap_size = 0;
    }

} // namespace tifo::panorama::builder