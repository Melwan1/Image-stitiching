#include <panorama/cutter/overlap_rectangular_cutter.hh>

namespace tifo::panorama::cutter
{

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_horizontal_slices(int horizontal_slices)
    {
        if (horizontal_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_horizontal_slices - horizontal_slices cannot be "
                "nonpositive.");
        }
        horizontal_slices_ = horizontal_slices;
        return *this;
    }

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_vertical_slices(int vertical_slices)
    {
        if (vertical_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_vertical_slices - vertical_slices cannot be nonpositive.");
        }
        vertical_slices_ = vertical_slices;
        return *this;
    }

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_horizontal_overlap_size(
        int horizontal_overlap_size)
    {
        if (horizontal_overlap_size <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_horizontal_overlap_size - horizontal_overlap_size cannot "
                "be nonpositive.");
        }
        horizontal_overlap_size_ = horizontal_overlap_size;
        return *this;
    }

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_vertical_overlap_size(
        int vertical_overlap_size)
    {
        if (vertical_overlap_size <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_vertical_overlap_size - vertical_overlap_size cannot be "
                "nonpositive.");
        }
        vertical_overlap_size_ = vertical_overlap_size;
        return *this;
    }

    std::vector<image::Image*> OverlapRectangularCutter::cut()
    {
        return {};
    }

} // namespace tifo::panorama::cutter