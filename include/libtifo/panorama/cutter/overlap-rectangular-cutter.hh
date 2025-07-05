#pragma once

#include <panorama/cutter/cutter.hh>

namespace tifo::panorama::cutter
{

    class OverlapRectangularCutter : public Cutter
    {
    public:
        OverlapRectangularCutter& set_horizontal_slices(int horizontal_slices);
        OverlapRectangularCutter& set_vertical_slices(int vertical_slices);
        OverlapRectangularCutter&
        set_horizontal_overlap_size(int horizontal_overlap_size);
        OverlapRectangularCutter&
        set_vertical_overlap_size(int vertical_overlap_size);
        std::vector<image::ColorImage*> cut() override;

    private:
        int horizontal_slices_;
        int vertical_slices_;

        int horizontal_overlap_size_;
        int vertical_overlap_size_;
    };

} // namespace tifo::panorama::cutter