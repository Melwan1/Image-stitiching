#pragma once

#include <panorama/builder/builder.hh>

namespace tifo::panorama::builder
{

    class OverlapRectangularBuilder : public Builder
    {
    public:
        OverlapRectangularBuilder& set_horizontal_slices(int horizontal_slices);
        OverlapRectangularBuilder& set_vertical_slices(int vertical_slices);
        // the overlap is not known

        int get_overlap_x();
        int get_overlap_y();
        image::Image* build() override;

    private:
        int horizontal_slices_;
        int vertical_slices_;
    };

} // namespace tifo::panorama::builder