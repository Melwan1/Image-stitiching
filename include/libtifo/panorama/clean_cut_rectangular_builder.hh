#pragma once

#include <panorama/builder.hh>

namespace tifo::panorama
{

    class CleanCutRectangularBuilder : public Builder
    {
    public:
        CleanCutRectangularBuilder&
        set_horizontal_slices(int horizontal_slices);
        CleanCutRectangularBuilder& set_vertical_slices(int vertical_slices);
        image::Image* build() override;

    private:
        int horizontal_slices_;
        int vertical_slices_;
    };

} // namespace tifo::panorama