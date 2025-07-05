#pragma once

#include <panorama/builder/builder.hh>

namespace tifo::panorama::builder
{

    class CleanRectangularBuilder : public Builder
    {
    public:
        CleanRectangularBuilder& set_horizontal_slices(int horizontal_slices);
        CleanRectangularBuilder& set_vertical_slices(int vertical_slices);
        image::ColorImage* build() override;

    private:
        int horizontal_slices_;
        int vertical_slices_;
    };

} // namespace tifo::panorama::builder