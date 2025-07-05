#pragma once

#include <panorama/cutter/cutter.hh>

namespace tifo::panorama::cutter
{

    class CleanRectangularCutter : public Cutter
    {
    public:
        CleanRectangularCutter& set_horizontal_slices(int horizontal_slices);
        CleanRectangularCutter& set_vertical_slices(int vertical_slices);

        std::vector<image::ColorImage*> cut() override;

    private:
        int horizontal_slices_;
        int vertical_slices_;
    };

} // namespace tifo::panorama::cutter