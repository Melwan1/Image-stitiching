#pragma once

#include <images/color-image.hh>

namespace tifo::panorama::cutter
{

    class Cutter
    {
    public:
        Cutter() = default;
        void set_input_image(image::ColorImage* input_image);
        virtual std::vector<image::ColorImage*> cut() = 0;
        void free_input();

    protected:
        image::ColorImage* input_image_ = nullptr;
    };

} // namespace tifo::panorama::cutter