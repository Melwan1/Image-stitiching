#pragma once

#include <images/image.hh>

namespace tifo::panorama::cutter
{

    class Cutter
    {
    public:
        Cutter() = default;
        void set_input_image(image::Image* input_image);
        virtual std::vector<image::Image*> cut() = 0;
        void free_input();

    protected:
        image::Image* input_image_ = nullptr;
    };

} // namespace tifo::panorama::cutter