#pragma once

#include <images/color-image.hh>
#include <vector>

namespace tifo::panorama::builder
{

    class Builder
    {
    public:
        Builder() = default;
        void set_input_images(const std::vector<image::ColorImage*>& input_images);
        virtual image::ColorImage* build() = 0;
        void free_inputs();

    protected:
        std::vector<image::ColorImage*> input_images_;
    };

} // namespace tifo::panorama::builder