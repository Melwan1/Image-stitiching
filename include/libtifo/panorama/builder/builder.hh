#pragma once

#include <images/image.hh>
#include <vector>

namespace tifo::panorama::builder
{

    class Builder
    {
    public:
        Builder() = default;
        void set_input_images(const std::vector<image::Image*>& input_images);
        virtual image::Image* build() = 0;
        void free_inputs();

    protected:
        std::vector<image::Image*> input_images_;
    };

} // namespace tifo::panorama::builder