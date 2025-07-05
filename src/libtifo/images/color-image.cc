#include <images/color-image.hh>

namespace tifo::image {

    ColorImage::ColorImage()
        : Image()
        , pixels_()
    {}

    ColorImage::ColorImage(int width, int height)
        : Image(width, height)
        , pixels_()
    {
        pixels_.resize(width * height);
        for (int index = 0; index < width * height; index++) {
            pixels_.at(index).resize(3);
        }
    }

    const ColorImage::container_type& ColorImage::get_pixels() const {
        return pixels_;
    }

    ColorImage::container_type& ColorImage::get_pixels() {
        return pixels_;
    }

}