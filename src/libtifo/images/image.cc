#include <images/image.hh>

namespace tifo::image
{

    Image::Image()
        : pixels_()
        , height_(0)
        , width_(0)
    {}

    const Image::container_type& Image::get_pixels() const
    {
        return pixels_;
    }

    Image::container_type& Image::get_pixels()
    {
        return pixels_;
    }

    int Image::get_height() const
    {
        return height_;
    }

    int Image::get_width() const
    {
        return width_;
    }

} // namespace tifo::image