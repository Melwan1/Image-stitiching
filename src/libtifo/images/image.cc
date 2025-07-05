#include <images/image.hh>
#include <iostream>
#include <sstream>

namespace tifo::image
{

    Image::Image()
        : height_(0)
        , width_(0)
    {}

    Image::Image(int width, int height)
        : height_(height)
        , width_(width)
    {}

    int Image::get_height() const
    {
        return height_;
    }

    int Image::get_width() const
    {
        return width_;
    }

    void Image::set_height(int height)
    {
        if (height <= 0)
        {
            throw std::runtime_error(
                "tifo::image::Image - set_height - height is nonpositive");
        }
        height_ = height;
    }

    void Image::set_width(int width)
    {
        if (width <= 0)
        {
            throw std::runtime_error(
                "tifo::image::Image - set_width - width is nonpositive");
        }
        width_ = width;
    }

} // namespace tifo::image

std::ostream& operator<<(std::ostream& ostr, const tifo::image::Image* image)
{
    if (!image)
    {
        return ostr << "image (null)";
    }
    return ostr << "image of size " << image->get_width() << "x"
                << image->get_height();
}