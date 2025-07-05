#include <images/grayscale-image.hh>

namespace tifo::image
{

    GrayscaleImage::GrayscaleImage()
        : Image()
        , pixels_()
    {}

    GrayscaleImage::GrayscaleImage(int width, int height)
        : Image(width, height)
        , pixels_()
    {
        pixels_.resize(width * height);
    }

    const GrayscaleImage::container_type& GrayscaleImage::get_pixels() const
    {
        return pixels_;
    }

    GrayscaleImage::container_type& GrayscaleImage::get_pixels()
    {
        return pixels_;
    }

} // namespace tifo::image