#include <images/grayscale-image.hh>
#include <images/grayscale-ppm-image.hh>

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

    GrayscaleImage* GrayscaleImage::downsample(int factor) const
    {
        GrayscalePPMImage* result =
            new GrayscalePPMImage(width_ / factor, height_ / factor);
        for (int y = 0; y < height_ / factor; y++)
        {
            for (int x = 0; x < width_ / factor; x++)
            {
                result->get_pixels()[y * result->get_width() + x] =
                    get_pixels()[y * factor * width_ + x * factor];
            }
        }
        return result;
    }

} // namespace tifo::image