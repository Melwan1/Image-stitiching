#include <images/grayscale-image.hh>
#include <images/grayscale-ppm-image.hh>
#include <sstream>

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

    bool GrayscaleImage::is_valid_access(int x, int y) const
    {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    const float& GrayscaleImage::operator()(int x, int y) const
    {
        if (!is_valid_access(x, y))
        {
            std::ostringstream oss;
            oss << "tifo::image::GrayscaleImage - pixel access - access at ("
                << x << ", " << y << ") is invalid for size (" << width_ << ", "
                << height_ << ")";
            throw std::runtime_error(oss.str());
        }
        return get_pixels()[y * get_width() + x];
    }

    float& GrayscaleImage::operator()(int x, int y)
    {
        if (!is_valid_access(x, y))
        {
            std::ostringstream oss;
            oss << "tifo::image::GrayscaleImage - pixel access - access at ("
                << x << ", " << y << ") is invalid for size (" << width_ << ", "
                << height_ << ")";
            throw std::runtime_error(oss.str());
        }
        return get_pixels()[y * get_width() + x];
    }

} // namespace tifo::image