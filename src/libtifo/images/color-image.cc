#include <images/color-image.hh>
#include <images/grayscale-ppm-image.hh>
#include <iostream>
#include <sstream>

namespace tifo::image
{

    ColorImage::ColorImage()
        : Image()
        , pixels_()
    {}

    ColorImage::ColorImage(int width, int height)
        : Image(width, height)
        , pixels_()
    {
        pixels_.resize(width * height);
        for (int index = 0; index < width * height; index++)
        {
            pixels_.at(index).resize(3);
        }
    }

    const ColorImage::container_type& ColorImage::get_pixels() const
    {
        return pixels_;
    }

    ColorImage::container_type& ColorImage::get_pixels()
    {
        return pixels_;
    }

    GrayscaleImage* ColorImage::to_grayscale() const
    {
        GrayscalePPMImage* result_image =
            new GrayscalePPMImage(width_, height_);
        for (int y = 0; y < height_; y++)
        {
            for (int x = 0; x < width_; x++)
            {
                int pixel_index = y * width_ + x;
                float sum = 0;
                for (int channel_index = 0; channel_index < 3; channel_index++)
                {
                    sum += get_pixels()[pixel_index][channel_index];
                }
                result_image->get_pixels()[pixel_index] = sum / 3;
            }
        }
        return result_image;
    }

    bool ColorImage::is_valid_access(int x, int y) const
    {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    const std::vector<float>& ColorImage::operator()(int x, int y) const
    {
        if (!is_valid_access(x, y))
        {
            std::ostringstream oss;
            oss << "tifo::image::ColorImage - pixel access - access at (" << x
                << ", " << y << ") is invalid for size (" << width_ << ", "
                << height_ << ")";
            throw std::runtime_error(oss.str());
        }
        return get_pixels()[y * get_width() + x];
    }

    std::vector<float>& ColorImage::operator()(int x, int y)
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