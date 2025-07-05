#include <images/color-ppm-image.hh>
#include <panorama/cutter/clean-rectangular-cutter.hh>
#include <sstream>

namespace tifo::panorama::cutter
{

    CleanRectangularCutter&
    CleanRectangularCutter::set_horizontal_slices(int horizontal_slices)
    {
        if (horizontal_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::CleanRectangularCutter - "
                "set_horizontal_slices - horizontal_slices cannot be "
                "nonpositive.");
        }
        horizontal_slices_ = horizontal_slices;
        return *this;
    }

    CleanRectangularCutter&
    CleanRectangularCutter::set_vertical_slices(int vertical_slices)
    {
        if (vertical_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::CleanRectangularCutter - "
                "set_vertical_slices - vertical_slices cannot be nonpositive.");
        }
        vertical_slices_ = vertical_slices;
        return *this;
    }

    std::vector<image::ColorImage*> CleanRectangularCutter::cut()
    {
        if (horizontal_slices_ <= 0
            || horizontal_slices_ > input_image_->get_width()
            || vertical_slices_ <= 0
            || vertical_slices_ > input_image_->get_height())
        {
            std::ostringstream oss;
            oss << "tifo::panorama::cutter::CleanRectangularCutter - cut - "
                   "cannot perform cuts "
                   "with slices count ("
                << horizontal_slices_ << ", " << vertical_slices_ << ")";
            throw std::runtime_error(oss.str());
        }

        std::vector<image::ColorImage*> cut_images;
        cut_images.resize(horizontal_slices_ * vertical_slices_);
        for (int horizontal_index = 0; horizontal_index < horizontal_slices_;
             horizontal_index++)
        {
            for (int vertical_index = 0; vertical_index < vertical_slices_;
                 vertical_index++)
            {
                int x_min = input_image_->get_width() * horizontal_index
                    / horizontal_slices_;
                int x_max = input_image_->get_width() * (horizontal_index + 1)
                    / horizontal_slices_;
                int y_min = input_image_->get_height() * vertical_index
                    / vertical_slices_;
                int y_max = input_image_->get_height() * (vertical_index + 1)
                    / vertical_slices_;

                image::ColorImage* cropped_image = cut_images.at(
                    vertical_index * horizontal_slices_ + horizontal_index);
                cut_images[vertical_index * horizontal_slices_
                           + horizontal_index] = new image::ColorPPMImage(x_max - x_min, y_max - y_min);
                for (int x = x_min; x < x_max; x++)
                {
                    for (int y = y_min; y < y_max; y++)
                    {
                        const std::vector<float>& input_pixel =
                            input_image_->get_pixels().at(
                                y * input_image_->get_width() + x);
                        std::vector<float>& output_pixel =
                            cropped_image->get_pixels().at(
                                (y - y_min) * cropped_image->get_width() + x
                                - x_min);
                        for (int color_index = 0; color_index < 3;
                             color_index++)
                        {
                            output_pixel[color_index] =
                                input_pixel[color_index];
                        }
                    }
                }
            }
        }
        return cut_images;
    }

} // namespace tifo::panorama::cutter