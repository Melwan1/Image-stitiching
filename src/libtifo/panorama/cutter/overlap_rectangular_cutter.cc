#include <images/ppm-image.hh>
#include <iostream>
#include <panorama/cutter/overlap_rectangular_cutter.hh>

namespace tifo::panorama::cutter
{

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_horizontal_slices(int horizontal_slices)
    {
        if (horizontal_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_horizontal_slices - horizontal_slices cannot be "
                "nonpositive.");
        }
        horizontal_slices_ = horizontal_slices;
        return *this;
    }

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_vertical_slices(int vertical_slices)
    {
        if (vertical_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_vertical_slices - vertical_slices cannot be nonpositive.");
        }
        vertical_slices_ = vertical_slices;
        return *this;
    }

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_horizontal_overlap_size(
        int horizontal_overlap_size)
    {
        if (horizontal_overlap_size <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_horizontal_overlap_size - horizontal_overlap_size cannot "
                "be nonpositive.");
        }
        horizontal_overlap_size_ = horizontal_overlap_size;
        return *this;
    }

    OverlapRectangularCutter&
    OverlapRectangularCutter::set_vertical_overlap_size(
        int vertical_overlap_size)
    {
        if (vertical_overlap_size <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::OverlapRectangularCutter - "
                "set_vertical_overlap_size - vertical_overlap_size cannot be "
                "nonpositive.");
        }
        vertical_overlap_size_ = vertical_overlap_size;
        return *this;
    }

    std::vector<image::Image*> OverlapRectangularCutter::cut()
    {
        // Each image goes from (x_min - horizontal_overlap, y_min -
        // vertical_overlap) to (x_max + horizontal_overlap, y_max +
        // horizontal_overlap) If the overlap makes any coordinate out of
        // bounds, it is clamped to the bounds of the initial image.

        std::vector<image::Image*> result_images;

        // the overlap cannot make an image completely overlap another, so it
        // cannot be larger than the size of the cut images
        if (horizontal_overlap_size_
                >= input_image_->get_width() / horizontal_slices_
            || vertical_overlap_size_
                >= input_image_->get_height() / vertical_slices_)
        {
            throw std::runtime_error(
                "tifo::panorama::OverlapRectangularCutter - cut - the overlap "
                "cannot be bigger than the size of the cut image");
        }

        for (int horizontal_index = 0; horizontal_index < horizontal_slices_;
             horizontal_index++)
        {
            for (int vertical_index = 0; vertical_index < vertical_slices_;
                 vertical_index++)
            {
                int x_min =
                    std::clamp(input_image_->get_width() * horizontal_index
                                       / horizontal_slices_
                                   - horizontal_overlap_size_,
                               0, input_image_->get_width());
                int x_max = std::clamp(input_image_->get_width()
                                               * (horizontal_index + 1)
                                               / horizontal_slices_
                                           + horizontal_overlap_size_,
                                       0, input_image_->get_width());
                int y_min =
                    std::clamp(input_image_->get_height() * vertical_index
                                       / vertical_slices_
                                   - vertical_overlap_size_,
                               0, input_image_->get_height());
                int y_max =
                    std::clamp(input_image_->get_height() * (vertical_index + 1)
                                       / vertical_slices_
                                   + vertical_overlap_size_,
                               0, input_image_->get_height());
                std::cout << "cutting image from (" << x_min << ", " << y_min
                          << ") to (" << x_max << ", " << y_max << ")\n";

                image::PPMImage* cut_image = new image::PPMImage();
                cut_image->set_height(y_max - y_min);
                cut_image->set_width(x_max - x_min);
                cut_image->get_pixels().resize(cut_image->get_width()
                                               * cut_image->get_height());

                for (int dx = 0; dx < cut_image->get_width(); dx++)
                {
                    for (int dy = 0; dy < cut_image->get_height(); dy++)
                    {
                        const std::vector<float>& input_pixel =
                            input_image_
                                ->get_pixels()[(y_min + dy)
                                                   * input_image_->get_width()
                                               + x_min + dx];
                        std::vector<float>& output_pixel =
                            cut_image->get_pixels()[dy * cut_image->get_width()
                                                    + dx];

                        output_pixel.resize(3);

                        for (int color_index = 0; color_index < 3;
                             color_index++)
                        {
                            output_pixel[color_index] =
                                input_pixel[color_index];
                        }
                    }
                }

                result_images.emplace_back(cut_image);
            }
        }
        return result_images;
    }

} // namespace tifo::panorama::cutter