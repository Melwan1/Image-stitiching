#include <images/ppm-image.hh>
#include <panorama/clean_cut_rectangular_builder.hh>

namespace tifo::panorama
{

    CleanCutRectangularBuilder&
    CleanCutRectangularBuilder::set_horizontal_slices(int horizontal_slices)
    {
        if (horizontal_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::CleanCutRectangularBuilder - cannot set "
                "nonpositive horizontal slices count.");
        }
        horizontal_slices_ = horizontal_slices;
        return *this;
    }

    CleanCutRectangularBuilder&
    CleanCutRectangularBuilder::set_vertical_slices(int vertical_slices)
    {
        if (vertical_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::CleanCutRectangularBuilder - cannot set "
                "nonpositive horizontal slices count.");
        }
        vertical_slices_ = vertical_slices;
        return *this;
    }

    image::Image* CleanCutRectangularBuilder::build()
    {
        if (input_images_.size()
                != static_cast<unsigned>(horizontal_slices_ * vertical_slices_)
            || horizontal_slices_ <= 0 || vertical_slices_ <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::CleanCutRectangularBuilder - build - "
                "cuts size is invalid.");
        }
        image::PPMImage* result_image = new image::PPMImage();
        int cut_width = input_images_[0]->get_width();
        int cut_height =
            input_images_[0]
                ->get_height(); // suppose every cut has the same size (could
                                // be dangerous for weird base image size!!)
        result_image->set_width(cut_width * horizontal_slices_);
        result_image->set_height(cut_height * vertical_slices_);
        result_image->get_pixels().resize(result_image->get_width()
                                          * result_image->get_height());
        for (int horizontal_index = 0; horizontal_index < horizontal_slices_;
             horizontal_index++)
        {
            for (int vertical_index = 0; vertical_index < vertical_slices_;
                 vertical_index++)
            {
                int x_min = cut_width * horizontal_index;
                int y_min = cut_height * vertical_index;
                for (int dx = 0; dx < cut_width; dx++)
                {
                    for (int dy = 0; dy < cut_height; dy++)
                    {
                        const std::vector<float>& input_pixel =
                            input_images_[vertical_index * horizontal_slices_
                                          + horizontal_index]
                                ->get_pixels()[dy * cut_width + dx];
                        std::vector<float>& output_pixel =
                            result_image
                                ->get_pixels()[(y_min + dy)
                                                   * result_image->get_width()
                                               + (x_min + dx)];
                        output_pixel.resize(3);
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
        return result_image;
    }

} // namespace tifo::panorama