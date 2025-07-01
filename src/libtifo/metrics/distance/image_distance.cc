#include <iostream>
#include <metrics/convert/color_converter.hh>
#include <metrics/distance/image_distance.hh>

namespace tifo::metrics::distance
{

    ImageDistance::ImageDistance()
        : input_images_({ nullptr, nullptr })
    {}

    void ImageDistance::set_input_images(
        const std::pair<image::Image*, image::Image*>& input_images)
    {
        if (!input_images.first || !input_images.second)
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_input_images - "
                "cannot set a nullptr input image");
        }
        input_images_ = input_images;
        set_image_crop_grid({ 0, 0, input_images_.first->get_width(),
                              input_images_.first->get_height() },
                            0);
        set_image_crop_grid({ 0, 0, input_images_.second->get_width(),
                              input_images_.second->get_height() },
                            1);
    }

    void
    ImageDistance::set_image_crop_grid(std::tuple<int, int, int, int> crop_grid,
                                       int image_index)
    {
        if (image_index < 0 || image_index > 1)
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_image_crop_grid "
                "- image_index must be 0 or 1.");
        }
        if (std::get<0>(crop_grid) < 0 || std::get<1>(crop_grid) < 0
            || std::get<2>(crop_grid) < 0 || std::get<3>(crop_grid) < 0)
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_image_crop_grid "
                "- the crop grid must contain positive integers.");
        }
        if (std::get<0>(crop_grid) >= std::get<2>(crop_grid)
            || std::get<1>(crop_grid) >= std::get<3>(crop_grid))
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_image_crop_grid "
                "- the crop grid has min coordinates >= max coordinates along "
                "an axis");
        }
        image::Image* image_to_check = nullptr;
        if (image_index == 0)
        {
            crop_grid1_ = crop_grid;
            image_to_check = input_images_.first;
        }
        else
        {
            crop_grid2_ = crop_grid;
            image_to_check = input_images_.second;
        }
        if (std::get<2>(crop_grid) > image_to_check->get_width()
            || std::get<3>(crop_grid) > image_to_check->get_height())
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_image_crop_grid "
                "- the crop grid must contain inbounds coordinates.");
        }
    }

    void ImageDistance::free_images()
    {
        delete input_images_.first;
        delete input_images_.second;
    }

    double ImageDistance::compute_distance()
    {
        if (!input_images_.first || !input_images_.second)
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - compute_distance - "
                "set input images before computing distances.");
        }
        // check crop grid sizes are the same
        if (std::get<2>(crop_grid1_) - std::get<0>(crop_grid1_)
            != std::get<2>(crop_grid2_) - std::get<0>(crop_grid2_))
        {
            std::cout << "crop grid 1: from (" << std::get<0>(crop_grid1_)
                      << ", " << std::get<1>(crop_grid1_) << ") to ("
                      << std::get<2>(crop_grid1_) << ", "
                      << std::get<3>(crop_grid1_) << ")\n";
            std::cout << "crop grid 2: from (" << std::get<0>(crop_grid2_)
                      << ", " << std::get<1>(crop_grid2_) << ") to ("
                      << std::get<2>(crop_grid2_) << ", "
                      << std::get<3>(crop_grid2_) << ")\n";
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - compute_distance - "
                "both crop grids must have same size along axis x.");
        }
        if (std::get<3>(crop_grid1_) - std::get<1>(crop_grid1_)
            != std::get<3>(crop_grid2_) - std::get<1>(crop_grid2_))
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - compute_distance - "
                "both crop grids must have same size along axis y.");
        }
        int crop_grid_size_x =
            std::get<2>(crop_grid1_) - std::get<0>(crop_grid1_);
        int crop_grid_size_y =
            std::get<3>(crop_grid1_) - std::get<1>(crop_grid1_);
        double sum_distance = 0;
        for (int dy = 0; dy < crop_grid_size_y; dy++)
        {
            for (int dx = 0; dx < crop_grid_size_x; dx++)
            {
                const std::vector<float>& input_pixel1 =
                    input_images_.first
                        ->get_pixels()[(dy + std::get<1>(crop_grid1_))
                                           * input_images_.first->get_width()
                                       + dx + std::get<0>(crop_grid1_)];
                const std::vector<float>& input_pixel2 =
                    input_images_.second
                        ->get_pixels()[(dy + std::get<1>(crop_grid2_))
                                           * input_images_.second->get_width()
                                       + dx + std::get<0>(crop_grid2_)];
                double distance = get_distance_between_rgb_pixels(
                    { input_pixel1[0], input_pixel1[1], input_pixel1[2] },
                    { input_pixel2[0], input_pixel2[1], input_pixel2[2] });
                sum_distance += distance;
            }
        }
        return sum_distance / (crop_grid_size_x * crop_grid_size_y);
    }

    double ImageDistance::get_distance_between_rgb_pixels(
        const std::tuple<float, float, float>& pixel1,
        const std::tuple<float, float, float>& pixel2)
    {
        std::tuple<double, double, double> lab1 =
            convert::ColorConverter::rgb_to_lab(pixel1);
        std::tuple<double, double, double> lab2 =
            convert::ColorConverter::rgb_to_lab(pixel2);
        return std::sqrt(std::pow(std::get<0>(lab1) - std::get<0>(lab2), 2)
                         + std::pow(std::get<1>(lab1) - std::get<1>(lab2), 2)
                         + std::pow(std::get<2>(lab1) - std::get<2>(lab2), 2));
    }

} // namespace tifo::metrics::distance