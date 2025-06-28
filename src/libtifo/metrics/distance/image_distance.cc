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
        if (input_images.first->get_width() != input_images.second->get_width())
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_input_images - "
                "images have different widths");
        }
        if (input_images.first->get_height()
            != input_images.second->get_height())
        {
            throw std::runtime_error(
                "tifo::metrics::distance::ImageDistance - set_input_images - "
                "images have different heights");
        }
        input_images_ = input_images;
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
        double sum_distance = 0;
        for (int y = 0; y < input_images_.first->get_height(); y++)
        {
            for (int x = 0; x < input_images_.first->get_width(); x++)
            {
                const std::vector<float>& input_pixel1 =
                    input_images_.first
                        ->get_pixels()[y * input_images_.first->get_width()
                                       + x];
                const std::vector<float>& input_pixel2 =
                    input_images_.second
                        ->get_pixels()[y * input_images_.first->get_width()
                                       + x];
                sum_distance += get_distance_between_rgb_pixels(
                    { input_pixel1[0], input_pixel1[1], input_pixel1[2] },
                    { input_pixel2[0], input_pixel2[1], input_pixel2[2] });
            }
        }
        return sum_distance;
    }

    double get_distance_between_rgb_pixels(
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