#pragma once

#include <images/image.hh>

namespace tifo::metrics::distance
{

    class ImageDistance
    {
    public:
        ImageDistance();

        void set_input_images(
            const std::pair<image::Image*, image::Image*>& input_images);
        double compute_distance();
        double get_distance_between_rgb_pixels(
            const std::tuple<float, float, float>& pixel1,
            const std::tuple<float, float, float>& pixel2);
        void free_images();

    private:
        std::pair<image::Image*, image::Image*> input_images_;
    };

} // namespace tifo::metrics::distance