#pragma once

#include <tuple>

namespace tifo::metrics::convert
{

    class ColorConverter
    {
    public:
        static std::tuple<double, double, double>
        rgb_to_xyz(const std::tuple<double, double, double>& rgb);
        static std::tuple<double, double, double>
        xyz_to_lab(const std::tuple<double, double, double>& xyz);
        static std::tuple<double, double, double>
        rgb_to_lab(const std::tuple<double, double, double>& rgb);

        static double lab_transform(double xyz_intensity);
        static double gamma_correction(double color_intensity);

    private:
    };

} // namespace tifo::metrics::convert