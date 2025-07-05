#include <algorithm>
#include <cmath>
#include <metrics/convert/color-converter.hh>

namespace tifo::metrics::convert
{

    std::tuple<double, double, double>
    ColorConverter::rgb_to_xyz(const std::tuple<double, double, double>& rgb)
    {
        double normalized_r = std::get<0>(rgb);
        double normalized_g = std::get<1>(rgb);
        double normalized_b = std::get<2>(rgb);

        normalized_r = gamma_correction(normalized_r);
        normalized_g = gamma_correction(normalized_g);
        normalized_b = gamma_correction(normalized_b);

        return { 0.4124564 * normalized_r + 0.3575761 * normalized_g
                     + 0.1804375 * normalized_b,
                 0.2126729 * normalized_r + 0.7151522 * normalized_g
                     + 0.0721750 * normalized_b,
                 0.0193339 * normalized_r + 0.1191920 * normalized_g
                     + 0.9503041 * normalized_b };
    }

    std::tuple<double, double, double>
    ColorConverter::xyz_to_lab(const std::tuple<double, double, double>& xyz)
    {
        double x = std::get<0>(xyz) / 0.95047;
        double y = std::get<1>(xyz);
        double z = std::get<2>(xyz) / 1.08883;

        double lab_transform_x = lab_transform(x);
        double lab_transform_y = lab_transform(y);
        double lab_transform_z = lab_transform(z);

        return { 116 * lab_transform_y - 16,
                 500 * (lab_transform_x - lab_transform_y),
                 200 * (lab_transform_y - lab_transform_z) };
    }

    std::tuple<double, double, double>
    ColorConverter::rgb_to_lab(const std::tuple<double, double, double>& rgb)
    {
        return xyz_to_lab(rgb_to_xyz(rgb));
    }

    double ColorConverter::gamma_correction(double color_intensity)
    {
        if (color_intensity > 0.04045)
        {
            return std::pow((color_intensity + 0.055) / 1.055, 2.4);
        }
        return color_intensity / 12.92;
    }

    double ColorConverter::lab_transform(double xyz_intensity)
    {
        if (xyz_intensity > std::pow(6. / 29., 3.))
        {
            return std::pow(xyz_intensity, 1. / 3.);
        }
        return 841. / 108. * xyz_intensity + 4. / 29.; // 1/3 * (29/6)^2
    }

} // namespace tifo::metrics::convert