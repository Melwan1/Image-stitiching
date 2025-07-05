#include <config/config-launcher.hh>
#include <filters/gaussian.hh>
#include <filters/laplacian.hh>
#include <filters/sobel.hh>
#include <images/color-ppm-image.hh>
#include <images/grayscale-ppm-image.hh>

int main()
{
    // tifo::config::ConfigLauncher config_launcher("config.yaml");

    tifo::image::ColorPPMImage input_image;
    input_image.read("./tests/julie2.ppm");
    tifo::filter::Sobel sobel;
    tifo::image::GrayscaleImage* result =
        sobel.apply_on_image(input_image.to_grayscale());
    result->write("sobel.ppm");
    tifo::filter::Laplacian laplacian;
    tifo::image::GrayscaleImage* result_laplacian =
        laplacian.apply_on_image(input_image.to_grayscale());
    result_laplacian->write("laplacian.ppm");

    tifo::filter::GaussianFilter<2> gaussian_filter(1);

    auto downsampled = input_image.to_grayscale()->downsample(2);
    auto blurred_downsampled = gaussian_filter.apply_on_image(downsampled);
    downsampled->write("downsampled.ppm");
    blurred_downsampled->write("blurred.ppm");

    return 0;
}