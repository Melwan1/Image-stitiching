#include <config/config-launcher.hh>
#include <images/color-ppm-image.hh>
#include <images/grayscale-ppm-image.hh>

int main()
{
    tifo::config::ConfigLauncher config_launcher("config.yaml");

    tifo::image::ColorPPMImage color_image;
    color_image.read("./tests/julie2.ppm");
    color_image.to_grayscale()->write("grayscale.ppm");

    return 0;
}