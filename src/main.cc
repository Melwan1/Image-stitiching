#include <config/config-launcher.hh>
#include <images/color-ppm-image.hh>
#include <images/grayscale-ppm-image.hh>
#include <iostream>
#include <panorama/sift/sift.hh>

int main()
{
    // tifo::config::ConfigLauncher config_launcher("config.yaml");

    tifo::image::ColorPPMImage input_image;
    tifo::panorama::sift::SIFT sift;
    std::cout << "loading image...\n";

    input_image.read("julie2.ppm");
    tifo::image::GrayscaleImage* grayscale_image = input_image.to_grayscale();

    std::cout << "Detecting SIFT features...\n";
    auto keypoints = sift.detect_and_compute(grayscale_image);

    std::cout << "Found " << keypoints.size() << " keypoints\n";

    sift.save_keypoints(keypoints, "keypoints.txt");
    std::cout << "Keypoints saved to keypoints.txt\n";

    return 0;
}