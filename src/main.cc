#include <config/config-launcher.hh>
#include <images/color-ppm-image.hh>
#include <images/grayscale-ppm-image.hh>
#include <iostream>
#include <panorama/sift/descriptor-matching.hh>
#include <panorama/sift/sift.hh>

int main()
{
    // tifo::config::ConfigLauncher config_launcher("config.yaml");

    tifo::image::ColorPPMImage input1, input2;
    tifo::panorama::sift::SIFT sift;
    std::cout << "loading image...\n";

    input1.read("tests/bedroom2.ppm");
    input2.read("tests/bedroom1.ppm");
    std::cout << "Detecting SIFT features...\n";
    auto keypoints1 = sift.detect_and_compute(input1.to_grayscale());
    auto keypoints2 = sift.detect_and_compute(input2.to_grayscale());

    std::cout << "Found " << keypoints1.size() << " keypoints\n";

    tifo::panorama::sift::DescriptorMatcher matcher;
    matcher.match_descriptors(keypoints1, keypoints2);

    return 0;
}