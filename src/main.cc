#include <config/config-launcher.hh>
#include <images/color-ppm-image.hh>
#include <images/grayscale-ppm-image.hh>
#include <iostream>
#include <math/matrix.hh>
#include <panorama/sift/descriptor-matching.hh>
#include <panorama/sift/sift.hh>
#include <panorama/cutter/overlap-rectangular-cutter.hh>

int main()
{
    // tifo::config::ConfigLauncher config_launcher("config.yaml");

    tifo::image::ColorPPMImage input;
    tifo::panorama::sift::SIFT sift;
    std::cout << "loading image...\n";
    input.read("tests/landscape_small.ppm");

    std::vector<tifo::image::ColorImage*> images;
    tifo::panorama::cutter::OverlapRectangularCutter cutter;
    cutter.set_input_image(&input);
    cutter.set_horizontal_overlap_size(200);
    cutter.set_vertical_overlap_size(20);
    cutter.set_horizontal_slices(2);
    cutter.set_vertical_slices(1);
    images = cutter.cut();
    images[0]->write("test1.ppm");
    images[1]->write("test2.ppm");

    std::cout << "Detecting SIFT features...\n";
    auto keypoints1 = sift.detect_and_compute(images[0]->to_grayscale());
    auto keypoints2 = sift.detect_and_compute(images[1]->to_grayscale());

    std::cout << "Found " << keypoints1.size() << " keypoints\n";

    tifo::panorama::sift::DescriptorMatcher matcher;
    std::vector<tifo::panorama::sift::Match> matches =
        matcher.robust_matching(keypoints1, keypoints2);
    tifo::image::ColorImage* final_result = matcher.stitch(images[0], images[1]);
    final_result->write("final.ppm");

    return 0;
}