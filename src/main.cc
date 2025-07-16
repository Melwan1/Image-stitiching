#include <config/config-launcher.hh>
#include <images/color-ppm-image.hh>
#include <images/grayscale-ppm-image.hh>
#include <iostream>
#include <math/matrix.hh>
#include <panorama/cutter/overlap-rectangular-cutter.hh>
#include <panorama/sift/descriptor-matching.hh>
#include <panorama/sift/sift.hh>

int main()
{
    // tifo::config::ConfigLauncher config_launcher("config.yaml");

    tifo::panorama::sift::SIFT sift;

    tifo::image::ColorPPMImage bedroom2, bedroom1;
    bedroom2.read("tests/front_01.ppm");
    bedroom1.read("tests/front_02.ppm");

    std::cout << "Detecting SIFT features...\n";
    auto keypoints1 = sift.detect_and_compute(bedroom2.to_grayscale());

    /*for (const auto& keypoint : keypoints1)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (bedroom2.is_valid_access(keypoint.x + dx, keypoint.y + dy)) {
                    bedroom2(keypoint.x + dx, keypoint.y + dy)[0] = 1;
                    bedroom2(keypoint.x + dx, keypoint.y + dy)[1] = 0;
                    bedroom2(keypoint.x + dx, keypoint.y + dy)[2] = 1;
                }
            }
        }
    }
    bedroom2.write("front_01_kp.ppm");*/

    auto keypoints2 = sift.detect_and_compute(bedroom1.to_grayscale());

    /*for (const auto& keypoint : keypoints2)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (bedroom1.is_valid_access(keypoint.x + dx, keypoint.y + dy)) {

                    bedroom1(keypoint.x + dx, keypoint.y + dy)[0] = 1;
                    bedroom1(keypoint.x + dx, keypoint.y + dy)[1] = 0;
                    bedroom1(keypoint.x + dx, keypoint.y + dy)[2] = 0;
                }
            }
        }
    }

    bedroom1.write("front_02_kp.ppm");*/

    tifo::panorama::sift::DescriptorMatcher matcher;
    std::vector<tifo::panorama::sift::Match> matches =
        matcher.robust_matching(keypoints1, keypoints2);
    tifo::image::ColorImage* final_result =
        matcher.stitch(&bedroom2, &bedroom1);
    final_result->write("final.ppm");

    return 0;
}