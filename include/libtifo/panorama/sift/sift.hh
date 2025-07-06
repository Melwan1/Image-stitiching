#pragma once

#include <algorithm>
#include <cmath>
#include <images/grayscale-image.hh>
#include <panorama/sift/key-point.hh>
#include <vector>

namespace tifo::panorama::sift
{

    constexpr float SIGMA = 1.6f;
    constexpr unsigned GAUSSIAN_FILTER_SIZE =
        11; // 2 * std::ceil(3 * SIGMA) + 1

    class SIFT
    {
    public:
        SIFT() = default;

        std::vector<std::vector<image::GrayscaleImage*>>
        build_gaussian_pyramid(image::GrayscaleImage* image);
        std::vector<std::vector<image::GrayscaleImage*>> build_dog_pyramid(
            const std::vector<std::vector<image::GrayscaleImage*>>& gaussian);
        bool
        is_extremum(const std::vector<std::vector<image::GrayscaleImage*>>& dog,
                    int octave, int scale, int x, int y);
        std::pair<float, float>
        compute_gradient(const image::GrayscaleImage* image, int x, int y);
        float compute_dominant_orientation(const image::GrayscaleImage* image,
                                           int x, int y, float scale);
        std::vector<float>
        compute_descriptor(const image::GrayscaleImage* image,
                           const KeyPoint& keypoint);

        // main function
        std::vector<KeyPoint> detect_and_compute(image::GrayscaleImage* image);

        void save_keypoints(const std::vector<KeyPoint>& keypoints,
                            const fs::path& save_path);

    private:
        const int octaves_ = 4;
        const int scales_ = 5;
        const float k_ = std::sqrt(2.0f);
        const float contrast_threshold = 0.04f;
    };

} // namespace tifo::panorama::sift