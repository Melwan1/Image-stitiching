#pragma once

#include <images/color-image.hh>
#include <math/matrix.hh>
#include <panorama/sift/key-point.hh>

namespace tifo::panorama::sift
{

    struct Match
    {
        int idx1;
        int idx2;
        float distance;
        std::pair<float, float> pt1;
        std::pair<float, float> pt2;

        Match() = default;
        Match(int idx1, int idx2, float distance, std::pair<float, float> pt1,
              std::pair<float, float> pt2)
            : idx1(idx1)
            , idx2(idx2)
            , distance(distance)
            , pt1(pt1)
            , pt2(pt2)
        {}
    };

    class DescriptorMatcher
    {
    public:
        float descriptor_distance(const std::vector<float>& descriptor1,
                                  const std::vector<float>& descriptor2);

        std::pair<int, int> find_two_nearest_neighbors(
            const std::vector<float>& query_desc,
            const std::vector<KeyPoint>& target_keypoints);
        std::vector<Match>
        forward_matching(const std::vector<KeyPoint>& keypoints1,
                         const std::vector<KeyPoint>& keypoints2);
        std::vector<Match>
        backward_matching(const std::vector<KeyPoint>& keypoints1,
                          const std::vector<KeyPoint>& keypoints2);
        std::vector<Match>
        cross_check_matching(const std::vector<KeyPoint>& keypoints1,
                             const std::vector<KeyPoint>& keypoints2);
        std::vector<Match>
        geometric_verification(const std::vector<Match>& matches);
        std::vector<Match>
        robust_matching(const std::vector<KeyPoint>& keypoints1,
                        const std::vector<KeyPoint>& keypoints2);
        std::vector<Match>
        match_descriptors(const std::vector<KeyPoint>& keypoints1,
                          const std::vector<KeyPoint>& keypoints2,
                          float ratio_threshold = 0.65f);

        std::vector<Match> random_pick(const std::vector<Match>& matches);

        math::Matrix3 make_hartley_norm(const std::vector<KeyPoint>& keypoints,
                                        const std::vector<int>& indices);
        void compute_point_normalization();
        const math::Matrix3
        compute_homography_minimal_DLT(const std::vector<Match>& random_sample);
        const math::Matrix3
        compute_homography_full_DLT(const std::vector<Match>& inliers);
        std::pair<float, float>
        apply_homography(const math::Matrix3& homography,
                         const math::Vector3& normalized_point);
        math::Matrix3 compute_homography(const std::vector<Match>& matches);
        std::vector<std::pair<float, float>>
        warp_corners(const math::Matrix3& H, int width, int height);
        std::vector<float> bilinear_sample(const image::ColorImage* image,
                                           float x, float y);
        image::ColorImage* stitch(const image::ColorImage* image1,
                                  const image::ColorImage* image2);

    private:
        std::vector<KeyPoint> keypoints1_;
        std::vector<KeyPoint> keypoints2_;
        std::vector<Match> matches_;
        math::Matrix3 normalization_matrix1_;
        math::Matrix3 normalization_matrix2_;

        float ratio_threshold = 0.7f;
        float max_descriptor_distance = 0.15f;
        float geometric_threshold = 50.0f;
    };

} // namespace tifo::panorama::sift