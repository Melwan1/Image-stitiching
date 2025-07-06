#pragma once

#include <math/matrix.hh>
#include <panorama/sift/key-point.hh>

namespace tifo::panorama::sift
{

    struct Match
    {
        int idx1;
        int idx2;
        float distance;

        Match() = default;
        Match(int idx1, int idx2, float distance)
            : idx1(idx1)
            , idx2(idx2)
            , distance(distance)
        {}
    };

    class DescriptorMatcher
    {
    public:
        float descriptor_distance(const std::vector<float>& descriptor1,
                                  const std::vector<float>& descriptor2);
        std::vector<Match>
        match_descriptors(const std::vector<KeyPoint>& keypoints1,
                          const std::vector<KeyPoint>& keypoints2,
                          float ratio_threshold = 0.75f);

        std::vector<Match> random_pick(const std::vector<Match>& matches);

        const math::Matrix3
        compute_homography_DLT(const std::vector<Match>& random_sample);
        std::pair<float, float>
        apply_homography(const math::Matrix3& homography, int x, int y);
        math::Matrix3 compute_homography(const std::vector<Match>& matches);

    private:
        std::vector<KeyPoint> keypoints1_;
        std::vector<KeyPoint> keypoints2_;
        std::vector<Match> matches_;
    };

} // namespace tifo::panorama::sift