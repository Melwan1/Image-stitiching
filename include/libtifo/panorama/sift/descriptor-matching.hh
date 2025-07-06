#pragma once

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
    };

} // namespace tifo::panorama::sift