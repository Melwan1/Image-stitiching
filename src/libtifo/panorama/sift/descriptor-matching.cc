#include <iostream>
#include <panorama/sift/descriptor-matching.hh>

namespace tifo::panorama::sift
{

    float DescriptorMatcher::descriptor_distance(
        const std::vector<float>& descriptor1,
        const std::vector<float>& descriptor2)
    {
        float distance = 0.0f;
        for (unsigned i = 0; i < descriptor1.size(); i++)
        {
            float difference = descriptor1[i] - descriptor2[i];
            distance += difference * difference;
        }
        return std::sqrt(distance);
    }

    std::vector<Match> DescriptorMatcher::match_descriptors(
        const std::vector<KeyPoint>& keypoints1,
        const std::vector<KeyPoint>& keypoints2, float ratio_threshold)
    {
        std::vector<Match> matches;

        for (unsigned i = 0; i < keypoints1.size(); i++)
        {
            const auto& desc1 = keypoints1[i].descriptor;

            float best_distance = std::numeric_limits<float>::max();
            float second_best_distance = std::numeric_limits<float>::max();
            int best_index = -1;

            for (unsigned j = 0; j < keypoints2.size(); j++)
            {
                const auto& desc2 = keypoints2[j].descriptor;
                float distance = descriptor_distance(desc1, desc2);

                if (distance < best_distance)
                {
                    second_best_distance = best_distance;
                    best_distance = distance;
                    best_index = j;
                }
                else if (distance < second_best_distance)
                {
                    second_best_distance = distance;
                }
            }

            // Lowe's ratio test
            if (best_distance < ratio_threshold * second_best_distance
                && best_distance <= 0.25)
            {
                matches.emplace_back(
                    Match(static_cast<int>(i), best_index, best_distance));
            }
        }

        for (const auto& match : matches)
        {
            auto& kp1 = keypoints1[match.idx1];
            auto& kp2 = keypoints2[match.idx2];
            std::cout << "Match: (" << kp1.x << ", " << kp1.y << ") <-> ("
                      << kp2.x << ", " << kp2.y
                      << "), dist = " << match.distance << '\n';
        }

        return matches;
    }

} // namespace tifo::panorama::sift