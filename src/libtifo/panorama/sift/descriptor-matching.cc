#include <algorithm>
#include <iostream>
#include <math/vector.hh>
#include <panorama/sift/descriptor-matching.hh>
#include <set>

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
        keypoints1_ = keypoints1;
        keypoints2_ = keypoints2;
        matches_ = matches;

        return matches;
    }

    std::vector<Match>
    DescriptorMatcher::random_pick(const std::vector<Match>& matches)
    {
        std::set<int> indices;
        while (indices.size() < 4)
        {
            indices.emplace(std::rand() * matches.size() / RAND_MAX);
        }
        std::vector<Match> result;
        for (int index : indices)
        {
            std::cout << index << "\n";
            result.push_back(matches[index]);
        }
        return result;
    }

    const math::Matrix3 DescriptorMatcher::compute_homography_DLT(
        const std::vector<Match>& random_sample)
    {
        std::vector<std::vector<double>> A;
        for (unsigned index = 0; index < random_sample.size(); index++)
        {
            double x = keypoints1_[random_sample[index].idx1].x;
            double y = keypoints1_[random_sample[index].idx1].y;
            double xp = keypoints2_[random_sample[index].idx2].x;
            double yp = keypoints2_[random_sample[index].idx2].y;
            std::cout << "x = " << x << ", y = " << y << ", x' = " << xp
                      << ", y' = " << yp << "\n";

            A.push_back({ -x, -y, -1, 0, 0, 0, x * xp, y * xp, xp });
            A.push_back({ 0, 0, 0, -x, -y, -1, x * yp, y * yp, yp });
        }
        std::vector<std::vector<double>> AtA;
        AtA.resize(9);
        for (int index = 0; index < 9; index++)
        {
            AtA[index].resize(9);
            for (int sub_index = 0; sub_index < 9; sub_index++)
            {
                AtA[index][sub_index] = 0;
            }
        }
        for (const auto& row : A)
        {
            for (int i = 0; i < 9; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    AtA[i][j] += row[i] * row[j];
                }
            }
        }

        math::Vector<double, 9> h;
        for (int i = 0; i < 9; i++)
        {
            h[i] = 0;
        }
        h = h.normalize();

        for (int iter = 0; iter < 100; iter++)
        {
            math::Vector<double, 9> h_new;
            for (int i = 0; i < 9; i++)
            {
                h_new[i] = 0;
            }
            for (int i = 0; i < 9; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    h_new[i] = AtA[i][j] * h[j];
                }
            }
            h_new = h_new.normalize();
            h = h_new;
        }

        math::Matrix3 H;
        for (int i = 0; i < 9; i++)
        {
            H(i / 3, i % 3) = h[i] / h[8]; // normalization
        }

        return H;
    }

    std::pair<float, float>
    DescriptorMatcher::apply_homography(const math::Matrix3& homography, int x,
                                        int y)
    {
        math::Vector3 result = homography
            * math::Vector3({ static_cast<float>(x), static_cast<float>(y),
                              1 });
        return { result[0] / result[2], result[1] / result[2] };
    }

    math::Matrix3
    DescriptorMatcher::compute_homography(const std::vector<Match>& matches)
    {
        std::vector<Match> best_inliers;
        int best_inliers_count;

        for (int iter = 0; iter < 1000; iter++)
        {
            std::vector<Match> sample = random_pick(matches);
            math::Matrix3 H = compute_homography_DLT(sample);
            std::cout << H << "\n";

            int inliers = 0;
            std::vector<Match> inlier_matches;

            for (const auto& m : matches)
            {
                std::pair<int, int> p1 = { keypoints1_[m.idx1].x,
                                           keypoints1_[m.idx1].y };
                std::pair<int, int> p2 = { keypoints2_[m.idx2].x,
                                           keypoints2_[m.idx2].y };
                std::pair<float, float> projected =
                    apply_homography(H, p1.first, p1.second);
                float distance = std::sqrt(
                    (projected.first - p2.first) * (projected.first - p2.first)
                    + (projected.second - p2.second)
                        * (projected.second - p2.second));
                if (distance < 3)
                {
                    inliers++;
                    inlier_matches.emplace_back(m);
                }
            }
            if (inliers > best_inliers_count)
            {
                best_inliers = inlier_matches;
                best_inliers_count = inliers;
            }
        }

        return compute_homography_DLT(best_inliers);
    }

} // namespace tifo::panorama::sift