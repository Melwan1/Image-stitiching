#include <algorithm>
#include <iostream>
#include <math/vector.hh>
#include <panorama/sift/descriptor-matching.hh>
#include <random>
#include <unordered_set>

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

        compute_point_normalization();

        return matches;
    }

    std::vector<Match>
    DescriptorMatcher::random_pick(const std::vector<Match>& matches)
    {
        std::unordered_set<int> indices;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, matches.size() - 1);

        while (indices.size() < 4)
        {
            indices.insert(dist(gen));
        }
        std::vector<Match> result;
        for (int index : indices)
        {
            result.push_back(matches[index]);
        }
        return result;
    }

    const math::Matrix3 DescriptorMatcher::compute_homography_minimal_DLT(
        const std::vector<Match>& random_sample)
    {
        std::vector<std::vector<double>> A;
        for (unsigned index = 0; index < random_sample.size(); index++)
        {
            double x = keypoints1_[random_sample[index].idx1].x;
            double y = keypoints1_[random_sample[index].idx1].y;
            double xp = keypoints2_[random_sample[index].idx2].x;
            double yp = keypoints2_[random_sample[index].idx2].y;

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
            h[i] = 1;
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
                    h_new[i] += AtA[i][j] * h[j];
                }
            }
            h_new = h_new.normalize();
            h = h_new;
        }

        math::Matrix3 H;
        for (int i = 0; i < 9; i++)
        {
            H(i / 3, i % 3) = h[i]; // normalization
        }

        return H;
    }

    const math::Matrix3 DescriptorMatcher::compute_homography_full_DLT(
        const std::vector<Match>& inliers)
    {
        std::vector<std::vector<double>> A;

        for (const auto& match : inliers)
        {
            math::Vector3 p1 = normalization_matrix1_
                * math::Vector3({ keypoints1_[match.idx1].x,
                                  keypoints1_[match.idx1].y, 1 });
            math::Vector3 p2 = normalization_matrix2_
                * math::Vector3({ keypoints2_[match.idx2].x,
                                  keypoints2_[match.idx2].y, 1 });
            double x = p1[0];
            double y = p1[1];
            double xp = p2[0];
            double yp = p2[1];

            A.push_back({ -x, -y, -1, 0, 0, 0, x * xp, y * xp, xp });
            A.push_back({ 0, 0, 0, -x, -y, -1, x * yp, y * yp, yp });
        }

        std::vector<std::vector<double>> AtA(9, std::vector<double>(9, 0.0));
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
            h[i] = 1.0;
        }
        h = h.normalize();

        for (int iter = 0; iter < 100; iter++)
        {
            math::Vector<double, 9> h_new;
            for (int i = 0; i < 9; i++)
            {
                h_new[i] = 0;
                for (int j = 0; j < 9; j++)
                {
                    h_new[i] += AtA[i][j] * h[j];
                }
            }
            h = h_new.normalize();
        }

        math::Matrix3 H;
        for (int i = 0; i < 9; i++)
        {
            H(i / 3, i % 3) = h[i];
        }

        return H;
    }

    std::pair<float, float>
    DescriptorMatcher::apply_homography(const math::Matrix3& homography,
                                        const math::Vector3& normalized_point)
    {
        math::Vector3 result = homography * normalized_point;
        return { result[0] / result[2], result[1] / result[2] };
    }

    math::Matrix3
    DescriptorMatcher::compute_homography(const std::vector<Match>& matches)
    {
        std::vector<Match> best_inliers;
        int best_inliers_count = -1;

        for (int iter = 0; iter < 1000; iter++)
        {
            std::vector<Match> sample = random_pick(matches);
            math::Matrix3 H = compute_homography_minimal_DLT(sample);

            int inliers = 0;
            std::vector<Match> inlier_matches;

            for (const auto& m : matches)
            {
                math::Vector3 t1p1 = normalization_matrix1_
                    * math::Vector3({ keypoints1_[m.idx1].x,
                                      keypoints1_[m.idx1].y, 1 });
                math::Vector3 t2p2 = normalization_matrix2_
                    * math::Vector3({ keypoints2_[m.idx2].x,
                                      keypoints2_[m.idx2].y, 1 });
                std::pair<float, float> projected = apply_homography(H, t1p1);
                float distance = std::sqrt((projected.first - t2p2[0])
                                               * (projected.first - t2p2[0])
                                           + (projected.second - t2p2[1])
                                               * (projected.second - t2p2[1]));
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

        float scale_factor2 = normalization_matrix2_(0, 0);
        float translation_x2 = normalization_matrix2_(0, 2);
        float translation_y2 = normalization_matrix2_(1, 2);

        math::Matrix3 normalization_matrix2_inverse = {
            { 1.f / scale_factor2, 0, -translation_x2 / scale_factor2 },
            { 0, 1.f / scale_factor2, -translation_y2 / scale_factor2 },
            { 0, 0, 1 }
        };

        math::Matrix3 denormalized = normalization_matrix2_inverse
            * compute_homography_full_DLT(best_inliers)
            * normalization_matrix1_;
        for (int i = 0; i < 9; i++)
        {
            denormalized(i / 3, i % 3) /= denormalized(2, 2);
        }
        return denormalized;
    }

    void DescriptorMatcher::compute_point_normalization()
    {
        float x1_centroid = 0;
        float y1_centroid = 0;
        float x2_centroid = 0;
        float y2_centroid = 0;
        for (const auto& match : matches_)
        {
            x1_centroid += keypoints1_[match.idx1].x;
            y1_centroid += keypoints1_[match.idx1].y;
            x2_centroid += keypoints2_[match.idx2].x;
            y2_centroid += keypoints2_[match.idx2].y;
        }
        x1_centroid /= matches_.size();
        y1_centroid /= matches_.size();
        x2_centroid /= matches_.size();
        y2_centroid /= matches_.size();

        float avg_distance1 = 0;
        float avg_distance2 = 0;
        for (const auto& match : matches_)
        {
            avg_distance1 +=
                std::sqrt((keypoints1_[match.idx1].x - x1_centroid)
                              * (keypoints1_[match.idx1].x - x1_centroid)
                          + (keypoints1_[match.idx1].y - y1_centroid)
                              * (keypoints1_[match.idx1].y - y1_centroid));
            avg_distance2 +=
                std::sqrt((keypoints2_[match.idx2].x - x2_centroid)
                              * (keypoints2_[match.idx2].x - x2_centroid)
                          + (keypoints2_[match.idx2].y - y2_centroid)
                              * (keypoints2_[match.idx2].y - y2_centroid));
        }
        avg_distance1 = avg_distance1 / matches_.size();
        avg_distance2 = avg_distance2 / matches_.size();

        float scale_factor1 = std::sqrt(2) / avg_distance1;
        float scale_factor2 = std::sqrt(2) / avg_distance2;

        normalization_matrix1_ = {
            { scale_factor1, 0, -scale_factor1 * x1_centroid },
            { 0, scale_factor1, -scale_factor1 * y1_centroid },
            { 0, 0, 1 }
        };
        normalization_matrix2_ = {
            { scale_factor2, 0, -scale_factor2 * x2_centroid },
            { 0, scale_factor2, -scale_factor2 * y2_centroid },
            { 0, 0, 1 }
        };
    }

} // namespace tifo::panorama::sift