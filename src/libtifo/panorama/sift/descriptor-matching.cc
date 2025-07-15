#include <algorithm>
#include <images/color-ppm-image.hh>
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

    std::pair<int, int> DescriptorMatcher::find_two_nearest_neighbors(
        const std::vector<float>& query_desc,
        const std::vector<KeyPoint>& target_keypoints)
    {
        float min_dist1 = 1000.0f;
        float min_dist2 = 1000.0f;
        int best_idx = -1;
        int second_best_idx = -1;
        for (unsigned i = 0; i < target_keypoints.size(); i++)
        {
            float distance =
                descriptor_distance(query_desc, target_keypoints[i].descriptor);
            if (distance < min_dist1)
            {
                min_dist2 = min_dist1;
                second_best_idx = best_idx;
                min_dist1 = distance;
                best_idx = i;
            }
            else if (distance < min_dist2)
            {
                min_dist2 = distance;
                second_best_idx = i;
            }
        }

        return std::make_pair(best_idx, second_best_idx);
    }

    std::vector<Match>
    DescriptorMatcher::forward_matching(const std::vector<KeyPoint>& keypoints1,
                                        const std::vector<KeyPoint>& keypoints2)
    {
        std::vector<Match> matches;

        for (unsigned i = 0; i < keypoints1.size(); i++)
        {
            auto [best_idx, second_best_idx] = find_two_nearest_neighbors(
                keypoints1[i].descriptor, keypoints2);
            if (best_idx == -1 || second_best_idx == -1)
            {
                continue;
            }

            float best_dist = descriptor_distance(
                keypoints1[i].descriptor, keypoints2[best_idx].descriptor);
            float second_best_dist =
                descriptor_distance(keypoints1[i].descriptor,
                                    keypoints2[second_best_idx].descriptor);

            if (best_dist < ratio_threshold * second_best_dist
                && best_dist < max_descriptor_distance)
            {
                Match match;
                match.idx1 = i;
                match.idx2 = best_idx;
                match.distance = best_dist;
                match.pt1 = { keypoints1[i].x, keypoints1[i].y };
                match.pt2 = { keypoints2[best_idx].x, keypoints2[best_idx].y };
                matches.push_back(match);
            }
        }
        return matches;
    }
    std::vector<Match> DescriptorMatcher::backward_matching(
        const std::vector<KeyPoint>& keypoints1,
        const std::vector<KeyPoint>& keypoints2)
    {
        std::vector<Match> matches;

        for (unsigned i = 0; i < keypoints2.size(); i++)
        {
            auto [best_idx, second_best_idx] = find_two_nearest_neighbors(
                keypoints2[i].descriptor, keypoints1);
            if (best_idx == -1 || second_best_idx == -1)
            {
                continue;
            }

            float best_dist = descriptor_distance(
                keypoints2[i].descriptor, keypoints1[best_idx].descriptor);
            float second_best_dist =
                descriptor_distance(keypoints2[i].descriptor,
                                    keypoints1[second_best_idx].descriptor);

            if (best_dist < ratio_threshold * second_best_dist
                && best_dist < max_descriptor_distance)
            {
                Match match;
                match.idx1 = best_idx;
                match.idx2 = i;
                match.distance = best_dist;
                match.pt1 = { keypoints1[best_idx].x, keypoints1[best_idx].y };
                match.pt2 = { keypoints2[i].x, keypoints2[i].y };
                matches.push_back(match);
            }
        }
        return matches;
    }

    std::vector<Match> DescriptorMatcher::cross_check_matching(
        const std::vector<KeyPoint>& keypoints1,
        const std::vector<KeyPoint>& keypoints2)
    {
        std::vector<Match> forward_matches =
            forward_matching(keypoints1, keypoints2);
        std::vector<Match> backward_matches =
            backward_matching(keypoints1, keypoints2);
        std::vector<Match> cross_checked_matches;

        for (const auto& forward_match : forward_matches)
        {
            for (const auto& backward_match : backward_matches)
            {
                if (forward_match.idx1 == backward_match.idx1
                    && forward_match.idx2 == backward_match.idx2)
                {
                    cross_checked_matches.push_back(forward_match);
                    break;
                }
            }
        }

        std::cout << "Forward matches: " << forward_matches.size() << "\n";
        std::cout << "Backward matches: " << backward_matches.size() << "\n";
        std::cout << "Cross-checked matches: " << cross_checked_matches.size()
                  << "\n";

        return cross_checked_matches;
    }

    std::vector<Match>
    DescriptorMatcher::geometric_verification(const std::vector<Match>& matches)
    {
        if (matches.size() < 4)
        {
            return matches;
        }

        std::vector<float> dx_values, dy_values;
        for (const auto& match : matches)
        {
            dx_values.push_back(match.pt2.first - match.pt1.first);
            dy_values.push_back(match.pt2.second - match.pt1.second);
        }

        std::sort(dx_values.begin(), dx_values.end());
        std::sort(dy_values.begin(), dy_values.end());

        float median_dx = dx_values[dx_values.size() / 2];
        float median_dy = dy_values[dy_values.size() / 2];

        std::cout << "Median displacement: " << "dx = " << median_dx
                  << ", dy = " << median_dy << "\n";

        std::vector<Match> geometrically_consistent_matches;

        for (const auto& match : matches)
        {
            float dx = match.pt2.first - match.pt1.first;
            float dy = match.pt2.second - match.pt1.second;

            float dx_diff = std::abs(dx - median_dx);
            float dy_diff = std::abs(dy - median_dy);

            if (dx_diff < geometric_threshold && dy_diff < geometric_threshold)
            {
                geometrically_consistent_matches.push_back(match);
                std::cout << "Keep match: (" << match.pt1.first << ", "
                          << match.pt1.second << ") <--> (" << match.pt2.first
                          << ", " << match.pt2.second << "), dx = " << dx
                          << ", dy = " << dy << "\n";
            }
            else
            {
                std::cout << "Rejecting match: (" << match.pt1.first << ", "
                          << match.pt1.second << ") <--> (" << match.pt2.first
                          << ", " << match.pt2.second << "), dx = " << dx
                          << ", dy = " << dy << "\n";
            }
        }

        std::cout << "Geometrically consistent matches: "
                  << geometrically_consistent_matches.size() << " out of "
                  << matches.size() << "\n";

        return geometrically_consistent_matches;
    }

    std::vector<Match>
    DescriptorMatcher::robust_matching(const std::vector<KeyPoint>& keypoints1,
                                       const std::vector<KeyPoint>& keypoints2)
    {
        std::cout << "=== ROBUST SIFT MATCHING PIPELINE ===\n";
        std::cout << "Keypoints1: " << keypoints1.size()
                  << ", Keypoints2: " << keypoints2.size() << "\n";
        std::vector<Match> cross_checked =
            cross_check_matching(keypoints1, keypoints2);
        std::vector<Match> final_matches =
            geometric_verification(cross_checked);
        std::cout << "Final matches: " << final_matches.size() << "\n";

        keypoints1_ = keypoints1;
        keypoints2_ = keypoints2;

        matches_ = final_matches;

        compute_point_normalization();

        return final_matches;
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
            int best_idx = -1;

            for (unsigned j = 0; j < keypoints2.size(); j++)
            {
                const auto& desc2 = keypoints2[j].descriptor;
                float distance = descriptor_distance(desc1, desc2);

                if (distance < best_distance)
                {
                    second_best_distance = best_distance;
                    best_distance = distance;
                    best_idx = j;
                }
                else if (distance < second_best_distance)
                {
                    second_best_distance = distance;
                }
            }

            // Lowe's ratio test
            if (best_distance < ratio_threshold * second_best_distance
                && best_distance <= 0.12)
            {
                matches.emplace_back(
                    Match(static_cast<int>(i), best_idx, best_distance,
                          { keypoints1[i].x, keypoints1[i].y },
                          { keypoints2[best_idx].x, keypoints2[best_idx].y }));
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
        (void)rd;
        std::mt19937 gen(2);
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
            math::Vector3 p1_normalized = normalization_matrix1_
                * math::Vector3({ keypoints1_[random_sample[index].idx1].x,
                                  keypoints1_[random_sample[index].idx1].y,
                                  1 });
            math::Vector3 p2_normalized = normalization_matrix2_
                * math::Vector3({ keypoints2_[random_sample[index].idx2].x,
                                  keypoints2_[random_sample[index].idx2].y,
                                  1 });
            double x = p1_normalized[0];
            double y = p1_normalized[1];
            double xp = p2_normalized[0];
            double yp = p2_normalized[1];

            A.push_back({ x, y, 1, 0, 0, 0, -x * xp, -y * xp, -xp });
            A.push_back({ 0, 0, 0, x, y, 1, -x * yp, -y * yp, -yp });
        }

        math::SquaredMatrix<double, 9> AtA;
        for (int index = 0; index < 9; index++)
        {
            for (int sub_index = 0; sub_index < 9; sub_index++)
            {
                AtA(index, sub_index) = 0;
            }
        }
        for (const auto& row : A)
        {
            for (int i = 0; i < 9; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    AtA(i, j) += row[i] * row[j];
                }
            }
        }
        math::Vector<double, 9> h;
        for (int i = 0; i < 9; i++)
        {
            h[i] = (i == 0) ? 1.0 : 0.0;
        }
        h = h.normalize();

        // Increase lambda for better numerical stability
        double lambda = 1e-1;
        math::SquaredMatrix<double, 9> AtA_shifted = AtA;
        for (unsigned i = 0; i < 9; i++)
        {
            AtA_shifted(i, i) += lambda;
        }

        math::SquaredMatrix<double, 9> inv = AtA_shifted.inverse();

        // More iterations and convergence check
        for (int iter = 0; iter < 1000; iter++)
        {
            math::Vector<double, 9> h_old = h;

            // Apply inverse matrix
            math::Vector<double, 9> h_new;
            for (int i = 0; i < 9; i++)
            {
                h_new[i] = 0;
                for (int j = 0; j < 9; j++)
                {
                    h_new[i] += inv(i, j) * h[j];
                }
            }

            h_new = h_new.normalize();

            // Check convergence
            double diff = 0;
            for (int i = 0; i < 9; i++)
            {
                diff += (h_new[i] - h_old[i]) * (h_new[i] - h_old[i]);
            }
            if (sqrt(diff) < 1e-10)
                break;

            h = h_new;
        }

        math::Matrix3 H;
        for (int i = 0; i < 9; i++)
        {
            H(i / 3, i % 3) = h[i] / h[8]; // normalization
        }
        std::cout << "H:\n" << H << "\n";

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

            A.push_back({ x, y, 1, 0, 0, 0, -x * xp, -y * xp, -xp });
            A.push_back({ 0, 0, 0, x, y, 1, -x * yp, -y * yp, -yp });
        }

        math::SquaredMatrix<double, 9> AtA;
        for (const auto& row : A)
        {
            for (int i = 0; i < 9; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    AtA(i, j) += row[i] * row[j];
                }
            }
        }

        math::Vector<double, 9> h;
        for (int i = 0; i < 9; i++)
        {
            h[i] = 1.0;
        }
        h = h.normalize();

        double lambda = 1e-3;
        math::SquaredMatrix<double, 9> AtA_shifted = AtA;
        for (unsigned i = 0; i < 9; i++)
        {
            AtA_shifted(i, i) += lambda;
        }

        math::SquaredMatrix<double, 9> inv = AtA_shifted.inverse();

        for (int iter = 0; iter < 100; iter++)
        {
            math::Vector<double, 9> h_new;
            for (int i = 0; i < 9; i++)
            {
                h_new[i] = 0;
                for (int j = 0; j < 9; j++)
                {
                    h_new[i] += inv(i, j) * h[j];
                }
            }
            h = h_new.normalize();
        }

        math::Matrix3 H;
        for (int i = 0; i < 9; i++)
        {
            H(i / 3, i % 3) = h[i] / h[8];
        }

        // std::cout << "final H " << H << "\n";

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

        float scale_factor1 = normalization_matrix1_(0, 0);
        float translation_x1 = normalization_matrix1_(0, 2);
        float translation_y1 = normalization_matrix1_(1, 2);

        float scale_factor2 = normalization_matrix2_(0, 0);
        float translation_x2 = normalization_matrix2_(0, 2);
        float translation_y2 = normalization_matrix2_(1, 2);

        math::Matrix3 normalization_matrix1_inverse = {
            { 1.f / scale_factor1, 0, -translation_x1 / scale_factor1 },
            { 0, 1.f / scale_factor1, -translation_y1 / scale_factor1 },
            { 0, 0, 1 }
        };

        math::Matrix3 normalization_matrix2_inverse = {
            { 1.f / scale_factor2, 0, -translation_x2 / scale_factor2 },
            { 0, 1.f / scale_factor2, -translation_y2 / scale_factor2 },
            { 0, 0, 1 }
        };

        for (int iter = 0; iter < 1000; iter++)
        {
            std::cout << "iteration " << iter << "\r" << std::flush;
            std::vector<Match> sample = random_pick(matches);
            math::Matrix3 H = compute_homography_minimal_DLT(sample);
            int inliers = 0;
            std::vector<Match> inlier_matches;

            for (const auto& m : matches)
            {
                // std::cout << "normalized homography: " << H << "\n";
                // std::cout << "denormalized homography: " << H_denormalized
                //           << "\n";
                math::Vector3 t1p1 = normalization_matrix1_
                    * math::Vector3({ keypoints1_[m.idx1].x,
                                      keypoints1_[m.idx1].y, 1 });
                math::Vector3 t2p2 = normalization_matrix2_
                    * math::Vector3({ keypoints2_[m.idx2].x,
                                      keypoints2_[m.idx2].y, 1 });
                std::pair<float, float> projected = apply_homography(H, t2p2);
                float distance = std::sqrt((projected.first - t1p1[0])
                                               * (projected.first - t1p1[0])
                                           + (projected.second - t1p1[1])
                                               * (projected.second - t1p1[1]));
                // std::cout << "distance between (" << projected.first << ", "
                //           << projected.second << ") and " << t1p1 << "\n";
                // std::cout << "distance: " << distance << "\n";
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
            // std::cout << "inliers : " << inliers << "\n";
        }

        math::Matrix3 denormalized = normalization_matrix2_inverse
            * compute_homography_full_DLT(best_inliers)
            * normalization_matrix1_;
        for (int i = 0; i < 9; i++)
        {
            denormalized(i / 3, i % 3) /= denormalized(2, 2);
        }
        return denormalized;
    }

    math::Matrix3
    DescriptorMatcher::make_hartley_norm(const std::vector<KeyPoint>& keypoints,
                                         const std::vector<int>& indices)
    {
        float cx = 0;
        float cy = 0;
        for (int index : indices)
        {
            cx += keypoints[index].x;
            cy += keypoints[index].y;
        }
        cx /= indices.size();
        cy /= indices.size();

        float mean_distance = 0;
        for (int index : indices)
        {
            float dx = keypoints[index].x - cx;
            float dy = keypoints[index].y - cy;
            mean_distance += std::sqrt(dx * dx + dy * dy);
        }
        mean_distance /= indices.size();

        float scale = std::sqrt(2.0) / mean_distance;

        return { { scale, 0, -scale * cx },
                 { 0, scale, -scale * cy },
                 { 0, 0, 1 } };
    }

    void DescriptorMatcher::compute_point_normalization()
    {
        std::vector<int> indices1;
        std::vector<int> indices2;
        for (unsigned match_index = 0; match_index < matches_.size();
             match_index++)
        {
            indices1.emplace_back(matches_[match_index].idx1);
            indices2.emplace_back(matches_[match_index].idx2);
        }
        normalization_matrix1_ = make_hartley_norm(keypoints1_, indices1);
        normalization_matrix2_ = make_hartley_norm(keypoints2_, indices2);
    }

    std::vector<std::pair<float, float>>
    DescriptorMatcher::warp_corners(const math::Matrix3& H, int width,
                                    int height)
    {
        std::vector<std::pair<float, float>> corners = {
            { 0, 0 }, { width, 0 }, { width, height }, { 0, height }
        };

        std::vector<std::pair<float, float>> warped;
        for (auto [x, y] : corners)
        {
            math::Vector3 p = H * math::Vector3({ x, y, 1 });
            warped.push_back({ p[0] / p[2], p[1] / p[2] });
        }
        return warped;
    }

    std::vector<float>
    DescriptorMatcher::bilinear_sample(const image::ColorImage* image, float x,
                                       float y)
    {
        std::vector<float> result;

        int x0 = static_cast<int>(std::floor(x));
        int y0 = static_cast<int>(std::floor(y));
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        if (x0 < 0 || x1 >= image->get_width() || y0 < 0
            || y1 >= image->get_height())
        {
            return { 0, 0, 0 };
        }
        float dx = x - x0;
        float dy = y - y0;

        std::vector<float> color00 = (*image)(x0, y0);
        std::vector<float> color10 = (*image)(x1, y0);
        std::vector<float> color01 = (*image)(x0, y1);
        std::vector<float> color11 = (*image)(x1, y1);

        for (int channel_index = 0; channel_index < 3; channel_index++)
        {
            result.emplace_back((1 - dx) * (1 - dy) * color00[channel_index]
                                + dx * (1 - dy) * color10[channel_index]
                                + (1 - dx) * dy * color01[channel_index]
                                + dx * dy * color11[channel_index]);
        }
        return result;
    }

    image::ColorImage*
    DescriptorMatcher::stitch(const image::ColorImage* image1,
                              const image::ColorImage* image2)
    {
        math::Matrix3 H = compute_homography(matches_);
        std::cout << "H: " << H << "\n";
        int image2_start = -H(0, 2);

        // Fixed panorama dimensions (same as original image)
        int pano_width = image1->get_width() + image2_start;
        int pano_height = std::max(image1->get_height(), image2->get_height());

        image::ColorPPMImage* panorama =
            new image::ColorPPMImage(pano_width, pano_height);

        // Initialize panorama with zeros
        for (int y = 0; y < pano_height; y++)
        {
            for (int x = 0; x < pano_width; x++)
            {
                (*panorama)(x, y) = { 0.0f, 0.0f, 0.0f };
            }
        }

        // First, place image1 directly (it stays at origin)
        for (int y = 0; y < image1->get_height(); y++)
        {
            for (int x = 0; x < image1->get_width(); x++)
            {
                if (x < pano_width && y < pano_height)
                {
                    (*panorama)(x, y) = (*image1)(x, y);
                }
            }
        }

        // Get inverse homography to transform from panorama to image2
        math::Matrix3 H_inv = H.inverse();
        std::cout << "H_inv: " << H_inv << "\n";

        // Now iterate through panorama pixels and sample from image2 where
        // needed
        for (int y = 0; y < pano_height; y++)
        {
            for (int x = 0; x < pano_width; x++)
            {
                // Only process regions where image2 should contribute
                if (x >= image2_start)
                {
                    // Transform panorama coordinates to image2 coordinates
                    math::Vector3 p = H
                        * math::Vector3({ static_cast<float>(x),
                                          static_cast<float>(y), 1.0f });
                    float u = p[0] / p[2];
                    float v = p[1] / p[2];

                    // Check if the transformed point is within image2 bounds
                    if (u >= 0 && u < image2->get_width() && v >= 0
                        && v < image2->get_height())
                    {
                        std::vector<float> pixel2 =
                            bilinear_sample(image2, u, v);
                        std::vector<float> current = (*panorama)(x, y);

                        if (x >= image2_start && x < image1->get_width())
                        {
                            // Region 2: x = 300 to 700 - blend image1 and
                            // image2
                            float blending_factor = linear_blending(
                                x, image1->get_width(), image2_start);
                            (*panorama)(x, y) = {
                                current[0] * blending_factor
                                    + pixel2[0] * (1 - blending_factor),
                                current[1] * blending_factor
                                    + pixel2[1] * (1 - blending_factor),
                                current[2] * blending_factor
                                    + pixel2[2] * (1 - blending_factor)
                            };
                        }
                        else if (x >= image2->get_width()
                                 && x < image2_start + image1->get_width())
                        {
                            // Region 3: x = 700 to 1000 - only image2
                            (*panorama)(x, y) = pixel2;
                        }
                    }
                }
                // Region 1: x = 0 to 300 - only image1 (already placed, do
                // nothing)
            }
        }

        return panorama;
    }

    float DescriptorMatcher::linear_blending(int x, int image1_width,
                                             int image2_start)
    {
        if (x < image2_start)
        {
            return 1.;
        }
        if (x < image1_width)
        {
            return static_cast<float>(image1_width - x)
                / static_cast<float>(image1_width - image2_start);
        }
        return 0.;
    }

} // namespace tifo::panorama::sift