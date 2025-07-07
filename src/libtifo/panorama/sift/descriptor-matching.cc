#include <algorithm>
#include <iostream>
#include <random>
#include <unordered_set>

#include <math/vector.hh>
#include <panorama/sift/descriptor-matching.hh>
#include <images/color-ppm-image.hh>

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


    std::pair<int, int> DescriptorMatcher::find_two_nearest_neighbors(const std::vector<float>& query_desc, const std::vector<KeyPoint>& target_keypoints) {
        float min_dist1 = 1000.0f;
        float min_dist2 = 1000.0f;
        int best_idx = -1;
        int second_best_idx = -1;
        for (unsigned i = 0; i < target_keypoints.size(); i++) {
            float distance = descriptor_distance(query_desc, target_keypoints[i].descriptor);
            if (distance < min_dist1) {
                min_dist2 = min_dist1;
                second_best_idx = best_idx;
                min_dist1 = distance;
                best_idx = i;
            }
            else if (distance < min_dist2) {
                min_dist2 = distance;
                second_best_idx = i;
            }
        }

        return std::make_pair(best_idx, second_best_idx);
    }

    std::vector<Match>
    DescriptorMatcher::forward_matching(const std::vector<KeyPoint>& keypoints1, const std::vector<KeyPoint>& keypoints2) {
        std::vector<Match> matches;

        for (unsigned i = 0; i < keypoints1.size(); i++) {
            auto [best_idx, second_best_idx] = find_two_nearest_neighbors(keypoints1[i].descriptor, keypoints2);
            if (best_idx == -1 || second_best_idx == -1) {
                continue;
            }

            float best_dist = descriptor_distance(keypoints1[i].descriptor, keypoints2[best_idx].descriptor);
            float second_best_dist = descriptor_distance(keypoints1[i].descriptor, keypoints2[second_best_idx].descriptor);

            if (best_dist < ratio_threshold * second_best_dist && best_dist < max_descriptor_distance) {

                Match match;
                match.idx1 = i;
                match.idx2 = best_idx;
                match.distance = best_dist;
                match.pt1 = {keypoints1[i].x, keypoints1[i].y};
                match.pt2 = {keypoints2[best_idx].x, keypoints2[best_idx].y};
                matches.push_back(match);
            }
        }
        return matches;
    }
    std::vector<Match>
    DescriptorMatcher::backward_matching(const std::vector<KeyPoint>& keypoints1, const std::vector<KeyPoint>& keypoints2) {
        std::vector<Match> matches;

        for (unsigned i = 0; i < keypoints1.size(); i++) {
            auto [best_idx, second_best_idx] = find_two_nearest_neighbors(keypoints2[i].descriptor, keypoints1);
            if (best_idx == -1 || second_best_idx == -1) {
                continue;
            }

            float best_dist = descriptor_distance(keypoints2[i].descriptor, keypoints1[best_idx].descriptor);
            float second_best_dist = descriptor_distance(keypoints2[i].descriptor, keypoints1[second_best_idx].descriptor);

            if (best_dist < ratio_threshold * second_best_dist && best_dist < max_descriptor_distance) {

                Match match;
                match.idx1 = best_idx;
                match.idx2 = i;
                match.distance = best_dist;
                match.pt1 = {keypoints1[best_idx].x, keypoints1[best_idx].y};
                match.pt2 = {keypoints2[i].x, keypoints2[i].y};
                matches.push_back(match);
            }
        }
        return matches;
    }

    std::vector<Match> DescriptorMatcher::cross_check_matching(const std::vector<KeyPoint>& keypoints1, const std::vector<KeyPoint>& keypoints2) {
        std::vector<Match> forward_matches = forward_matching(keypoints1, keypoints2);
        std::vector<Match> backward_matches = backward_matching(keypoints1, keypoints2);
        std::vector<Match> cross_checked_matches;

        for (const auto& forward_match : forward_matches) {
            for (const auto& backward_match : backward_matches) {
                if (forward_match.idx1 == backward_match.idx1 && forward_match.idx2 == backward_match.idx2) {
                    cross_checked_matches.push_back(forward_match);
                    break;
                }
            }
        }

        std::cout << "Forward matches: " << forward_matches.size() << "\n";
        std::cout << "Backward matches: " << backward_matches.size() << "\n";
        std::cout << "Cross-checked matches: " << cross_checked_matches.size() << "\n";

        return cross_checked_matches;

    }

    std::vector<Match> DescriptorMatcher::geometric_verification(const std::vector<Match>& matches) {
        if (matches.size() < 4) {
            return matches;
        }

        std::vector<float> dx_values, dy_values;
        for (const auto& match : matches) {
            dx_values.push_back(match.pt2.first - match.pt1.first);
            dy_values.push_back(match.pt2.second - match.pt1.second);
        }

        std::sort(dx_values.begin(), dx_values.end());
        std::sort(dy_values.begin(), dy_values.end());

        float median_dx = dx_values[dx_values.size() / 2];
        float median_dy = dy_values[dy_values.size() / 2];

        std::cout << "Median displacement: " << "dx = " << median_dx << ", dy = " << median_dy << "\n";

        std::vector<Match> geometrically_consistent_matches;

        for (const auto& match : matches) {
            float dx = match.pt2.first - match.pt1.first;
            float dy = match.pt2.second - match.pt1.second;

            float dx_diff = std::abs(dx - median_dx);
            float dy_diff = std::abs(dy - median_dy);

            if (dx_diff < geometric_threshold && dy_diff < geometric_threshold) {
                geometrically_consistent_matches.push_back(match);
            }
            else {
                std::cout << "Rejecting match: (" << match.pt1.first << ", " << match.pt1.second << ") <--> (" << match.pt2.first << ", " << match.pt2.second << "), dx = " << dx << ", dy = " << dy << "\n";
            }
        }

        std::cout << "Geometrically consistent matches: " << geometrically_consistent_matches.size() << " out of " << matches.size() << "\n";

        return geometrically_consistent_matches;
    }


    std::vector<Match> DescriptorMatcher::robust_matching(const std::vector<KeyPoint>& keypoints1, const std::vector<KeyPoint>& keypoints2) {

        std::cout << "=== ROBUST SIFT MATCHING PIPELINE ===\n";
        std::cout << "Keypoints1: " << keypoints1.size() << ", Keypoints2: " << keypoints2.size() << "\n";  
        std::vector<Match> cross_checked = cross_check_matching(keypoints1, keypoints2);
        std::vector<Match> final_matches = geometric_verification(cross_checked);
        std::cout << "Final matches: " << final_matches.size() << "\n";

        keypoints1_ = keypoints1;
        keypoints2_ = keypoints2;

        matches_ = final_matches;

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
                    Match(static_cast<int>(i), best_idx, best_distance, {keypoints1[i].x, keypoints1[i].y}, {keypoints2[best_idx].x, keypoints2[best_idx].y}));
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


    std::vector<std::pair<float, float>> DescriptorMatcher::warp_corners(const math::Matrix3& H, int width, int height) {
        std::vector<std::pair<float, float>> corners = {
            {0, 0},
            {width, 0},
            {width, height},
            {0, height}
        };

        std::vector<std::pair<float, float>> warped;
        for (auto [x, y] : corners) {
            math::Vector3 p = H * math::Vector3({x, y, 1});
            warped.push_back({p[0] / p[2], p[1] / p[2]});
        }
        return warped;
    }


    std::vector<float> DescriptorMatcher::bilinear_sample(const image::ColorImage* image, float x, float y) {

        std::vector<float> result;

        int x0 = static_cast<int>(std::floor(x));
        int y0 = static_cast<int>(std::floor(y));
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        if (x0 < 0 || x1 >= image->get_width() || y0 < 0 || y1 >= image->get_height()) {
            return {0, 0, 0};
        }
        float dx = x - x0;
        float dy = y - y0;

        std::vector<float> color00 = (*image)(x0, y0);
        std::vector<float> color10 = (*image)(x1, y0);
        std::vector<float> color01 = (*image)(x0, y1);
        std::vector<float> color11 = (*image)(x1, y1);

        for (int channel_index = 0; channel_index < 3; channel_index++) {
            result.emplace_back((1 - dx) * (1 - dy) * color00[channel_index] + dx * (1 - dy) * color10[channel_index] + (1 - dx) * dy * color01[channel_index] + dx * dy * color11[channel_index]);
        }
        return result;
    }

    image::ColorImage* DescriptorMatcher::stitch(const image::ColorImage* image1, const image::ColorImage* image2) {

        math::Matrix3 H = compute_homography(matches_);

        std::cout << "H: " << H << "\n";
        float det = H(0, 0) * (H(1, 1) * H(2, 2) - H(1, 2) * H(2, 1)) - 
                H(0, 1) * (H(1, 0) * H(2, 2) - H(1, 2) * H(2, 0)) + 
                H(0, 2) * (H(1, 0) * H(2, 1) - H(1, 1) * H(2, 0));
        
        std::cout << "Homography determinant: " << det << std::endl;

        math::Matrix3 translation = {
            {1, 0, static_cast<float>(image1->get_width())},
            {0, 1, 0},
            {0, 0, 1}
        };
        math::Matrix3 H_translated = translation * H;

        std::vector<math::Vector3> corners = {
            {0, 0, 1},
            {static_cast<float>(image2->get_width()), 0, 1},
            {0, static_cast<float>(image2->get_height()), 1},
            {static_cast<float>(image2->get_width()), static_cast<float>(image2->get_height()), 1}
        };

        float min_x = 0;
        float max_x = image1->get_width();
        float min_y = 0;
        float max_y = image1->get_height();

        for (const auto& corner : corners) {
            math::Vector3 warped = H_translated * corner;
            float x = warped[0] / warped[2];
            float y = warped[1] / warped[2];
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }

        int offset_x = static_cast<int>(std::min(0.0f, min_x));
        int offset_y = static_cast<int>(std::min(0.0f, min_y));

        int pano_width = (max_x - min_x);
        int pano_height = (max_y - min_y);

        image::ColorPPMImage* panorama = new image::ColorPPMImage(pano_width, pano_height);

        for (int y = 0; y < image1->get_height(); y++) {
            for (int x = 0; x < image1->get_width(); x++) {
                (*panorama)(x, y) = {0, 0, 0};
            }
        }

        for (int y = 0; y < image1->get_height(); y++) {
            for (int x = 0; x < image1->get_width(); x++) {
                int pano_x = x - offset_x;
                int pano_y = y - offset_y;
                if (pano_x >= 0 && pano_x < pano_width && pano_y >= 0 && pano_y < pano_height) {
                    (*panorama)(pano_x, pano_y) = (*image1)(x, y);
                }
            }
        }

        math::Matrix3 offset_correction = {
            {1, 0, static_cast<float>(-offset_x)},
            {0, 1, static_cast<float>(-offset_y)},
            {0, 0, 1}
        };

        math::Matrix3 final_H = offset_correction * H_translated;

        math::Matrix3 H_inv = final_H.inverse();

        for (int y = 0; y < pano_height; y++) {
            for (int x = 0; x < pano_width; x++) {
                math::Vector3 p = H_inv * math::Vector3({static_cast<float>(x), static_cast<float>(y), 1.0});
                float u = p[0] / p[2];
                float v = p[1] / p[2];

                if (u >= 0 && u < image2->get_width() && v >= 0 && v < image2->get_height()) {
                    std::vector<float> pixel2 = bilinear_sample(image2, u, v);
                    std::vector<float> current = (*panorama)(x, y);
                    if (current[0] > 0 || current[1] > 0 || current[2] > 0) {
                        (*panorama)(x, y) = {(current[0] + pixel2[0]) / 2, (current[1] + pixel2[1]) / 2, (current[2] + pixel2[2]) / 2};
                    }
                    else {
                        (*panorama)(x, y) = pixel2;
                    }
                }
            }
        }

        return panorama;

    }

} // namespace tifo::panorama::sift