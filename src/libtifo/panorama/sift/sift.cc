#include <filters/gaussian.hh>
#include <fstream>
#include <panorama/sift/sift.hh>

namespace tifo::panorama::sift
{

    std::vector<std::vector<image::GrayscaleImage*>>
    SIFT::build_gaussian_pyramid(image::GrayscaleImage* image)
    {
        std::vector<std::vector<image::GrayscaleImage*>> pyramid(octaves_);

        image::GrayscaleImage* current = image;

        for (int octave = 0; octave < octaves_; octave++)
        {
            pyramid[octave].reserve(scales_ + 3);

            if (octave == 0)
            {
                filter::GaussianFilter<GAUSSIAN_FILTER_SIZE> gaussian_filter(
                    SIGMA);
                pyramid[octave].emplace_back(
                    gaussian_filter.apply_on_image(current));
            }
            else
            {
                pyramid[octave].emplace_back(current);
            }

            // generate other scales
            for (int scale = 1; scale < scales_ + 3; scale++)
            {
                float sigma = SIGMA * std::pow(k_, scale);
                image::GrayscaleImage* blurred =
                    gaussian_filter.apply_on_image(pyramid[octave][scale - 1]);
                pyramid[octave].emplace_back(blurred);
            }

            // downsample for next octave
            if (octave < octaves_ - 1)
            {
                current = pyramid[octave][scales_]->downsample(2);
            }
        }

        return pyramid;
    }

    std::vector<std::vector<image::GrayscaleImage*>> SIFT::build_dog_pyramid(
        const std::vector<std::vector<image::GrayscaleImage*>>& gaussian)
    {
        std::vector<std::vector<image::GrayscaleImage*>> dog(octaves_);

        for (int octave = 0; octave < octaves_; octave++)
        {
            dog[octave].reserve(scales_ + 2);

            for (int scale = 0; scale < scales_ + 2; scale++)
            {
                const image::GrayscaleImage* image1 = gaussian[octave][scale];
                const image::GrayscaleImage* image2 =
                    gaussian[octave][scale + 1];

                image::GrayscaleImage* difference =
                    new image::GrayscalePPMImage(image1->get_width(),
                                                 image1->get_height());
                for (int index = 0;
                     index < image1->get_width() * image1->get_height();
                     index++)
                {
                    difference->get_pixels()[index] =
                        image2->get_pixels()[index]
                        - image1->get_pixels()[index];
                }
                dog[octave].emplace_back(difference);
            }
        }

        return dog;
    }

    bool SIFT::is_extremum(
        const std::vector<std::vector<image::GrayscaleImage*>>& dog, int octave,
        int scale, int x, int y)
    {
        const image::GrayscaleImage* current = dog[octave][scale];
        const image::GrayscaleImage* above = dog[octave][scale + 1];
        const image::GrayscaleImage* below = dog[octave][scale - 1];

        float center = (*current)(x, y);

        bool is_max = true;
        bool is_min = true;

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (!current->is_valid_access(x + dx, y + dy))
                {
                    continue;
                }

                float val1 = (*below)(x + dx, y + dy);
                float val2 = (*current)(x + dx, y + dy);
                float val3 = (*above)(x + dx, y + dx);

                if (val1 >= center || val2 >= center || val3 >= center)
                {
                    is_max = false;
                }
                if (val1 <= center || val2 <= center || val3 <= center)
                {
                    is_min = false;
                }
            }
        }

        return is_max || is_min;
    }

    std::pair<float, float>
    SIFT::compute_gradient(const image::GrayscaleImage* image, int x, int y)
    {
        if (!image->is_valid_access(x - 1, y)
            || !image->is_valid_access(x + 1, y)
            || !image->is_valid_access(x, y - 1)
            || !image->is_valid_access(x, y + 1))
        {
            return { 0.0f, 0.0f };
        }
        float dx = (*image)(x + 1, y) - (*image)(x - 1, y);
        float dy = (*image)(x, y + 1) - (*image)(x, y - 1);

        float magnitude = std::sqrt(dx * dx + dy * dy);
        float orientation = std::atan2(dy, dx);

        return { magnitude, orientation };
    }

    float SIFT::compute_dominant_orientation(const image::GrayscaleImage* image,
                                             int x, int y, float scale)
    {
        const int radius = static_cast<int>(3 * scale);
        const int bins = 36;
        std::vector<float> histogram(bins, 0.0f);

        for (int dx = -radius; dx <= radius; dx++)
        {
            for (int dy = -radius; dy <= radius; dy++)
            {
                int px = x + dx;
                int py = y + dy;

                if (!image->is_valid_access(px, py))
                {
                    continue;
                }

                float squared_distance = dx * dx + dy * dy;
                if (squared_distance > radius * radius)
                {
                    continue;
                }

                auto [magnitude, orientation] = compute_gradient(image, px, py);
                float weight =
                    std::exp(-squared_distance / (2 * scale * scale));

                int bin = static_cast<int>((orientation + std::numbers::pi)
                                           * bins / (2 * std::numbers::pi))
                    % bins;
                histogram[bin] += magnitude * weight;
            }
        }

        // find peak
        int maxBin = 0;
        for (int i = 1; i < bins; i++)
        {
            if (histogram[i] > histogram[maxBin])
            {
                maxBin = i;
            }
        }

        return (maxBin * 2 * std::numbers::pi / bins) - std::numbers::pi;
    }

    std::vector<float>
    SIFT::compute_descriptor(const image::GrayscaleImage* image,
                             const KeyPoint& keypoint)
    {
        const int desc_width = 4;
        const int desc_bins = 8;
        const int desc_size = desc_width * desc_width
            * desc_bins; // each descriptor has 128 floats

        std::vector<float> descriptor(desc_size, 0.0f);

        float cos_angle = std::cos(-keypoint.orientation);
        float sin_angle = std::sin(-keypoint.orientation);

        int radius = static_cast<int>(3 * keypoint.scale * desc_width);

        for (int dx = -radius; dx <= radius; dx++)
        {
            for (int dy = -radius; dy <= radius; dy++)
            {
                int px = static_cast<int>(keypoint.x) + dx;
                int py = static_cast<int>(keypoint.y) + dy;

                if (!image->is_valid_access(px, py))
                {
                    continue;
                }

                // rotate coordinates
                float rot_x = cos_angle * dx - sin_angle * dy;
                float rot_y = sin_angle * dx + cos_angle * dy;

                // which descriptor bin does this fall into
                float desc_x = rot_x / keypoint.scale + desc_width / 2.0f;
                float desc_y = rot_y / keypoint.scale + desc_width / 2.0f;
                if (desc_x < 0 || desc_x >= desc_width || desc_y < 0
                    || desc_y >= desc_width)
                {
                    continue;
                }

                auto [magnitude, orientation] = compute_gradient(image, px, py);
                float relative_angle = orientation - keypoint.orientation;

                // normalize angle to [0, 2 * pi]
                while (relative_angle < 0)
                {
                    relative_angle += 2 * std::numbers::pi;
                }
                while (relative_angle >= 2 * std::numbers::pi)
                {
                    relative_angle -= 2 * std::numbers::pi;
                }

                float angle_bin =
                    relative_angle * desc_bins / (2 * std::numbers::pi);

                int x0 = static_cast<int>(desc_x);
                int y0 = static_cast<int>(desc_y);
                int a0 = static_cast<int>(angle_bin) % desc_bins;
                if (x0 < 0 || x0 >= desc_width - 1 || y0 < 0
                    || y0 >= desc_width - 1)
                {
                    continue;
                }

                float dx_weight = desc_x - x0;
                float dy_weight = desc_y - y0;
                float da_weight = angle_bin - a0;

                for (int dx_i = 0; dx_i <= 1; dx_i++)
                {
                    for (int dy_i = 0; dy_i <= 1; dy_i++)
                    {
                        for (int da_i = 0; da_i <= 1; da_i++)
                        {
                            int x_idx = x0 + dx_i;
                            int y_idx = y0 + dy_i;
                            int a_idx = (a0 + da_i) % desc_bins;

                            float weight = magnitude;
                            weight *= (dx_i == 0) ? (1 - dx_weight) : dx_weight;
                            weight *= (dy_i == 0) ? (1 - dy_weight) : dy_weight;
                            weight *= (da_i == 0) ? (1 - da_weight) : da_weight;

                            int desc_idx =
                                (y_idx * desc_width + x_idx) * desc_bins
                                + a_idx;
                            if (desc_idx < desc_size)
                            {
                                descriptor[desc_idx] += weight;
                            }
                        }
                    }
                }
            }
        }

        // normalize descriptor
        float norm = 0.0f;
        for (float val : descriptor)
        {
            norm += val * val;
        }
        norm = std::sqrt(norm);

        if (norm > 0)
        {
            for (float& val : descriptor)
            {
                val /= norm;
                val = std::min(val, 0.2f); // reduce illumination effects
            }
            // normalize again
            norm = 0.0f;
            for (float val : descriptor)
            {
                norm += val * val;
            }
            norm = std::sqrt(norm);
            if (norm > 0)
            {
                for (float& val : descriptor)
                {
                    val /= norm;
                }
            }
        }

        return descriptor;
    }

    std::vector<KeyPoint> SIFT::detect_and_compute(image::GrayscaleImage* image)
    {
        auto gaussian_pyramid = build_gaussian_pyramid(image);
        auto dog_pyramid = build_dog_pyramid(gaussian_pyramid);

        std::vector<KeyPoint> keypoints;

        for (int octave = 0; octave < octaves_; octave++)
        {
            for (int scale = 1; scale < scales_ + 1; scale++)
            {
                const image::GrayscaleImage* current =
                    dog_pyramid[octave][scale];

                for (int y = 1; y < current->get_height() - 1; y++)
                {
                    for (int x = 1; x < current->get_width() - 1; x++)
                    {
                        if (std::abs((*current)(x, y)) > contrast_threshold
                            && is_extremum(dog_pyramid, octave, scale, x, y))
                        {
                            float actual_scale = SIGMA * std::pow(k_, scale)
                                * std::pow(2, octave);
                            float orientation = compute_dominant_orientation(
                                gaussian_pyramid[octave][scale], x, y,
                                actual_scale);

                            // create keypoint
                            KeyPoint keypoint(x * std::pow(2, octave),
                                              y * std::pow(2, octave),
                                              actual_scale, orientation, octave,
                                              scale);

                            keypoint.descriptor = compute_descriptor(
                                gaussian_pyramid[octave][scale], keypoint);
                            keypoints.emplace_back(keypoint);
                        }
                    }
                }
            }
        }

        return keypoints;
    }

    void SIFT::save_keypoints(const std::vector<KeyPoint>& keypoints,
                              const fs::path& save_path)
    {
        std::ofstream file(save_path);

        file << keypoints.size() << "\n";

        for (const auto& keypoint : keypoints)
        {
            file << keypoint.x << " " << keypoint.y << " " << keypoint.scale
                 << " " << keypoint.orientation << " ";
            file << keypoint.octave << " " << keypoint.layer;

            for (unsigned i = 0; i < keypoint.descriptor.size(); i++)
            {
                file << keypoint.descriptor[i];
                if (i < keypoint.descriptor.size() - 1)
                {
                    file << " ";
                }
            }
            file << "\n";
        }
    }

} // namespace tifo::panorama::sift