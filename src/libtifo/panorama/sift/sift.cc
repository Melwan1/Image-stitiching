#include <filters/gaussian.hh>
#include <fstream>
#include <panorama/sift/sift.hh>
#include <sstream>

namespace tifo::panorama::sift
{

    std::vector<float> SIFT::generate_gaussian_kernel(float sigma)
    {
        int size = static_cast<int>(2 * std::ceil(3 * sigma) + 1);
        if (size % 2 == 0)
        {
            size++;
        }
        std::vector<float> kernel(size);
        int center = size / 2;
        float sum = 0.0f;

        for (int i = 0; i < size; i++)
        {
            float x = i - center;
            kernel[i] = std::exp(-(x * x) / (2 * sigma * sigma));
            sum += kernel[i];
        }

        for (float& k : kernel)
        {
            k /= sum;
        }
        return kernel;
    }

    image::GrayscaleImage*
    SIFT::gaussian_blur(const image::GrayscaleImage* image, float sigma)
    {
        std::vector<float> kernel = generate_gaussian_kernel(sigma);
        int radius = kernel.size() / 2;

        // horizontal blur
        image::GrayscaleImage* temp = new image::GrayscalePPMImage(
            image->get_width(), image->get_height());
        for (int y = 0; y < image->get_height(); y++)
        {
            for (int x = 0; x < image->get_width(); x++)
            {
                float sum = 0.0f;
                for (int k = 0; k < static_cast<int>(kernel.size()); k++)
                {
                    int px = x + k - radius;
                    px = std::clamp(px, 0, image->get_width() - 1);
                    sum += (*image)(px, y) * kernel[k];
                }
                (*temp)(x, y) = sum;
            }
        }

        // vertical blur
        image::GrayscaleImage* result = new image::GrayscalePPMImage(
            image->get_width(), image->get_height());
        for (int y = 0; y < image->get_height(); y++)
        {
            for (int x = 0; x < image->get_width(); x++)
            {
                float sum = 0.0f;
                for (int k = 0; k < static_cast<int>(kernel.size()); k++)
                {
                    int py = y + k - radius;
                    py = std::clamp(py, 0, image->get_height() - 1);
                    sum += (*temp)(x, py) * kernel[k];
                }
                (*result)(x, y) = sum;
            }
        }
        delete temp;
        return result;
    }

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
                pyramid[octave].emplace_back(gaussian_blur(current, SIGMA));
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
                    gaussian_blur(pyramid[octave][scale - 1], sigma);
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
        float center = (*dog[octave][scale])(x, y);

        bool is_max = true;
        bool is_min = true;

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int ds = -1; ds <= 1; ds++)
                {
                    if (!dog[octave][scale]->is_valid_access(x + dx, y + dy))
                    {
                        continue;
                    }
                    if (dy == 0 && dx == 0 && ds == 0)
                    {
                        continue;
                    }
                    float neighbor = (*dog[octave][scale + ds])(x + dx, y + dy);
                    if (neighbor >= center)
                    {
                        is_max = false;
                    }
                    if (neighbor <= center)
                    {
                        is_min = false;
                    }
                }
            }
        }

        return is_max || is_min;
    }

    std::pair<float, float>
    SIFT::compute_gradient(const image::GrayscaleImage* image, int x, int y)
    {
        int x_left = std::max(0, x - 1);
        int x_right = std::min(image->get_width() - 1, x + 1);
        int y_up = std::max(0, y - 1);
        int y_down = std::min(image->get_height() - 1, y + 1);
        float dx = (*image)(x_right, y) - (*image)(x_left, y);
        float dy = (*image)(x, y_down) - (*image)(x, y_up);

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
                int min_x = (scale > 0) ? 0 : 1;
                int max_x = (scale < scales_ + 1) ? current->get_width() : current->get_width() - 1;
                int min_y = (scale > 0) ? 0 : 1;
                int max_y = (scale < scales_ + 1) ? current->get_height() : current->get_height() - 1;
                for (int y = min_y; y < max_y; y++)
                {
                    for (int x = min_x; x < max_x; x++)
                    {
                        if (std::abs((*current)(x, y)) > contrast_threshold_
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
                            float max = 0;
                            for (int i = 0; i < 128; i++)
                            {
                                if (keypoint.descriptor[i] > max)
                                {
                                    max = keypoint.descriptor[i];
                                }
                            }
                            if (max > std::numeric_limits<float>::epsilon())
                            {
                                keypoints.emplace_back(keypoint);
                            }
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