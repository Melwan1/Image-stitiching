#include <images/image.hh>

#include <sstream>
#include <iostream>

#include <images/ppm-image.hh>

namespace tifo::image
{

    Image::Image()
        : pixels_()
        , height_(0)
        , width_(0)
    {}

    const Image::container_type& Image::get_pixels() const
    {
        return pixels_;
    }

    Image::container_type& Image::get_pixels()
    {
        return pixels_;
    }

    int Image::get_height() const
    {
        return height_;
    }

    int Image::get_width() const
    {
        return width_;
    }

    std::vector<Image*> Image::rectangular_cut(int horizontal_slices, int vertical_slices) const {
        if (horizontal_slices <= 0 || horizontal_slices > width_ || vertical_slices <= 0 || vertical_slices > height_) {
            std::ostringstream oss;
            oss << "tifo::image::Image - rectangular_cut - cannot perform cuts with slices count (" << horizontal_slices << ", " << vertical_slices << ")";
            throw std::runtime_error(oss.str());
        }

        std::vector<Image*> cut_images;
        cut_images.resize(horizontal_slices * vertical_slices);
        for (int horizontal_index = 0; horizontal_index < horizontal_slices; horizontal_index++) {
            for (int vertical_index = 0; vertical_index < vertical_slices; vertical_index++) {
                cut_images[vertical_index * horizontal_slices + horizontal_index] = new PPMImage();
                int x_min = width_ * horizontal_index / horizontal_slices;
                int x_max = width_ * (horizontal_index + 1) / horizontal_slices;
                int y_min = height_ * vertical_index / vertical_slices;
                int y_max = height_ * (vertical_index + 1) / vertical_slices;
                std::cout << "image from (" << x_min << ", " << y_min << ") to (" << x_max << ", " << y_max << ")\n";
                
                Image* cropped_image = cut_images.at(vertical_index * horizontal_slices + horizontal_index);
                cropped_image->width_ = x_max - x_min;
                cropped_image->height_ = y_max - y_min;
                cropped_image->get_pixels().resize(cropped_image->width_ * cropped_image->height_);
                for (int x = x_min; x < x_max; x++) {
                    for (int y = y_min; y < y_max; y++) {
                        const std::vector<float>& input_pixel = get_pixels().at(y * width_ + x);
                        std::vector<float>& output_pixel = cropped_image->get_pixels().at((y - y_min) * cropped_image->width_ + x - x_min);
                        output_pixel.resize(3);
                        std::cout << "setting pixel at (" << x - x_min << ", " << y - y_min << ")\n";
                        for (int color_index = 0; color_index < 3; color_index++) {
                            output_pixel[color_index] = input_pixel[color_index];
                        }
                    }
                }
            }
        }
        return cut_images;
    }


    Image* Image::rebuild_from_cuts(const std::vector<Image*>& cuts, int horizontal_slices, int vertical_slices) {
        if (cuts.size() != static_cast<unsigned>(horizontal_slices * vertical_slices) || horizontal_slices <= 0 || vertical_slices <= 0) {
            throw std::runtime_error("tifo::image::Image - rebuild_from_cuts - cuts size is invalid.");
        }
        PPMImage* image = new PPMImage();
        int cut_width = cuts[0]->get_width();
        int cut_height = cuts[0]->get_height(); // suppose every cut has the same size (could be dangerous for weird base image size!!)
        image->width_ = cut_width * horizontal_slices;
        image->height_ = cut_height * vertical_slices;
        image->get_pixels().resize(image->get_width() * image->get_height());
        for (int horizontal_index = 0; horizontal_index < horizontal_slices; horizontal_index++) {
            for (int vertical_index = 0; vertical_index < vertical_slices; vertical_index++) {
                int x_min = cut_width * horizontal_index;
                int y_min = cut_height * vertical_index;
                for (int dx = 0; dx < cut_width; dx++) {
                    for (int dy = 0; dy < cut_height; dy++) {
                        const std::vector<float>& input_pixel = cuts[vertical_index * horizontal_slices + horizontal_index]->get_pixels()[dy * cut_width + dx];
                        std::vector<float>& output_pixel = image->get_pixels()[(y_min + dy) * image->get_width() + (x_min + dx)];
                        output_pixel.resize(3);
                        for (int color_index = 0; color_index < 3; color_index++) {
                            output_pixel[color_index] = input_pixel[color_index];
                        }
                    }
                }
            }
        }
        return image;
    }

} // namespace tifo::image