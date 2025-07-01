#include <fstream>
#include <images/ppm-image.hh>
#include <iostream>
#include <set>
#include <stdexcept>

namespace tifo::image
{

    void PPMImage::read(const fs::path& src_path)
    {
        if (src_path.extension() != ".ppm")
        {
            throw std::runtime_error("tifo::image::PPMImage - the source file: "
                                     + src_path.string()
                                     + " does not have a .ppm extension.");
        }
        std::ifstream ifs(src_path, std::ios::binary);
        if (!ifs.is_open())
        {
            throw std::runtime_error("tifo::image::PPMImage - the source file: "
                                     + src_path.string()
                                     + " could not be opened.");
        }

        std::string file_format;
        ifs >> file_format;
        std::set<std::string> supported_file_formats = { "P6" };
        if (!supported_file_formats.contains(file_format))
        {
            throw std::runtime_error("tifo::image::PPMImage - the file format "
                                     + file_format + " of the source file: "
                                     + src_path.string()
                                     + " is not supported.");
        }
        ifs >> width_;
        ifs >> height_;
        int number_pixels = width_ * height_;
        float max_color = 0;
        ifs >> max_color;
        ifs.ignore();
        pixels_.resize(number_pixels);
        for (int pixel_index = 0; pixel_index < number_pixels; pixel_index++)
        {
            pixels_[pixel_index].resize(3);
            for (int color_index = 0; color_index < 3; color_index++)
            {
                char element;
                ifs.read(&element, 1);
                pixels_[pixel_index][color_index] = static_cast<float>(element)
                    / max_color; // rescale inside [0, 1]
            }
        }
    }

    void PPMImage::write(const fs::path& dst_path) const
    {
        if (dst_path.extension() != ".ppm")
        {
            throw std::runtime_error(
                "tifo::image::PPMImage - the destination file: "
                + dst_path.string() + " does not have a .ppm extension.");
        }
        std::ofstream ofs(dst_path);
        if (!ofs.is_open())
        {
            throw std::runtime_error(
                "tifo::image::PPMImage - the destination file: "
                + dst_path.string() + " could not be opened.");
        }
        ofs << "P6\n" << width_ << " " << height_ << "\n" << 255 << "\n";
        float number_pixels = width_ * height_;
        for (int pixel_index = 0; pixel_index < number_pixels; pixel_index++)
        {
            for (int color_index = 0; color_index < 3; color_index++)
            {
                char value = static_cast<char>(pixels_[pixel_index][color_index]
                                               * 255.f);
                ofs.write(&value, 1);
            }
        }
        ofs.close();
    }

} // namespace tifo::image