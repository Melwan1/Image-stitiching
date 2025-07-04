#pragma once

#include <filesystem>
#include <ostream>
#include <vector>

namespace fs = std::filesystem;

namespace tifo::image
{

    class Image
    {
        using container_type = std::vector<std::vector<float>>;

    public:
        Image();
        Image(int width, int height);
        virtual ~Image() = default;
        virtual void read(const fs::path& src_path) = 0;

        virtual void write(const fs::path& dst_path) const = 0;

        const container_type& get_pixels() const;
        container_type& get_pixels();

        int get_height() const;
        int get_width() const;

        void set_height(int height);
        void set_width(int width);

    protected:
        container_type pixels_;
        int height_;
        int width_;
    };

} // namespace tifo::image

std::ostream& operator<<(std::ostream& ostr, const tifo::image::Image* image);