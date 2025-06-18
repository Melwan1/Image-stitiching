#pragma once

#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace tifo::image
{

    class Image
    {
        using container_type = std::vector<std::vector<float>>;

    public:
        Image();
        virtual ~Image() = default;
        virtual void read(const fs::path& src_path) = 0;

        virtual void write(const fs::path& dst_path) const = 0;

        const container_type& get_pixels() const;
        container_type& get_pixels();

        int get_height() const;
        int get_width() const;

        std::vector<Image*> rectangular_cut(int horizontal_slices, int vertical_slices) const;
        static Image* rebuild_from_cuts(const std::vector<Image*>& cuts, int horizontal_slices, int vertical_slices);

    protected:
        container_type pixels_;
        int height_;
        int width_;
    };

} // namespace tifo::image