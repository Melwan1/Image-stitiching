#pragma once

#include <array>
#include <filesystem>
#include <vector>
#include <array>

namespace fs = std::filesystem;

namespace tifo::image
{

    class Image
    {

        using container_type = std::vector<std::array<float, 3>>;

        public:
            
            Image();
            virtual void read(const fs::path& src_path) = 0;

            virtual void write(const fs::path& dst_path) const = 0;

            const container_type& get_pixels() const;
            container_type& get_pixels();

            int get_height() const;
            int get_width() const;

        protected:
            container_type pixels_;
            int height_;
            int width_;

    };

} // namespace tifo::image