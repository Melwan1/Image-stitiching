#pragma once

#include <vector>

#include <images/image.hh>

namespace tifo::image {

    class ColorImage : public Image {

        public:

            using container_type = std::vector<std::vector<float>>;

            ColorImage();
            ColorImage(int width, int height);

            const container_type& get_pixels() const;
            container_type& get_pixels();

        protected:

            container_type pixels_;

    };

}