#pragma once

#include <vector>

#include <images/image.hh>

namespace tifo::image {

    class GrayscaleImage : public Image {

        public:

            using container_type = std::vector<float>;

            GrayscaleImage();
            GrayscaleImage(int width, int height);

            const container_type& get_pixels() const;
            container_type& get_pixels();

        protected:

            container_type pixels_;

    };

}