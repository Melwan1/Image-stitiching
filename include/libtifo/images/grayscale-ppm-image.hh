#pragma once

#include <images/grayscale-image.hh>

namespace tifo::image {

    class GrayscalePPMImage : public GrayscaleImage {

        public:

            GrayscalePPMImage() = default;
            GrayscalePPMImage(int width, int height);

            virtual void read(const fs::path& src_path) override;
            virtual void write(const fs::path& dst_path) const override;

    };

}