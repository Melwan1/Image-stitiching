#pragma once

#include <images/image.hh>

namespace tifo::image
{

    class PPMImage : public Image
    {
    public:
        PPMImage() = default;
        PPMImage(int width, int height);

        virtual void read(const fs::path& src_path) override;
        virtual void write(const fs::path& dst_path) const override;
    };

} // namespace tifo::image