#pragma once

#include <images/image.hh>
#include <images/color-image.hh>

namespace tifo::image
{

    class ColorPPMImage : public ColorImage
    {
    public:
        ColorPPMImage() = default;
        ColorPPMImage(int width, int height);

        virtual void read(const fs::path& src_path) override;
        virtual void write(const fs::path& dst_path) const override;
    };

} // namespace tifo::image