#pragma once

#include <images/image.hh>

namespace tifo::image
{

    class PPMImage : public Image
    {
    public:
        void read(const fs::path& src_path) override;
        void write(const fs::path& dst_path) const override;
    };

} // namespace tifo::image