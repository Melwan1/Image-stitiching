#pragma once

#include <images/image.hh>

namespace tifo::image
{

    class PPMImage : public Image
    {
    public:
        virtual void read(const fs::path& src_path) override;
        virtual void write(const fs::path& dst_path) const override;
    };

} // namespace tifo::image