#pragma once

#include <images/image.hh>
#include <vector>

namespace tifo::image
{

    class GrayscaleImage : public Image
    {
    public:
        using container_type = std::vector<float>;

        GrayscaleImage();
        GrayscaleImage(int width, int height);

        const container_type& get_pixels() const;
        container_type& get_pixels();

    protected:
        container_type pixels_;
    };

} // namespace tifo::image