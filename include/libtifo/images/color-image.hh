#pragma once

#include <images/grayscale-image.hh>
#include <images/image.hh>
#include <vector>

namespace tifo::image
{

    class ColorImage : public Image
    {
    public:
        using container_type = std::vector<std::vector<float>>;

        ColorImage();
        ColorImage(int width, int height);

        const container_type& get_pixels() const;
        container_type& get_pixels();

        GrayscaleImage* to_grayscale() const;

    protected:
        container_type pixels_;
    };

} // namespace tifo::image