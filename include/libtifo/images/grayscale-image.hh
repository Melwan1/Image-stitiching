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

        // access operator
        const float& operator()(int x, int y) const;
        float& operator()(int x, int y);

        // check validity of pixel access
        bool is_valid_access(int x, int y) const;

        GrayscaleImage* downsample(int factor) const;

    protected:
        container_type pixels_;
    };

} // namespace tifo::image