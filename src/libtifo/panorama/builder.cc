#include <panorama/builder.hh>

namespace tifo::panorama
{

    void
    Builder::set_input_images(const std::vector<image::Image*>& input_images)
    {
        input_images_ = input_images;
    }

    void Builder::free_inputs(const std::vector<image::Image*>& input_images)
    {
        for (const auto input_image : input_images)
        {
            delete input_image;
        }
    }

} // namespace tifo::panorama