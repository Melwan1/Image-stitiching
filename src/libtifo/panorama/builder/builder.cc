#include <panorama/builder/builder.hh>

namespace tifo::panorama::builder
{

    void Builder::set_input_images(
        const std::vector<image::ColorImage*>& input_images)
    {
        input_images_ = input_images;
    }

    void Builder::free_inputs()
    {
        for (const auto input_image : input_images_)
        {
            delete input_image;
        }
    }

} // namespace tifo::panorama::builder