#include <panorama/cutter/cutter.hh>

namespace tifo::panorama::cutter
{

    void Cutter::set_input_image(image::Image* input_image)
    {
        if (!input_image)
        {
            throw std::runtime_error(
                "tifo::panorama::cutter::Cutter - set_input_image - the image "
                "cannot be null.");
        }
        input_image_ = input_image;
    }

    void Cutter::free_input()
    {
        delete input_image_;
    }

} // namespace tifo::panorama::cutter