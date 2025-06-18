#include <images/ppm-image.hh>
#include <panorama/clean_cut_rectangular_builder.hh>
#include <sstream>

int main()
{
    tifo::image::PPMImage image;
    image.read("tests/julie.ppm");
    std::vector<tifo::image::Image*> cut_images = image.rectangular_cut(2, 4);
    tifo::panorama::CleanCutRectangularBuilder builder;
    builder.set_input_images(cut_images);
    builder.set_horizontal_slices(2).set_vertical_slices(4);
    tifo::image::Image* built_image = builder.build();
    builder.free_inputs(cut_images);
    built_image->write("tests/new_julie.ppm");
    return 0;
}