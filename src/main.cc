#include <images/ppm-image.hh>
#include <panorama/builder/overlap_rectangular_builder.hh>
#include <panorama/cutter/overlap_rectangular_cutter.hh>
#include <sstream>

int main()
{
    tifo::image::PPMImage* image = new tifo::image::PPMImage();
    image->read("tests/julie2.ppm");
    tifo::panorama::cutter::OverlapRectangularCutter cutter;
    cutter.set_input_image(image);
    cutter.set_horizontal_slices(2)
        .set_vertical_slices(2)
        .set_horizontal_overlap_size(10)
        .set_vertical_overlap_size(20);
    std::vector<tifo::image::Image*> cut_images = cutter.cut();
    cutter.free_input();
    int index = 1;
    for (const auto cut_image : cut_images)
    {
        std::ostringstream oss;
        oss << "tests/cut_julie_" << index++ << ".ppm";
        cut_image->write(oss.str());
    }

    tifo::panorama::builder::OverlapRectangularBuilder builder;
    builder.set_input_images(cut_images);
    builder.set_horizontal_slices(2).set_vertical_slices(3);
    tifo::image::Image* built_image = builder.build();
    (void)built_image;

    /*tifo::panorama::builder::CleanRectangularBuilder builder;
    builder.set_input_images(cut_images);
    builder.set_horizontal_slices(2).set_vertical_slices(4);
    tifo::image::Image* built_image = builder.build();
    builder.free_inputs();
    built_image->write("tests/new_julie.ppm");*/
    return 0;
}