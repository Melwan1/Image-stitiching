#include <images/ppm-image.hh>
#include <iostream>
#include <metrics/distance/image_distance.hh>
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
        .set_vertical_overlap_size(10);
    std::vector<tifo::image::Image*> cut_images = cutter.cut();
    int index = 1;
    for (const auto cut_image : cut_images)
    {
        std::ostringstream oss;
        oss << "tests/cut_temporary_lab_" << index++ << ".ppm";
        cut_image->write(oss.str());
    }

    tifo::panorama::builder::OverlapRectangularBuilder builder;
    builder.set_input_images(cut_images);
    builder.set_horizontal_slices(2).set_vertical_slices(2);
    tifo::image::Image* built_image = builder.build();
    built_image->write("tests/result.ppm");

    tifo::metrics::distance::ImageDistance image_distance;
    std::cout << "built image: " << built_image->get_width() << "x"
              << built_image->get_height() << "\n";
    std::cout << "initial image: " << image->get_width() << "x"
              << image->get_height() << "\n";
    image_distance.set_input_images({ built_image, image });
    double distance = image_distance.compute_distance();
    std::cout << "cut distance with respect to original image: " << distance
              << "\n";

    return 0;
}