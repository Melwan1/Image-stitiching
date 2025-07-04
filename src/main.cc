#include <images/ppm-image.hh>
#include <sstream>

int main()
{
    tifo::image::PPMImage image;
    image.read("tests/julie.ppm");
    std::vector<tifo::image::Image*> cut_images = image.rectangular_cut(2, 4);
    tifo::image::Image* rebuilt_image =
        tifo::image::Image::rebuild_from_cuts(cut_images, 2, 4);
    rebuilt_image->write("tests/new_julie.ppm");
    return 0;
}