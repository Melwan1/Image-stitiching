#include <images/ppm-image.hh>

#include <sstream>

int main()
{
    tifo::image::PPMImage image;
    image.read("tests/julie.ppm");
    std::vector<tifo::image::Image*> cut_images = image.rectangular_cut(2, 4);
    int index = 1;
    for (const auto& cut_image : cut_images) {
        std::ostringstream oss;
        oss << "tests/cut_julie_" << index++ << ".ppm";
        cut_image->write(oss.str());
    }
    return 0;
}