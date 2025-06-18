#include <images/ppm-image.hh>

int main()
{
    tifo::image::PPMImage image;
    image.read("julie.ppm");
    image.write("autre_julie.ppm");
    return 0;
}