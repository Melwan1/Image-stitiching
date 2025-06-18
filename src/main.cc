#include <images/ppm-image.hh>

int main()
{
    tifo::image::PPMImage cat_image;
    cat_image.read("cute_cat.ppm");
    cat_image.write("other_cute_cat.ppm");
    return 0;
}