#include <iostream>
#include <metrics/distance/image_distance.hh>
#include <panorama/builder/overlap_rectangular_builder.hh>

namespace tifo::panorama::builder
{

    OverlapRectangularBuilder&
    OverlapRectangularBuilder::set_horizontal_slices(int horizontal_slices)
    {
        if (horizontal_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::OverlapRectangularBuilder - cannot set "
                "nonpositive horizontal slices count.");
        }
        horizontal_slices_ = horizontal_slices;
        return *this;
    }

    OverlapRectangularBuilder&
    OverlapRectangularBuilder::set_vertical_slices(int vertical_slices)
    {
        if (vertical_slices <= 0)
        {
            throw std::runtime_error(
                "tifo::panorama::OverlapRectangularBuilder - cannot set "
                "nonpositive horizontal slices count.");
        }
        vertical_slices_ = vertical_slices;
        return *this;
    }

    image::Image* OverlapRectangularBuilder::build()
    {
        /** for now we will consider that the images are well placed with
         * respect to each other. We therefore just need to compute the overlap
         * size and glue the images back together. For simplicity, we will also
         * consider that the overlap is constant among the images, so we only
         * calculate the overlap for images at position (0, 0) and (0, 1) on one
         * hand and (0, 0) and (1, 0) on the other hand
         */

        // Compute between (0, 0) and (1, 0)
        for (const auto image : input_images_)
        {
            std::cout << "OverlapRectangularBuilder - image size : "
                      << image->get_width() << " x " << image->get_height()
                      << "\n";
        }
        image::Image* image_0_0 = input_images_[0];
        image::Image* image_1_0 = input_images_[1];
        image::Image* image_0_1 = input_images_[horizontal_slices_];
        image_0_0->write("image00.ppm");
        image_1_0->write("image10.ppm");
        image_0_1->write("image01.ppm");
        std::cout << "OverlapRectangularBuilder - image (0, 0) size : "
                  << image_0_0->get_width() << " x " << image_0_0->get_height()
                  << "\n";
        std::cout << "OverlapRectangularBuilder - image (1, 0) size : "
                  << image_1_0->get_width() << " x " << image_1_0->get_height()
                  << "\n";
        double min_distance_x;
        int overlap_x = 0;
        for (int candidate_overlap_x = 1; candidate_overlap_x
             < std::min(image_0_0->get_width(), image_0_1->get_width());
             candidate_overlap_x++)
        {
            metrics::distance::ImageDistance image_distance;
            image_distance.set_input_images({ image_0_0, image_1_0 });
            image_distance.set_image_crop_grid(
                { image_0_0->get_width() - candidate_overlap_x - 1, 0,
                  image_0_0->get_width(), image_0_0->get_height() },
                0);
            image_distance.set_image_crop_grid(
                { 0, 0, candidate_overlap_x + 1, image_1_0->get_height() }, 1);
            double lab_distance = image_distance.compute_distance();
            std::cout << "lab distance for overlap x = " << candidate_overlap_x
                      << " is " << lab_distance << "\n";
            if (overlap_x == 0 || lab_distance < min_distance_x)
            {
                min_distance_x = lab_distance;
                overlap_x = candidate_overlap_x;
            }
        }
        std::cout << "OverlapRectangularBuilder - image (0, 0) size : "
                  << image_0_0->get_width() << " x " << image_0_0->get_height()
                  << "\n";
        std::cout << "OverlapRectangularBuilder - image (0, 1) size : "
                  << image_0_1->get_width() << " x " << image_0_1->get_height()
                  << "\n";
        double min_distance_y;
        int overlap_y = 0;
        for (int candidate_overlap_y = 1; candidate_overlap_y
             < std::min(image_0_0->get_height(), image_1_0->get_height());
             candidate_overlap_y++)
        {
            metrics::distance::ImageDistance image_distance;
            image_distance.set_input_images({ image_0_0, image_0_1 });
            image_distance.set_image_crop_grid(
                { 0, image_0_0->get_height() - candidate_overlap_y - 1,
                  image_0_0->get_width(), image_0_0->get_height() },
                0);
            image_distance.set_image_crop_grid(
                { 0, 0, image_0_1->get_width(), candidate_overlap_y + 1 }, 1);
            double lab_distance = image_distance.compute_distance();
            std::cout << "lab distance for overlap y = " << candidate_overlap_y
                      << " is " << lab_distance << "\n";
            if (overlap_y == 0 || lab_distance < min_distance_y)
            {
                min_distance_y = lab_distance;
                overlap_y = candidate_overlap_y;
            }
        }

        std::cout << "Overlap x = " << overlap_x << "\n";
        std::cout << "Overlap y = " << overlap_y << "\n";

        return nullptr;
    }

} // namespace tifo::panorama::builder