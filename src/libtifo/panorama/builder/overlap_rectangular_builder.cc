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
        image::Image* image_0_0 = input_images_[0];
        image::Image* image_1_0 = input_images_[1];
        std::vector<double> distances_by_overlap_x;
        distances_by_overlap_x.resize(image_0_0->get_width()
                                      - 2); // the heck is this shit??
        for (int candidate_overlap_x = 1; candidate_overlap_x
             < std::min(image_0_0->get_width(), image_1_0->get_width());
             candidate_overlap_x++)
        {
            metrics::distance::ImageDistance image_distance;
            image_distance.set_input_images({ image_0_0, image_1_0 });
            image_distance.set_image_crop_grid(
                { image_0_0->get_width() - candidate_overlap_x - 1, 0,
                  image_0_0->get_width(), image_0_0->get_height() },
                0);
            image_distance.set_image_crop_grid(
                { 0, 0, candidate_overlap_x, image_1_0->get_height() }, 1);
            double lab_distance = image_distance.compute_distance();
            std::cout << "lab distance for overlap x = " << candidate_overlap_x
                      << " is " << lab_distance << "\n";
            distances_by_overlap_x[candidate_overlap_x - 1] = lab_distance;
        }
        image::Image* image_1_1 = input_images_[horizontal_slices_];
        std::vector<double> distances_by_overlap_y;
        distances_by_overlap_y.resize(image_0_0->get_height()
                                      - 2); // again, what the flip is this
        for (int candidate_overlap_y = 1; candidate_overlap_y
             < std::min(image_0_0->get_height(), image_1_0->get_height());
             candidate_overlap_y++)
        {
            metrics::distance::ImageDistance image_distance;
            image_distance.set_input_images({ image_0_0, image_1_1 });
            image_distance.set_image_crop_grid(
                { 0, image_0_0->get_height() - candidate_overlap_y - 1,
                  image_0_0->get_width(), image_0_0->get_height() },
                0);
            image_distance.set_image_crop_grid(
                { 0, 0, image_1_1->get_width(), candidate_overlap_y }, 1);
            double lab_distance = image_distance.compute_distance();
            std::cout << "lab distance for overlap y = " << candidate_overlap_y
                      << " is " << lab_distance << "\n";
            distances_by_overlap_y[candidate_overlap_y - 1] = lab_distance;
        }

        return nullptr;
    }

} // namespace tifo::panorama::builder