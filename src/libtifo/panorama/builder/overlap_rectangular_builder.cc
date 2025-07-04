#include <images/ppm-image.hh>
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

    int OverlapRectangularBuilder::get_overlap_x()
    {
        if (horizontal_slices_ == 1)
        {
            return 0;
        }
        image::Image* image_0_0 = input_images_[0];
        image::Image* image_1_0 = input_images_[1];
        double min_distance_x;
        int overlap_x = 0;
        for (int candidate_overlap_x = 1; candidate_overlap_x
             < std::min(image_0_0->get_width(), image_1_0->get_width());
             candidate_overlap_x++)
        {
            metrics::distance::ImageDistance image_distance;
            image_distance.set_input_images({ image_0_0, image_1_0 });
            image_distance.set_image_crop_grid(
                { image_0_0->get_width() - candidate_overlap_x, 0,
                  image_0_0->get_width(), image_0_0->get_height() },
                0);
            image_distance.set_image_crop_grid(
                { 0, 0, candidate_overlap_x, image_1_0->get_height() }, 1);
            double lab_distance = image_distance.compute_distance();
            if (overlap_x == 0 || lab_distance < min_distance_x)
            {
                min_distance_x = lab_distance;
                overlap_x = candidate_overlap_x;
            }
        }
        return overlap_x;
    }

    int OverlapRectangularBuilder::get_overlap_y()
    {
        if (vertical_slices_ == 1)
        {
            return 0;
        }
        image::Image* image_0_0 = input_images_[0];
        image::Image* image_0_1 = input_images_[horizontal_slices_];
        double min_distance_y;
        int overlap_y = 0;
        for (int candidate_overlap_y = 1; candidate_overlap_y
             < std::min(image_0_0->get_height(), image_0_1->get_height());
             candidate_overlap_y++)
        {
            std::cout << image_0_0 << "\n" << image_0_1 << "\n";
            metrics::distance::ImageDistance image_distance;
            image_distance.set_input_images({ image_0_0, image_0_1 });
            image_distance.set_image_crop_grid(
                { image_0_0->get_height() - candidate_overlap_y, 0,
                  image_0_0->get_width(), image_0_0->get_height() },
                0);
            image_distance.set_image_crop_grid(
                { 0, 0, image_0_1->get_width(), candidate_overlap_y }, 1);
            std::cout << "image 0 0: " << image_0_0->get_width() << "x"
                      << image_0_1->get_height();
            double lab_distance = image_distance.compute_distance();
            if (overlap_y == 0 || lab_distance < min_distance_y)
            {
                min_distance_y = lab_distance;
                overlap_y = candidate_overlap_y;
            }
        }
        return overlap_y;
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

        int overlap_x = get_overlap_x();
        int overlap_y = get_overlap_y();

        std::cout << "Overlap x = " << overlap_x << "\n";
        std::cout << "Overlap y = " << overlap_y << "\n";

        int total_width = 0;
        for (int horizontal_slice = 0; horizontal_slice < horizontal_slices_;
             horizontal_slice++)
        {
            total_width += input_images_[horizontal_slice]->get_width();
        }
        total_width -= (horizontal_slices_ - 1) * overlap_x;
        int total_height = 0;
        for (int vertical_slice = 0; vertical_slice < vertical_slices_;
             vertical_slice++)
        {
            total_height += input_images_[horizontal_slices_ * vertical_slice]
                                ->get_height();
        }
        total_height -= (vertical_slices_ - 1) * overlap_y;
        image::PPMImage* result =
            new image::PPMImage(total_width, total_height);

        for (int y = 0; y < result->get_height(); y++)
        {
            for (int x = 0; x < result->get_width(); x++)
            {
                // assume all images are of the same size
                int input_index_x = std::min(
                    x / (input_images_[0]->get_width() - overlap_x / 2),
                    horizontal_slices_ - 1);
                int input_index_y = std::min(
                    y / (input_images_[0]->get_height() - overlap_y / 2),
                    vertical_slices_ - 1);
                int dx = x
                    - input_index_x
                        * (input_images_[0]->get_width() - overlap_x / 2);
                int dy = y
                    - input_index_y
                        * (input_images_[0]->get_height() - overlap_y / 2);
                if (input_index_x > 0)
                {
                    dx += overlap_x / 2;
                }
                if (input_index_y > 0)
                {
                    dy += overlap_y / 2;
                }

                for (int color_index = 0; color_index < 3; color_index++)
                {
                    image::Image* retrieved_image =
                        input_images_[input_index_y * horizontal_slices_
                                      + input_index_x];
                    result->get_pixels()[y * result->get_width() + x]
                                        [color_index] =
                        retrieved_image
                            ->get_pixels()[dy * retrieved_image->get_width()
                                           + dx][color_index];
                }
            }
        }

        return result;
    }

} // namespace tifo::panorama::builder