#include <config/config-launcher.hh>
#include <config/function-timer.hh>
#include <iostream>

#include "images/color-ppm-image.hh"
#include "metrics/distance/color-image-distance.hh"
#include "panorama/builder/builder.hh"
#include "panorama/builder/overlap-rectangular-builder.hh"
#include "panorama/cutter/overlap-rectangular-cutter.hh"
#include "yaml-cpp/yaml.h"

namespace tifo::config
{
    ConfigLauncher::ConfigLauncher(const std::string& config_path)
        : config_path_(config_path)
    {
        FunctionTimer function_timer("tifo::config::ConfigLauncher",
                                     "ConfigLauncher constructor");

        YAML::Node node = YAML::LoadFile(config_path);

        std::cout << "Loading config file: " << config_path << std::endl;

        YAML::Emitter emitter;
        emitter << node;
        std::cout << emitter.c_str() << std::endl;

        current_directory_ = node["current_directory"].as<std::string>();

        for (auto pipeline_node : node["pipeline"])
        {
            const std::string algorithm =
                pipeline_node["algorithm"].as<std::string>();

            if (algorithm == "overlap-rectangular-cutter")
            {
                launch_overlap_rectangular_cutter(pipeline_node);
            }
            else if (algorithm == "overlap-rectangular-builder")
            {
                launch_overlap_rectangular_builder(pipeline_node);
            }
        }
    }

    bool ConfigLauncher::launch_overlap_rectangular_cutter(
        const YAML::Node& pipeline_node)
    {
        FunctionTimer function_timer("tifo::config::ConfigLauncher",
                                     "launch_overlap_rectangular_cutter");
        // If no pipeline_images_ are set beforehand in the pipeline
        // we expect an input image
        if (pipeline_images_.empty())
        {
            if (!add_input_image_to_pipeline(pipeline_node["input"]["image"]))
            {
                return false;
            }
        }

        // Get the parameters of the algorithm
        std::map<std::string, int> parameters =
            pipeline_node["input"]["parameters"]
                .as<std::map<std::string, int>>();

        panorama::cutter::OverlapRectangularCutter cutter;

        // Set the parameters of the cutter
        cutter.set_horizontal_slices(parameters["horizontal_slices"])
            .set_vertical_slices(parameters["vertical_slices"])
            .set_horizontal_overlap_size(parameters["horizontal_overlap_size"])
            .set_vertical_overlap_size(parameters["vertical_overlap_size"]);

        // Get the file prefix for all the generated images
        const std::string file_prefix =
            pipeline_node["output"]["file_prefix"].as<std::string>();

        int output_image_index = 1;

        std::vector<image::ColorImage*> new_pipeline_images;

        for (const auto image : pipeline_images_)
        {
            // Set the image to cut
            cutter.set_input_image(image);

            std::cout << "initial image: " << image->get_width() << "x"
                      << image->get_height() << "\n";

            const std::vector<tifo::image::ColorImage*> cut_images = cutter.cut();

            // Actually write the images and put them in the pipeline
            for (const auto cut_image : cut_images)
            {
                std::ostringstream oss;
                oss << current_directory_ << "/" << file_prefix
                    << output_image_index++ << ".ppm";
                cut_image->write(oss.str());

                // Put the images in the pipeline
                new_pipeline_images.emplace_back(cut_image);
            }
        }

        pipeline_images_ = new_pipeline_images;

        return true;
    }

    bool ConfigLauncher::launch_overlap_rectangular_builder(
        const YAML::Node& pipeline_node)
    {
        FunctionTimer function_timer("tifo::config::ConfigLauncher",
                                     "launch_overlap_rectangular_builder");

        // If no pipeline_images_ are set beforehand in the pipeline
        // we expect a list of input images
        if (pipeline_images_.empty())
        {
            if (!add_input_images_to_pipeline(pipeline_node["input"]["images"]))
            {
                return false;
            }
        }

        // Get the parameters of the algorithm
        std::map<std::string, int> parameters =
            pipeline_node["input"]["parameters"]
                .as<std::map<std::string, int>>();

        panorama::builder::OverlapRectangularBuilder builder;

        // Set the input images to the pipeline images
        builder.set_input_images(pipeline_images_);

        // Set the parameters of the cutter
        builder.set_horizontal_slices(parameters["horizontal_slices"])
            .set_vertical_slices(parameters["vertical_slices"]);

        // Create the image
        image::ColorImage* built_image = builder.build();

        // Get the file prefix for all the generated images
        const std::string output_filename =
            pipeline_node["output"]["filename"].as<std::string>();
        const std::string output_file_type =
            pipeline_node["output"]["type"].as<std::string>();

        // Write the build image to the corresponding path
        built_image->write(current_directory_ + "/" + output_filename + "."
                           + output_file_type);

        // Metrics and prints
        metrics::distance::ColorImageDistance image_distance;

        std::cout << "initial image: " << initial_image_->get_width() << "x"
                  << initial_image_->get_height() << "\n";
        std::cout << "built image: " << built_image->get_width() << "x"
                  << built_image->get_height() << "\n";

        image_distance.set_input_images({ built_image, initial_image_ });
        double distance = image_distance.compute_distance();
        std::cout << "cut distance with respect to original image: " << distance
                  << "\n";

        // Clear the pipeline images and add the result
        pipeline_images_.clear();
        pipeline_images_.emplace_back(built_image);

        return true;
    }

    bool
    ConfigLauncher::add_input_image_to_pipeline(const YAML::Node& image_node)
    {
        // Get the information of the base image input
        const std::string input_image_type =
            image_node["type"].as<std::string>();
        const std::string input_image_filename =
            image_node["filename"].as<std::string>();

        const std::string input_image_file_path = current_directory_ + "/"
            + input_image_filename + "." + input_image_type;

        image::ColorImage* image;
        if (input_image_type == "ppm")
        {
            image = new tifo::image::ColorPPMImage();
            image->read(input_image_file_path);
        }
        else
        {
            // What is your format bro
            std::cerr << "tifo::panorama::config: Unknown input image type: "
                      << input_image_file_path << std::endl;
            return false;
        }

        // Set this image to the initial image if it wasn't already
        if (initial_image_ == nullptr)
        {
            std::cout << "Adding initial image to the pipeline: "
                      << input_image_file_path << std::endl;

            initial_image_ = image;
        }
        else
        {
            std::cout << "Adding image to the pipeline: "
                      << input_image_file_path << std::endl;
        }

        // Add it the pipeline images
        pipeline_images_.emplace_back(image);
        return true;
    };

    bool
    ConfigLauncher::add_input_images_to_pipeline(const YAML::Node& images_node)
    {
        // images_node is a list of images
        for (auto image_node : images_node)
        {
            // Add all the images to the pipeline
            if (!add_input_image_to_pipeline(image_node))
            {
                return false;
            }
        }

        return true;
    };

} // namespace tifo::config
