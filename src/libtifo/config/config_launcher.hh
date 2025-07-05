#pragma once

#include <string>

#include "images/image.hh"
#include "yaml-cpp/node/iterator.h"

namespace tifo::panorama::config
{

    class ConfigLauncher
    {
    public:
        ConfigLauncher(const std::string& config_path);

        bool launch_overlap_rectangular_cutter(const YAML::Node& pipeline_node);
        bool
        launch_overlap_rectangular_builder(const YAML::Node& pipeline_node);

        // Fill pipeline_images from the input node
        bool add_input_image_to_pipeline(const YAML::Node& image_node);
        bool add_input_images_to_pipeline(const YAML::Node& images_node);

    private:
        std::string config_path_;
        std::string current_directory_;

        tifo::image::Image* initial_image_ = nullptr;
        std::vector<tifo::image::Image*> pipeline_images_;
    };

} // namespace tifo::panorama::config