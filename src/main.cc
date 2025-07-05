#include <images/ppm-image.hh>
#include <iostream>
#include <metrics/distance/image_distance.hh>
#include <panorama/builder/overlap_rectangular_builder.hh>
#include <panorama/cutter/overlap_rectangular_cutter.hh>
#include <sstream>
#include <yaml-cpp/yaml.h>

#include "libtifo/config/config_launcher.hh"

int main()
{
    tifo::panorama::config::ConfigLauncher config_launcher =
        tifo::panorama::config::ConfigLauncher("config.yaml");

    return 0;
}