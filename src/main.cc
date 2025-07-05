#include <config/config_launcher.hh>
#include <images/ppm-image.hh>
#include <iostream>
#include <metrics/distance/image_distance.hh>
#include <panorama/builder/overlap_rectangular_builder.hh>
#include <panorama/cutter/overlap_rectangular_cutter.hh>
#include <sstream>
#include <yaml-cpp/yaml.h>

int main()
{
    tifo::config::ConfigLauncher config_launcher("config.yaml");

    return 0;
}