#pragma once

#include <vector>

namespace tifo::panorama::sift
{

    struct KeyPoint
    {
        float x, y; // position
        float scale; // sigma
        float orientation; // dominant orientation in degrees
        int octave; // octave level
        int layer; // layer within octave
        std::vector<float> descriptor; // size of 128

        KeyPoint();
        KeyPoint(float x, float y, float scale, float orientation, int octave,
                 int layer);
    };

} // namespace tifo::panorama::sift