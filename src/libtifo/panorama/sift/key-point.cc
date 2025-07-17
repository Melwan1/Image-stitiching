#include <panorama/sift/key-point.hh>

namespace tifo::panorama::sift
{

    KeyPoint::KeyPoint()
        : x(0)
        , y(0)
        , scale(0)
        , orientation(0)
        , octave(0)
        , layer(0)
    {}

    KeyPoint::KeyPoint(float x, float y, float scale, float orientation,
                       int octave, int layer)
        : x(x)
        , y(y)
        , scale(scale)
        , orientation(orientation)
        , octave(octave)
        , layer(layer)
    {}

} // namespace tifo::panorama::sift