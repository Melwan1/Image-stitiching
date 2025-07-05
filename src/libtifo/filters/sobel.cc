#include <filters/sobel.hh>

namespace tifo::filter
{

    SobelX::SobelX()
        : Filter<int, 3>({ { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } })
    {}

    SobelY::SobelY()
        : Filter<int, 3>({ { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } })
    {}

    Sobel::Sobel()
        : BidirectionalFilter<int, 3>(SobelX(), SobelY()) {};

} // namespace tifo::filter