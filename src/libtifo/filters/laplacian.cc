#include <filters/laplacian.hh>

namespace tifo::filter
{

    Laplacian::Laplacian()
        : Filter<int, 3>({ { 0, -1, 0 }, { -1, 4, -1 }, { 0, -1, 0 } })
    {}

} // namespace tifo::filter