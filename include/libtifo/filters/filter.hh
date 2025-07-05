#pragma once

#include <images/image.hh>
#include <math/matrix.hh>

namespace tifo::filter
{

    template <typename ElementType, unsigned size>
    class Filter
    {
    public:
        Filter(const math::SquaredMatrix<ElementType, size>& matrix);
        virtual image::Image* apply_on_image(const image::Image* input_image);

    protected:
        math::SquaredMatrix<ElementType, size> matrix_;
    };

} // namespace tifo::filter

#include "filter.hxx"