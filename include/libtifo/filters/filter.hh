#pragma once

#include <images/grayscale-image.hh>
#include <math/matrix.hh>

namespace tifo::filter
{

    template <typename ElementType, unsigned size>
    class Filter
    {
    public:
        Filter(const math::SquaredMatrix<ElementType, size>& matrix);
        virtual image::GrayscaleImage*
        apply_on_image(const image::GrayscaleImage* input_image);
        const math::SquaredMatrix<ElementType, size> get_matrix() const;

    protected:
        math::SquaredMatrix<ElementType, size> matrix_;
    };

} // namespace tifo::filter

#include "filter.hxx"