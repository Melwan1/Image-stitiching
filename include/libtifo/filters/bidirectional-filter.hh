#pragma once

#include <filters/filter.hh>

namespace tifo::filter
{

    template <typename ElementType, unsigned size>
    class BidirectionalFilter : Filter<ElementType, size>
    {
    public:
        BidirectionalFilter(const Filter<ElementType, size>& horizontal_filter,
                            const Filter<ElementType, size>& vertical_filter);
        image::GrayscaleImage*
        apply_on_image(const image::GrayscaleImage* input_image) override;

    private:
        Filter<ElementType, size> horizontal_filter_;
        Filter<ElementType, size> vertical_filter_;
    };

} // namespace tifo::filter

#include "bidirectional-filter.hxx"