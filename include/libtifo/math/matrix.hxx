#pragma once

#include <iostream>

#include "matrix.hh"
#include "vector.hh"

namespace tifo::math
{

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>::Matrix(const container_type& elements)
    {
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                elements_[line][column] = elements[line][column];
            }
        }
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>::Matrix(
        std::initializer_list<std::initializer_list<ElementType>> elements)
    {
        if (elements.size() != lines)
        {
            std::cerr << "Matrix constructor initializer_list has wrong number "
                         "of elements. Expected: "
                      << lines << ", got: " << elements.size() << "\n";
        }
        for (unsigned line = 0; line < lines; line++)
        {
            if (elements.begin()[line].size() != columns)
            {
                std::cerr << "Matrix constructor initializer_list at index "
                          << line << " has wrong number of elements. Expected: "
                          << columns
                          << ", got: " << elements.begin()[line].size() << "\n";
                return;
            }
            for (unsigned column = 0; column < columns; column++)
            {
                elements_[line][column] =
                    elements.begin()[line].begin()[column];
            }
        }
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>::Matrix()
    {
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                elements_[line][column] = get_element_base();
            }
        }
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    const ElementType* Matrix<ElementType, lines, columns>::to_array() const
    {
        return &elements_[0][0];
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    ElementType&
    Matrix<ElementType, lines, columns>::operator()(unsigned line,
                                                    unsigned column)
    {
        if (line >= lines)
        {
            std::cerr
                << "Matrix element query outside of bounds. Number of lines: "
                << lines << ", line queried: " << line << "\n";
            return get_element_base();
        }
        if (column >= columns)
        {
            std::cerr
                << "Matrix element query outside of bounds. Number of columns: "
                << columns << ", column queried: " << column << "\n";
            return get_element_base();
        }
        return elements_[line][column];
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    const ElementType&
    Matrix<ElementType, lines, columns>::operator()(unsigned line,
                                                    unsigned column) const
    {
        if (line >= lines)
        {
            std::cerr
                << "Matrix element query outside of bounds. Number of lines: "
                << lines << ", line queried: " << line << "\n";
            return get_element_base();
        }
        if (column >= columns)
        {
            std::cerr
                << "Matrix element query outside of bounds. Number of columns: "
                << columns << ", column queried: " << column << "\n";
            return get_element_base();
        }
        return elements_[line][column];
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>
    Matrix<ElementType, lines, columns>::operator+(
        const Matrix<ElementType, lines, columns>& rhs) const
    {
        Matrix result;
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                result(line, column) =
                    (*this)(line, column) + rhs(line, column);
            }
        }
        return result;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>
    Matrix<ElementType, lines, columns>::operator+=(
        const Matrix<ElementType, lines, columns>& rhs)
    {
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                (*this)(line, column) += rhs(line, column);
            }
        }
        return *this;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    template <unsigned rhs_columns>
    Matrix<ElementType, lines, rhs_columns>
    Matrix<ElementType, lines, columns>::operator*(
        const Matrix<ElementType, columns, rhs_columns>& rhs) const
    {
        Matrix result;
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < rhs_columns; column++)
            {
                ElementType new_element = 0;
                for (unsigned k = 0; k < columns; k++)
                {
                    new_element += (*this)(line, k) * rhs(k, column);
                }
                result(line, column) = new_element;
            }
        }
        return result;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>
    Matrix<ElementType, lines, columns>::operator*(ElementType scalar) const
    {
        Matrix result;
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                result(line, column) = (*this)(line, column) * scalar;
            }
        }
        return result;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>
    Matrix<ElementType, lines, columns>::operator*=(ElementType scalar)
    {
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                (*this)(line, column) *= scalar;
            }
        }
        return *this;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Vector<ElementType, lines> Matrix<ElementType, lines, columns>::operator*(
        const Vector<ElementType, columns>& rhs) const
    {
        Vector<ElementType, lines> result;
        for (unsigned line = 0; line < lines; line++)
        {
            for (unsigned column = 0; column < columns; column++)
            {
                result[line] += (*this)(line, column) * rhs[column];
            }
        }
        return result;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    ElementType& Matrix<ElementType, lines, columns>::get_element_base()
    {
        static ElementType element = ElementType();
        return element;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns>
    Matrix<ElementType, lines, columns>::identity()
        requires(lines == columns)
    {
        Matrix result;
        for (unsigned index = 0; index < lines; index++)
        {
            result(index, index) = get_element_base() + 1;
        }
        return result;
    }

    template <typename ElementType, unsigned lines, unsigned columns>
    Matrix<ElementType, lines, columns> Matrix<ElementType, lines, columns>::inverse() requires (lines == columns && lines == 3) {
        auto a = (*this)(0, 0);
        auto b = (*this)(0, 1);
        auto c = (*this)(0, 2);
        auto d = (*this)(1, 0);
        auto e = (*this)(1, 1);
        auto f = (*this)(1, 2);
        auto g = (*this)(2, 0);
        auto h = (*this)(2, 1);
        auto i = (*this)(2, 2);

        auto det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        return math::Matrix3({
            {e * i - f * h, c * h - b * i, b * f - c * e},
            {f * g - d * i, a * i - c * g, c * d - a * f},
            {d * h - e * g, b * g - a * h, a * e - b * d}
        }) * (1. / det);
    }

} // namespace tifo::math

template <typename ElementType, unsigned lines, unsigned columns>
std::ostream&
operator<<(std::ostream& ostr,
           const tifo::math::Matrix<ElementType, lines, columns>& matrix)
{
    for (unsigned line = 0; line < lines; line++)
    {
        if (line == 0)
        {
            ostr << "( | ";
        }
        else
        {
            ostr << "  | ";
        }
        for (unsigned column = 0; column < columns; column++)
        {
            if (column > 0)
            {
                ostr << ", ";
            }
            ostr << matrix(line, column);
        }
        if (line == lines - 1)
        {
            ostr << " | )";
        }
        else
        {
            ostr << " |  ";
        }
        ostr << "\n";
    }
    return ostr;
}
