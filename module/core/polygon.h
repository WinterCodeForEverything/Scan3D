#ifndef SCAN3D_CORE_POLYGON_H
#define SCAN3D_CORE_POLYGON_H


#include <vector>
#include <algorithm>

#include <module/core/vec.h>

namespace scan3d {

    template<typename FT>
    class GenericPolygon : public std::vector< Vec<2, FT> > {
        typedef std::vector< Vec<2, FT> > BaseClass;

    public:
        GenericPolygon() {}

        // Initialize with a know size.
        GenericPolygon(std::size_t size) { BaseClass::resize(size); }

        template<class InputIterator>
        GenericPolygon(InputIterator first, InputIterator last);

        /**
         * @brief Check if the polygon has a clockwise orientation (right-hand rule).
         * @details This method determines the orientation of a polygon based on the sign of its signed area:
         *              - negative: clockwise
         *              - positive: counterclockwise
         */
        bool is_clockwise() const;

        // Reverse the orientation of the polygon. The first vertex remains the same.
        // First vertex: pointed to by `p.begin()`, or [0]
        void reverse_orientation();

        // Test if point lies inside the polygon
        bool contains(const Vec<2, FT> &point) const;

        // Test if a polygon lies inside this polygon
        bool contains(const GenericPolygon<FT> &plg) const;

        // Return the signed area of the polygon. This means that the
        // area is positive for counter clockwise polygons and negative
        // for clockwise polygons.
        FT signed_area() const;

        // Return the absolute value of the signed_area
        FT area() const;

        // Return the smallest bounding box containing this polygon.
        GenericBox<2, FT> bbox() const;
    };

    //////////////////////////////////////////////////////////////////////////

    template<typename FT>
    template<class InputIterator>
    inline GenericPolygon<FT>::GenericPolygon(InputIterator first, InputIterator last) {
        std::copy(first, last, BaseClass::end());
    }

    template<typename FT>
    inline bool GenericPolygon<FT>::is_clockwise() const {
    #if 0
        return signed_area() < 0;
    #else
        double sum = 0;
        for (std::size_t i = 0; i + 1 < BaseClass::size(); ++i) {
            const auto &p1 = BaseClass::at(i);
            const auto &p2 = BaseClass::at((i + 1) % BaseClass::size());
            sum += (p2.x - p1.x) * (p2.y + p1.y);
        }
        return sum < 0.0;
    #endif
    }

    template<typename FT>
    inline void GenericPolygon<FT>::reverse_orientation() {
        if (BaseClass::size() <= 1)
            return;
        auto it = BaseClass::begin();
        std::reverse(++it, BaseClass::end());
    }


    template<typename FT>
    inline bool GenericPolygon<FT>::contains(const Vec<2, FT> &p) const {
        bool inside = false;
        std::size_t n = BaseClass::size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i, ++i) {
            const Vec<2, FT> &u0 = BaseClass::at(i);
            const Vec<2, FT> &u1 = BaseClass::at(j);  // current edge

            if (((u0.y <= p.y) && (p.y < u1.y)) ||  // U1 is above the ray, U0 is on or below the ray
                ((u1.y <= p.y) && (p.y < u0.y)))    // U0 is above the ray, U1 is on or below the ray
            {
                // find x-intersection of current edge with the ray.
                // Only consider edge crossings on the ray to the right of P.
                double x = u0.x + (p.y - u0.y) * (u1.x - u0.x) / (u1.y - u0.y);
                if (x > p.x)
                    inside = !inside;
            }
        }

        return inside;
    }

    template<typename FT>
    inline bool GenericPolygon<FT>::contains(const GenericPolygon<FT> &plg) const {
        for (const auto &p : plg) {
            if (!contains(p))
                return false;
        }
        return true;
    }

    template<typename FT>
    inline FT GenericPolygon<FT>::area() const {
        return std::fabs(signed_area());
    }


    // http://astronomy.swin.edu.au/~pbourke/geometry/polyarea/
    template<typename FT>
    inline FT GenericPolygon<FT>::signed_area() const {
        FT result = 0;
        for (unsigned int i = 0; i < BaseClass::size(); i++) {
            unsigned int j = (i + 1) % BaseClass::size();
            const Vec<2, FT> &t1 = BaseClass::at(i);
            const Vec<2, FT> &t2 = BaseClass::at(j);
            result += t1.x * t2.y - t2.x * t1.y;
        }
        result /= 2.0;
        return result;
    }


    template<typename FT>
    inline GenericBox<2, FT> GenericPolygon<FT>::bbox() const {
        GenericBox<2, FT> box;
        for (auto p : *this)
            box.add_point(p);
        return box;
    }

}


#endif  // SCAN3D_CORE_POLYGON_H
