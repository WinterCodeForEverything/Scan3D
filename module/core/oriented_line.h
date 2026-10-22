#ifndef SCAN3D_CORE_ORIENTED_LINE_H
#define SCAN3D_CORE_ORIENTED_LINE_H

#include <module/core/vec.h>


namespace scan3d {


    enum Sign {
        NEGATIVE = -1,
        ZERO = 0,
        POSITIVE = 1,
    };

    template<class T>
    inline Sign sign(T x) {
        return (x > 0) ? POSITIVE : ((x < 0) ? NEGATIVE : ZERO);
    }

    /**
     * OrientedLine implements plucker coordinates, which enables
     *  oriented lines to be compared. The comparison, implemented by the
     *  side() function, is a predicate similar to the right hand rule.
     * For instance, this class is used for the line-polygon intersection test.
     * (see D.M.Y. Sommerville, Analytical Geometry of Three Dimensions.
     *  Cambridge University Press, 1959).
     */
    template<class FT>
    class GenericOrientedLine {
    public:
        typedef Vec<3, FT> Point;

        GenericOrientedLine(const Point &p, const Point &q);

        GenericOrientedLine();

        /**
         * "right hand rule" like predicate.
         * @return +1 Positive side, -1 for negative side.
         */
        static Sign side(const GenericOrientedLine<FT> &a, const GenericOrientedLine<FT> &b);

    private:
        FT pi_[6];
    };

    //_________________________________________________________

    template<class FT>
    inline
    GenericOrientedLine<FT>::GenericOrientedLine(const Point &p, const Point &q) {
        // There are several conventions for defining plucker coordinates,
        // this one is introduced in : Marco Pellegrini, Stabbing and
        // ray-shooting in 3-dimensional space. In Proc. 6th ACM Symposium
        // on Computational Geometry, pages 177-186, 1990.
        // I think that it is possible to have a more symmetric formulation
        // of plucker coordinates, leading to a more symmetric side()
        // predicate, but I have no time to investigate this.
        pi_[0] = p.x * q.y - p.y * q.x;
        pi_[1] = p.x * q.z - p.z * q.x;
        pi_[2] = p.x - q.x;
        pi_[3] = p.y * q.z - p.z * q.y;
        pi_[4] = p.z - q.z;
        pi_[5] = q.y - p.y;
    }

    template<class FT>
    inline
    GenericOrientedLine<FT>::GenericOrientedLine() {
        pi_[0] = 0;
        pi_[1] = 0;
        pi_[2] = 0;
        pi_[3] = 0;
        pi_[4] = 0;
        pi_[5] = 0;
    }


    template<class FT>
    inline
    Sign GenericOrientedLine<FT>::side(const GenericOrientedLine<FT> &a, const GenericOrientedLine<FT> &b) {
        // Note: the order might seem strange, but product between
        //  lines in plucker coordinates is a permuted cross product.
        FT cross_product =
                a.pi_[0] * b.pi_[4] +
                a.pi_[1] * b.pi_[5] +
                a.pi_[2] * b.pi_[3] +
                a.pi_[4] * b.pi_[0] +
                a.pi_[5] * b.pi_[1] +
                a.pi_[3] * b.pi_[2];

        return sign(cross_product);
    }

}

#endif //SCAN3D_CORE_ORIENTED_LINE_H
