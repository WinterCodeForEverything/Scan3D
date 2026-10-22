#ifndef SCAN3D_CORE_RECTANGLE_H
#define SCAN3D_CORE_RECTANGLE_H

#include <algorithm>

#include <module/core/vec.h>


namespace scan3d {

    // Seems duplicating 2D bounding box?

    template <typename FT>
    class GenericRect {
    public:
        GenericRect(const Vec<2, FT>& p, const Vec<2, FT>& q) {
            x_min_ = std::min(q.x, p.x);
            x_max_ = std::max(q.x, p.x);
            y_min_ = std::min(q.y, p.y);
            y_max_ = std::max(q.y, p.y);

        }

        GenericRect(FT xmin, FT xmax, FT ymin, FT ymax)
            : x_min_(std::min(xmin, xmax))
            , y_min_(std::min(ymin, ymax))
            , x_max_(std::max(xmin, xmax))
            , y_max_(std::max(ymin, ymax))
        {
        }

        FT& x_min() { return x_min_; }
        FT& y_min() { return y_min_; }
        FT& x_max() { return x_max_; }
        FT& y_max() { return y_max_; }
        FT x_min() const { return x_min_; }
        FT y_min() const { return y_min_; }
        FT x_max() const { return x_max_; }
        FT y_max() const { return y_max_; }

        FT& x() { return x_min_; }
        FT& y() { return y_min_; }
        FT x() const { return x_min_; }
        FT y() const { return y_min_; }

        FT width()  const { return x_max() - x_min(); }
        FT height() const { return y_max() - y_min(); }

        FT& left() { return x_min_; }
        FT& top()  { return y_min_; }
        FT& right() { return x_max_; }
        FT& bottom() { return y_max_; }
        FT left() const { return x_min_; }
        FT top() const  { return y_min_; }
        FT right() const  { return x_max_; }
        FT bottom() const { return y_max_; }

        Vec<2, FT> top_left() const  { return Vec<2, FT>(x_min_, y_min_); }
        Vec<2, FT> bottom_right() const  { return Vec<2, FT>(x_max_, y_max_); }
        Vec<2, FT> top_right() const  { return Vec<2, FT>(x_max_, y_min_); }
        Vec<2, FT> bottom_left() const  { return Vec<2, FT>(x_min_, y_max_); }

        Vec<2, FT> min() const { return Vec<2, FT>(x_min_, y_min_); }
        Vec<2, FT> max() const { return Vec<2, FT>(x_max_, y_max_); }
        Vec<2, FT> center() const {
            return Vec<2, FT>(FT(0.5) * (x_max() + x_min()), FT(0.5) * (y_max() + y_min()));
        }

    private:
        FT x_min_;
        FT y_min_;
        FT x_max_;
        FT y_max_;
    };

}


#endif  // SCAN3D_CORE_RECTANGLE_H
