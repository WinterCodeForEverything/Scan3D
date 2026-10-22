#ifndef SCAN3D_CORE_CONSTANT_H
#define SCAN3D_CORE_CONSTANT_H

#include <cfloat>
#include <climits>

namespace scan3d {

#ifndef M_PI
#define M_PI			3.14159265358979323846264338327950288419716939937510582   // pi
#endif

#define half_pi			M_PI * 0.5
#define quarter_pi		M_PI * 0.25
#define two_pi			M_PI * 2.0          
#define rad2deg(a)		((a) * 180.0 / M_PI)
#define deg2rad(a)		((a) * M_PI / 180.0)

    // Function returning min/max for corresponding type
    template <typename FT> inline FT min();
    template <typename FT> inline FT max();
    // Template specializations for float and double
    template <> inline int   min<int>() { return INT_MIN; }
    template <> inline int   max<int>() { return INT_MAX; }
    template <> inline float   min<float>() { return FLT_MIN; }
    template <> inline float   max<float>() { return FLT_MAX; }
    template <> inline double  min<double>() { return DBL_MIN; }
    template <> inline double  max<double>() { return DBL_MAX; }

    // standard epsilon values
    // Function returning epsilon for corresponding type
    template <typename FT> inline FT epsilon();
    template <typename FT> inline FT epsilon_sqr();

    // Template specializations for float and double
    template <> inline float  epsilon<float>() { return 1.0e-6f; }
    template <> inline float  epsilon_sqr<float>() { return 1.0e-12f; }
    template <> inline double epsilon<double>() { return 1.0e-12; }
    template <> inline double epsilon_sqr<double>() { return 1.0e-24; }
}


#endif // SCAN3D_CORE_CONSTANT_H
