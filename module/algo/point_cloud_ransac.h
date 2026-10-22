#ifndef SCAN3D_ALGO_POINT_CLOUD_RANSAC_H
#define SCAN3D_ALGO_POINT_CLOUD_RANSAC_H


#include <set>
#include <vector>


namespace scan3d {

    class PointCloud;

    // usage:
    //    PrimitivesRansac ransac;
    //    ransac.add_primitive_type(PrimitivesRansac::PLANE);
    //    int num = ransac.detect(cloud);

    class PrimitivesRansac {
    public:
        enum PrimType { // values have to be exactly the same as in RANSAC
            PLANE = 0,
            SPHERE = 1,
            CYLINDER = 2,
            CONE = 3,
            TORUS = 4,
            UNKNOWN = -1
        };

    public:
        // add/remove the primitive type to be extracted
        void add_primitive_type(PrimType t);

        void remove_primitive_type(PrimType t);

        // extract primitive from the entire point cloud.
        // returns the number of extracted primitives.
        // result will store as properties:
        //      - "v:primitive_type"  (one of PLANE, SPHERE, CYLINDER, CONE, TORUS, and UNKNOWN)
        //      - "v:primitive_index" (-1, 0, 1, 2...). -1 meaning a vertex does not belong to any primitive (thus its
        //        primtive_type must be UNKNOWN.
        int detect(
                PointCloud *cloud,
                unsigned int min_support = 1000,    // the minimal number of points required for a primitive
                float dist_thresh = 0.005f,    // relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
                float bitmap_reso = 0.02f,    // relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
                float normal_thresh = 0.8f,    // the cos of the maximal normal deviation
                float overlook_prob = 0.001f    // the probability with which a primitive is overlooked
        );

        // extract primitive from a subset of a point cloud.
        // result will store as properties:
        //      - "v:primitive_type"  (one of PLANE, SPHERE, CYLINDER, CONE, TORUS, and UNKNOWN)
        //      - "v:primitive_index" (-1, 0, 1, 2...). -1 meaning a vertex does not belong to any primitive (thus its
        //        primtive_type must be UNKNOWN.
        int detect(
                PointCloud *cloud,
                const std::vector<int> &vertitces,
                unsigned int min_support = 1000,    // the minimal number of points required for a primitive
                float dist_thresh = 0.005f,    // relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
                float bitmap_reso = 0.02f,    // relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
                float normal_thresh = 0.8f,    // the cos of the maximal normal deviation
                float overlook_prob = 0.001f    // the probability with which a primitive is overlooked
        );

    private:
        std::set<PrimType> types_;
    };

}


#endif  //  SCAN3D_ALGO_POINT_CLOUD_RANSAC_H
