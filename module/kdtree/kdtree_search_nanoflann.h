#ifndef SCAN3D_KD_TREE_SEARCH_NANOFLANN_H
#define SCAN3D_KD_TREE_SEARCH_NANOFLANN_H

#include <module/kdtree/kdtree_search.h>

namespace scan3d {

    class PointCloud;

    class KdTreeSearch_NanoFLANN : public KdTreeSearch  {
    public:
        KdTreeSearch_NanoFLANN();
        virtual ~KdTreeSearch_NanoFLANN();

        //______________ tree construction __________________________

        virtual void begin() ;
        virtual void add_point_cloud(PointCloud* cloud) ;
        virtual void end() ;

        //________________ closest point ____________________________

        // NOTE: *squared* distance is returned
        virtual int find_closest_point(const vec3& p, float& squared_distance) const ;
        virtual int find_closest_point(const vec3& p) const ;

        //_________________ K-nearest neighbors ____________________

        // NOTE: *squared* distances are returned
        virtual void find_closest_k_points(
            const vec3& p, int k,
            std::vector<int>& neighbors, std::vector<float>& squared_distances
            ) const ;

        virtual void find_closest_k_points(
            const vec3& p, int k,
            std::vector<int>& neighbors
            ) const ;

        //___________________ radius search ___________________________

        // fixed-radius	search. Search for all points in the range.
        // NOTE: the range must be *squared* radius and *squared* distances are returned
        virtual void find_points_in_range(
            const vec3& p, float squared_radius,
            std::vector<int>& neighbors, std::vector<float>& squared_distances
        ) const;

        // fixed-radius search. Search for all points in the range.
        // NOTE: the range must be *squared* radius
        virtual void find_points_in_range(
            const vec3& p, float squared_radius,
            std::vector<int>& neighbors
        ) const;

    protected:
        std::vector<vec3>*	points_; // reference of the original point cloud data
        void*				tree_;
    } ;

} // namespace scan3d

#endif  // SCAN3D_KD_TREE_SEARCH_NANOFLANN_H


