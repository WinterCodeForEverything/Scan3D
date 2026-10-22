#ifndef SCAN3D_KD_TREE_SEARCH_ANN_H
#define SCAN3D_KD_TREE_SEARCH_ANN_H

#include <module/kdtree/kdtree_search.h>


namespace scan3d {

    class PointCloud;

    class KdTreeSearch_ANN : public KdTreeSearch  {
    public:
        KdTreeSearch_ANN();
        virtual ~KdTreeSearch_ANN();


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

        //___________________ radius search __________________________

        // Liangliang: it seems ANN's annkFRSearch() needs to specify k.
        /* "It does two things. First, it computes the k nearest neighbors within the radius bound.
            Second, it returns the total number of points lying within the radius bound. It is
            permitted to set k = 0, in which case it only answers a range counting query. */
        void set_k_for_radius_search(int k) { k_for_radius_search_ = k; }

        // fixed-radius search. Search for all points in the range.
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
        int		points_num_;

        float**	points_; // a copy of the point cloud data (due to different data structure);

        void*	tree_;
        int		k_for_radius_search_;
    } ;

} // namespace scan3d

#endif  // SCAN3D_KD_TREE_SEARCH_ANN_H


