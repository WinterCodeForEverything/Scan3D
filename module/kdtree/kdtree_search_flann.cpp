#include <module/kdtree/kdtree_search_flann.h>
#include <module/core/point_cloud.h>

#include <3rd_party/kd_tree/FLANN/flann.hpp>


#define get_tree(x) (reinterpret_cast<const flann::Index< flann::L2<float> > *>(x))

namespace scan3d {

    KdTreeSearch_FLANN::KdTreeSearch_FLANN()  {
        points_ = nullptr;
        points_num_ = 0;
        tree_ = nullptr;

        //checks_ = 32;
        checks_ = flann::FLANN_CHECKS_AUTOTUNED;
    }


    KdTreeSearch_FLANN::~KdTreeSearch_FLANN() {
        delete get_tree(tree_);
    }


    void KdTreeSearch_FLANN::set_checks(int chk) {
        checks_ = chk;
    }

    void KdTreeSearch_FLANN::begin()  {
        delete get_tree(tree_);
        tree_ = nullptr;
    }


    void KdTreeSearch_FLANN::end()  {
        flann::Matrix<float> dataset(points_, points_num_, 3);

        // construct a single kd-tree optimized for searching lower dimensionality data
        flann::Index< flann::L2<float> >* tree = new flann::Index< flann::L2<float> >(dataset, flann::KDTreeSingleIndexParams());
        tree->buildIndex();

        tree_ = tree;
    }


    void KdTreeSearch_FLANN::add_point_cloud(PointCloud* cloud)  {
        points_num_ = int(cloud->n_vertices());
        std::vector<vec3>& points = cloud->points();
        points_ = points[0];
    }


    int KdTreeSearch_FLANN::find_closest_point(const vec3& p, float& squared_distance) const {
        flann::Matrix<float> query(const_cast<float*>(p.data()), 1, 3);

        std::vector< std::vector<int> >		indices;
        std::vector< std::vector<float> >	dists;

        get_tree(tree_)->knnSearch(query, indices, dists, 1, flann::SearchParams(checks_));

        squared_distance = dists[0][0];
        return indices[0][0];
    }


    int KdTreeSearch_FLANN::find_closest_point(const vec3& p) const {
        float dist = 0;
        return find_closest_point(p, dist);
    }


    void KdTreeSearch_FLANN::find_closest_k_points(
        const vec3& p, int k, std::vector<int>& neighbors, std::vector<float>& squared_distances
        )  const
    {
        flann::Matrix<float> query(const_cast<float*>(p.data()), 1, 3);

        std::vector< std::vector<int> >		indices;
        std::vector< std::vector<float> >	dists;

        get_tree(tree_)->knnSearch(query, indices, dists, k, flann::SearchParams(checks_));

        neighbors = indices[0];
        squared_distances = dists[0];
    }


    void KdTreeSearch_FLANN::find_closest_k_points(
        const vec3& p, int k, std::vector<int>& neighbors
    )  const
    {
        std::vector<float> squared_distances;
        return find_closest_k_points(p, k, neighbors, squared_distances);
    }


    void KdTreeSearch_FLANN::find_points_in_range(
        const vec3& p, float squared_radius, std::vector<int>& neighbors, std::vector<float>& squared_distances
        )  const {
        flann::Matrix<float> query(const_cast<float*>(p.data()), 1, 3);

        std::vector< std::vector<int> >		indices;
        std::vector< std::vector<float> >	dists;

        get_tree(tree_)->radiusSearch(query, indices, dists, squared_radius, flann::SearchParams(checks_));

        size_t num = indices[0].size();
        neighbors.resize(num);
        squared_distances.resize(num);
        for (size_t i=0; i<num; ++i) {
            neighbors[i] = indices[0][i];
            squared_distances[i] = dists[0][i];
        }
    }


    void KdTreeSearch_FLANN::find_points_in_range(
        const vec3& p, float squared_radius, std::vector<int>& neighbors
    )  const
    {
        std::vector<float> squared_distances;
        return find_points_in_range(p, squared_radius, neighbors, squared_distances);
    }


} // namespace scan3d
