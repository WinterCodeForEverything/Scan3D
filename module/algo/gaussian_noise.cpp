#include <module/algo/gaussian_noise.h>
#include <module/core/point_cloud.h>
#include <module/core/surface_mesh.h>
#include <module/core/random.h>
#include <module/util/logging.h>

#include <chrono>
#include <random>


namespace scan3d {

    void GaussianNoise::apply(SurfaceMesh *mesh, float sigma) {
        if (!mesh) {
            LOG(WARNING) << "empty surface mesh";
            return;
        }

        if (sigma <= 0) {
            LOG(WARNING) << "sigma must be positive";
            return;
        }

        // Define random generator with Gaussian distribution
        const double mean = 0.0;
        // construct a trivial random generator engine from a time-based seed:
        unsigned int seed = (unsigned int) std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(mean, sigma);

        auto points = mesh->get_vertex_property<vec3>("v:point");
        for (auto v : mesh->vertices()) {
            double offset = distribution(generator);
            vec3 dir(random_float(-1.0f, 1.0f), random_float(-1.0f, 1.0f),
                     random_float(-1.0f, 1.0f));
            dir = normalize(dir);
            points[v] = points[v] + dir * offset;
        }

        // update normals if exist
        if (mesh->get_vertex_property<vec3>("v:normal"))
            mesh->update_vertex_normals();
        if (mesh->get_face_property<vec3>("f:normal"))
            mesh->update_face_normals();
    }


    void GaussianNoise::apply(PointCloud *cloud, float sigma) {
        if (!cloud) {
            LOG(WARNING) << "empty point cloud";
            return;
        }

        if (sigma <= 0) {
            LOG(WARNING) << "sigma must be positive";
            return;
        }

        // Define random generator with Gaussian distribution
        const double mean = 0.0;
        // construct a trivial random generator engine from a time-based seed:
        unsigned int seed = (unsigned int) std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(mean, sigma);

        auto points = cloud->get_vertex_property<vec3>("v:point");
        for (auto v : cloud->vertices()) {
            double offset = distribution(generator);
            vec3 dir(random_float(-1.0f, 1.0f), random_float(-1.0f, 1.0f),
                     random_float(-1.0f, 1.0f));
            dir = normalize(dir);
            points[v] = points[v] + dir * offset;
        }
    }

}