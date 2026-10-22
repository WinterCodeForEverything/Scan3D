#ifndef SCAN3D_CORE_CURVE_H
#define SCAN3D_CORE_CURVE_H


#include <vector>


namespace scan3d {

    /*
    unsigned int steps = 20;
    {
        vec3 a(0, 0, 0);
        vec3 b(800, 0, 0);
        vec3 c(800, 800, 0);

        std::vector<vec3> points;
        curve::quadratic(a, b, c, steps, points);
        std::cout << "first point: " << points.front() << ", last point: " << points.back() << std::endl;

        std::vector<unsigned int> indices;
        for (unsigned int i=0; i<points.size() - 1; ++i) {
            indices.push_back(i);
            indices.push_back(i+1);
        }
        LinesDrawable *curve = new LinesDrawable;
        curve->update_vertex_buffer(points);
        curve->update_element_buffer(indices);
        curve->set_impostor_type(scan3d::LinesDrawable::CYLINDER);
        curve->set_line_width(5);
        curve->set_uniform_coloring(vec4(0, 0, 1, 1));
        viewer.add_drawable(curve);
    }

    {
        vec3 a(0, 0, 0);
        vec3 b(400, 0, 0);
        vec3 c(400, 800, 0);
        vec3 d(800, 800, 0);
        std::vector<vec3> points;
        curve::cubic(a, b, c, d, steps, points);
        std::cout << "first point: " << points.front() << ", last point: " << points.back() << std::endl;

        std::vector<unsigned int> indices;
        for (unsigned int i = 0; i < points.size() - 1; ++i) {
            indices.push_back(i);
            indices.push_back(i + 1);
        }
        LinesDrawable *curve = new LinesDrawable;
        curve->update_vertex_buffer(points);
        curve->update_element_buffer(indices);
        curve->set_impostor_type(scan3d::LinesDrawable::CYLINDER);
        curve->set_line_width(5);
        curve->set_uniform_coloring(vec4(0, 1, 0, 1));
        viewer.add_drawable(curve);
    }
     */

    namespace curve {

        /**
         * De Casteljau algorithm evaluating a quadratic or conic (second degree) curve.
         * Works for both 2D and 3D.
         * @param curve Returns the sequence of points on the curve.
         * @param bezier_steps Controls the smoothness of the curved corners. A greater value results in a smoother
         *      transitions but more vertices. Suggested value is 4.
         * @param include_end Ture to extend the curve to the end point.
         */
        template<typename Point>
        inline
        void quadratic(const Point &A, const Point &B, const Point &C, std::vector<Point> &curve,
                       unsigned int bezier_steps = 4, bool include_end = false) {
            typedef typename Point::FT FT;
            for (unsigned int i = 0; i < bezier_steps; i++) {
                const FT t = static_cast<FT>(i) / bezier_steps;
                const Point U = (1.0 - t) * A + t * B;
                const Point V = (1.0 - t) * B + t * C;
                curve.push_back((1.0 - t) * U + t * V);
            }

            if (include_end)
                curve.push_back(C);
        }

        /**
         * De Casteljau algorithm evaluating a cubic (third degree) curve.
         * Works for both 2D and 3D.
         * @param curve Returns the sequence of points on the curve.
         * @param bezier_steps Controls the smoothness of the curved corners. A greater value results in a smoother
         *      transitions but more vertices. Suggested value is 4.
         * @param include_end Ture to extend the curve to the end point.
         */
        template<typename Point>
        inline
        void cubic(const Point &A, const Point &B, const Point &C, const Point &D, std::vector<Point> &curve,
                   unsigned int bezier_steps = 4, bool include_end = false) {
            typedef typename Point::FT FT;
            for (unsigned int i = 0; i < bezier_steps; i++) {
                const FT t = static_cast<FT>(i) / bezier_steps;
                const Point U = (1.0 - t) * A + t * B;
                const Point V = (1.0 - t) * B + t * C;
                const Point W = (1.0 - t) * C + t * D;
                const Point M = (1.0 - t) * U + t * V;
                const Point N = (1.0 - t) * V + t * W;
                curve.push_back((1.0 - t) * M + t * N);
            }
            if (include_end)
                curve.push_back(D);
        }

    }   // namespace curve

}   // namespace scan3d


#endif  // SCAN3D_CORE_CURVE_H
