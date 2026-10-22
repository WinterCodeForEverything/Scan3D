#ifndef SCAN3D_RENDERER_SETTING_H
#define SCAN3D_RENDERER_SETTING_H


#include <module/core/types.h>


namespace scan3d {

    class ClippingPlane;

    // the default parameters used for initialization of both ui and rendering
    namespace setting {

        // ui
        extern vec4 background_color;

        // lighting
        extern vec4 light_position;  // light position defined in camera coordinate system
        extern bool light_distinct_back_color;
        extern vec4 back_color;
        extern bool light_two_sides;

        // material
        extern vec4 material_ambient;
        extern vec4 material_specular;
        extern float material_shininess;    // specular power

        // effect
        extern int effect_ssao_algorithm;
        extern float effect_ssao_radius;
        extern float effect_ssao_intensity;
        extern float effect_ssao_bias;
        extern float effect_ssao_sharpness;
        extern bool effect_edl_enabled;
        extern bool effect_transparency_enabled;
        extern bool effect_shadow_enabled;
        extern float shadow_light_distance;
        extern float shadow_softness;
        extern float shadow_darkness;

        // point cloud
        extern bool point_cloud_use_color_property;
        extern bool point_cloud_show_points;
        extern vec4 point_cloud_points_color;
        extern float point_cloud_point_size;
        extern bool point_cloud_impostors;

        // surface mesh - surface
        extern bool surface_mesh_phong_shading;
        extern bool surface_mesh_show_faces;
        extern vec4 surface_mesh_faces_color;
        extern bool surface_mesh_use_color_property;
        extern float surface_mesh_opacity;

        // surface mesh - vertices
        extern bool surface_mesh_show_vertices;
        extern vec4 surface_mesh_vertices_color;
        extern bool surface_mesh_vertices_imposters;
        extern float surface_mesh_vertices_point_size;

        // surface mesh - edges
        extern bool surface_mesh_show_edges;
        extern vec4 surface_mesh_edges_color;
        extern bool surface_mesh_edges_imposters;
        extern float surface_mesh_edges_line_width;

        // surface mesh - borders
        extern bool surface_mesh_show_borders;
        extern vec4 surface_mesh_borders_color;
        extern bool surface_mesh_borders_imposters;
        extern float surface_mesh_borders_line_width;

        // graph - vertices
        extern bool graph_show_vertices;
        extern vec4 graph_vertices_color;
        extern float graph_vertices_point_size;

        // surface mesh - edges
        extern bool graph_show_edges;
        extern vec4 graph_edges_color;
        extern float graph_edges_line_width;

        extern ClippingPlane* clipping_plane;
        extern vec4 clipping_plane_color;


    } // namespace setting

} // namespace scan3d


#endif // SCAN3D_RENDERER_SETTING_H
