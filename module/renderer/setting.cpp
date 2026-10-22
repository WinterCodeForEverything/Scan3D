#include <module/renderer/setting.h>


namespace scan3d {

    // the default parameters used for initialization of both ui and rendering
    namespace setting {

        // ui
        vec4 background_color = vec4(1.0f, 1.0f, 1.0f, 1.0f);

        // lighting
        vec4 light_position = vec4(0.27f, 0.27f, 0.92f, 0.0f);
        bool light_distinct_back_color = false;
        vec4 back_color = vec4(0.8f, 0.4f, 0.4f, 1.0f);
        bool light_two_sides = false;

        // material
        vec4 material_ambient = vec4(0.05f, 0.05f, 0.05f, 1.0f);
        vec4 material_specular = vec4(0.4f, 0.4f, 0.4f, 1.0f);
        float material_shininess = 64.0f;

        // effect
        int effect_ssao_algorithm = 0; // disabled
        float effect_ssao_radius = 2.0f;
        float effect_ssao_intensity = 1.5f;
        float effect_ssao_bias = 0.1f;
        float effect_ssao_sharpness = 40.0f;
        bool effect_edl_enabled = false;
        bool effect_transparency_enabled = false;
        bool effect_shadow_enabled = false;
        float shadow_light_distance = 50.0f;
        float shadow_softness = 0.5f;
        float shadow_darkness = 0.6f;

        // point cloud
        bool point_cloud_use_color_property = true;
        bool point_cloud_show_points = true;
        vec4 point_cloud_points_color = vec4(85 / 255.0f, 170 / 255.0f, 1.0f, 1.0f);
        float point_cloud_point_size = 2.0f;
        bool point_cloud_impostors = false;

        // surface mesh - surface
        bool surface_mesh_phong_shading = false;
        bool surface_mesh_show_faces = true;
        vec4 surface_mesh_faces_color = vec4(0.4f, 0.8f, 0.8f, 1.0f);
        bool surface_mesh_use_color_property = true;
        float surface_mesh_opacity = 0.6f;

        // surface mesh - vertices
        bool surface_mesh_show_vertices = false;
        vec4 surface_mesh_vertices_color = vec4(0.0f, 1.0f, 0.0f, 1.0f);
        bool surface_mesh_vertices_imposters = true;
        float surface_mesh_vertices_point_size = 5.0f;

        // surface mesh - edges
        bool surface_mesh_show_edges = false;
        vec4 surface_mesh_edges_color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        bool surface_mesh_edges_imposters = false;
        float surface_mesh_edges_line_width = 1.0f;

        // surface mesh - borders
        bool surface_mesh_show_borders = false;
        vec4 surface_mesh_borders_color = vec4(1.0f, 0.0f, 0.0f, 1.0f);
        bool surface_mesh_borders_imposters = true;
        float surface_mesh_borders_line_width = 2.0f;

        // graph - vertices
        bool graph_show_vertices = true;
        vec4 graph_vertices_color = vec4(0.8f, 0.0f, 0.5f, 1.0f);
        float   graph_vertices_point_size = 15;

        // surface mesh - edges
        bool graph_show_edges = true;
        vec4 graph_edges_color = vec4(1.0f, 0.67f, 0.5f, 1.0f);
        float   graph_edges_line_width = 3.0f;

        // clipping plane
        ClippingPlane* clipping_plane = nullptr;
        vec4 clipping_plane_color = vec4(1.0f, 0.0f, 0.0f, 0.2f);

    } // namespace setting

} // namespace scan3d

