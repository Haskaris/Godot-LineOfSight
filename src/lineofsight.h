#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H

#include <corecrt_math_defines.h>

#include <godot_cpp/classes/mesh_instance2d.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/physics_direct_space_state2d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters2d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/world2d.hpp>

#include <godot_cpp/classes/node2d.hpp>

namespace godot {

  class LineOfSight2D : public Node2D {
    GDCLASS(LineOfSight2D, Node2D)

  public:
    struct ViewCastInfo {
      bool hit;         // Whether the ray hit an obstacle.
      Vector2 origin;   // The origin of the ray.
      Vector2 point;    // The point at which the ray hit the obstacle.
      double distance;  // The distance from the origin to the point.
      double angle;     // The angle at which the ray was cast in degrees.

      ViewCastInfo() {
        hit = false;
        origin = Vector2();
        point = Vector2();
        distance = 0;
        angle = 0;
      }

      ViewCastInfo(
          bool p_hit, Vector2 p_origin, Vector2 p_point, double p_distance, double p_angle
      ) {
        hit = p_hit;
        origin = p_origin;
        point = p_point;
        distance = p_distance;
        angle = p_angle;
      }
    };

    struct EdgeInfo {
      Vector2 point_A;
      Vector2 point_B;

      EdgeInfo() {
        point_A = Vector2();
        point_B = Vector2();
      }

      EdgeInfo(Vector2 p_point_A, Vector2 p_point_B) {
        point_A = p_point_A;
        point_B = p_point_B;
      }
    };

  private:
    double resolution;               // The number of steps to take when casting rays.
    int edge_resolve_iterations;     // The number of iterations to take when resolving edges.
    double edge_distance_threshold;  // The distance threshold for resolving edges.
    double distance_from_origin;     // The distance from the origin of the start of the LOS.
    double angle;                    // The angle of the LOS.
    double radius;                   // The radius of the LOS (how far).

    double mesh_creation_time;  // The time it takes to create the mesh.
    Performance *performance;   // The performance monitor.

    MeshInstance2D *mesh;

  public:
    void set_resolution(const double p_resolution);
    double get_resolution() const;

    void set_edge_resolve_iterations(const int p_edge_resolve_iterations);
    int get_edge_resolve_iterations() const;

    void set_edge_distance_threshold(const double p_edge_distance_threshold);
    double get_edge_distance_threshold() const;

    void set_distance_from_origin(const double p_distance_from_origin);
    double get_distance_from_origin() const;

    void set_angle(const double p_angle);
    double get_angle() const;

    void set_radius(const double p_radius);
    double get_radius() const;

    void set_mesh_creation_time(const double p_mesh_creation_time);
    double get_mesh_creation_time() const;

  private:
    ViewCastInfo view_cast(const double p_angle);
    EdgeInfo find_edge(ViewCastInfo p_min_view_cast, ViewCastInfo p_max_view_cast);

  protected:
    static void _bind_methods();

  public:
    LineOfSight2D();
    ~LineOfSight2D();

    void _enter_tree() override;
    void _exit_tree() override;
    void _process(double delta) override;

    void draw_line_of_sight();
  };

}  // namespace godot

#endif
