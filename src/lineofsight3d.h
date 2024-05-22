#ifndef LINEOFSIGHT_3D_H
#define LINEOFSIGHT_3D_H

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/world3d.hpp>

#include <godot_cpp/classes/node3d.hpp>

using namespace godot;

class LineOfSight3D : public Node3D {
  GDCLASS(LineOfSight3D, Node3D)

public:
  struct ViewCastInfo {
    bool hit;         // Whether the ray hit an obstacle.
    Vector3 origin;   // The origin of the ray.
    Vector3 point;    // The point at which the ray hit the obstacle.
    double distance;  // The distance from the origin to the point.
    double angle;     // The angle at which the ray was cast in degrees.

    ViewCastInfo() {
      hit = false;
      origin = Vector3();
      point = Vector3();
      distance = 0;
      angle = 0;
    }

    ViewCastInfo(bool p_hit, Vector3 p_origin, Vector3 p_point, double p_distance, double p_angle) {
      hit = p_hit;
      origin = p_origin;
      point = p_point;
      distance = p_distance;
      angle = p_angle;
    }
  };

  struct EdgeInfo {
    Vector3 point_A;
    Vector3 point_B;

    EdgeInfo() {
      point_A = Vector3();
      point_B = Vector3();
    }

    EdgeInfo(Vector3 p_point_A, Vector3 p_point_B) {
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

  MeshInstance3D *mesh;

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
  LineOfSight3D();
  ~LineOfSight3D();

  void _enter_tree() override;
  void _exit_tree() override;
  void _process(double delta) override;

  void draw_line_of_sight();
};

#endif
