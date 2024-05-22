#include "lineofsight3d.h"

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void LineOfSight3D::_bind_methods() {
  // Bind getter and setter methods for private properties
  ClassDB::bind_method(D_METHOD("get_resolution"), &LineOfSight3D::get_resolution);
  ClassDB::bind_method(D_METHOD("set_resolution", "p_resolution"), &LineOfSight3D::set_resolution);
  ClassDB::add_property(
      "LineOfSight3D", PropertyInfo(Variant::FLOAT, "resolution", PROPERTY_HINT_RANGE, "0,100,0.1"),
      "set_resolution", "get_resolution"
  );

  ClassDB::bind_method(
      D_METHOD("get_edge_resolve_iterations"), &LineOfSight3D::get_edge_resolve_iterations
  );
  ClassDB::bind_method(
      D_METHOD("set_edge_resolve_iterations", "p_edge_resolve_iterations"),
      &LineOfSight3D::set_edge_resolve_iterations
  );
  ClassDB::add_property(
      "LineOfSight3D",
      PropertyInfo(Variant::INT, "edge_resolve_iterations", PROPERTY_HINT_RANGE, "1,100,1"),
      "set_edge_resolve_iterations", "get_edge_resolve_iterations"
  );

  ClassDB::bind_method(
      D_METHOD("get_edge_distance_threshold"), &LineOfSight3D::get_edge_distance_threshold
  );
  ClassDB::bind_method(
      D_METHOD("set_edge_distance_threshold", "p_edge_distance_threshold"),
      &LineOfSight3D::set_edge_distance_threshold
  );
  ClassDB::add_property(
      "LineOfSight3D",
      PropertyInfo(Variant::FLOAT, "edge_distance_threshold", PROPERTY_HINT_RANGE, "0,9999,0.1"),
      "set_edge_distance_threshold", "get_edge_distance_threshold"
  );

  ClassDB::bind_method(
      D_METHOD("get_distance_from_origin"), &LineOfSight3D::get_distance_from_origin
  );
  ClassDB::bind_method(
      D_METHOD("set_distance_from_origin", "p_distance_from_origin"),
      &LineOfSight3D::set_distance_from_origin
  );
  ClassDB::add_property(
      "LineOfSight3D",
      PropertyInfo(Variant::FLOAT, "distance_from_origin", PROPERTY_HINT_RANGE, "0,9999,0.1"),
      "set_distance_from_origin", "get_distance_from_origin"
  );

  ClassDB::bind_method(D_METHOD("get_angle"), &LineOfSight3D::get_angle);
  ClassDB::bind_method(D_METHOD("set_angle", "p_angle"), &LineOfSight3D::set_angle);
  ClassDB::add_property(
      "LineOfSight3D", PropertyInfo(Variant::FLOAT, "angle", PROPERTY_HINT_RANGE, "0,360,0.1"),
      "set_angle", "get_angle"
  );

  ClassDB::bind_method(D_METHOD("get_radius"), &LineOfSight3D::get_radius);
  ClassDB::bind_method(D_METHOD("set_radius", "p_radius"), &LineOfSight3D::set_radius);
  ClassDB::add_property(
      "LineOfSight3D", PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,9999,0.1"),
      "set_radius", "get_radius"
  );

  ClassDB::bind_method(D_METHOD("get_mesh_creation_time"), &LineOfSight3D::get_mesh_creation_time);
}

LineOfSight3D::LineOfSight3D() {
  resolution = 1;
  edge_resolve_iterations = 0.5;
  edge_distance_threshold = 0.1;
  distance_from_origin = 1;
  angle = 90;
  radius = 10;

  mesh_creation_time = 0;
  Callable mesh_creation_callable = Callable(this, StringName("get_mesh_creation_time"));
  performance = Performance::get_singleton();
  if (!performance->has_custom_monitor(StringName("draw_time"))) {
    performance->add_custom_monitor(StringName("draw_time"), mesh_creation_callable);
  }
}

LineOfSight3D::~LineOfSight3D() {
  // mesh->queue_free();
  // mesh = nullptr;
  performance = nullptr;
}

void LineOfSight3D::_enter_tree() {
  mesh = new MeshInstance3D();
  draw_line_of_sight();
  add_child(mesh);

  // Required to make the mesh render correctly (don't overturn the mesh when the parent rotates)
  // mesh->set_as_top_level(true);
}

void LineOfSight3D::_exit_tree() {
  remove_child(mesh);
  mesh->queue_free();
}

void LineOfSight3D::_process(double delta) {
  Time *time = Time::get_singleton();
  double start_time = time->get_unix_time_from_system();
  draw_line_of_sight();
  double end_time = time->get_unix_time_from_system();
  set_mesh_creation_time(end_time - start_time);

  // Because the mesh is detached from the parent, we need to update its position manually.
  mesh->set_global_position(get_global_position());
}

/// @brief Create a raycast from the center of the circle to the point at the given angle.
/// @param p_angle The angle at which to cast the ray in degrees.
/// @return A ViewCastInfo object containing the information about the raycast.
LineOfSight3D::ViewCastInfo LineOfSight3D::view_cast(const double p_angle) {
  double cos_angle = Math::cos(Math::deg_to_rad(p_angle));
  double sin_angle = Math::sin(Math::deg_to_rad(p_angle));
  Vector3 local_from =
      Vector3(get_distance_from_origin() * cos_angle, 0, get_distance_from_origin() * sin_angle);
  Vector3 local_to = Vector3(get_radius() * cos_angle, 0, get_radius() * sin_angle);
  Vector3 from = local_from + get_global_position();
  Vector3 to = local_to + get_global_position();
  Ref<PhysicsRayQueryParameters3D> parameters = PhysicsRayQueryParameters3D::create(from, to);

  Dictionary dict = get_world_3d()->get_direct_space_state()->intersect_ray(parameters);

  // If the raycast hit something, draw a line from the center of the circle to the point.
  if (dict.has("position")) {
    Vector3 position = dict["position"];
    return ViewCastInfo(true, from, position, position.distance_to(from), p_angle);
  } else {
    return ViewCastInfo(false, from, to, radius, p_angle);
  }
}

/// @brief Find the edge of the object that is between the two given view cast points.
/// @param p_min_view_cast The first view cast point.
/// @param p_max_view_cast The second view cast point.
/// @return An EdgeInfo object containing the information about the edge of the object.
LineOfSight3D::EdgeInfo
LineOfSight3D::find_edge(ViewCastInfo p_min_view_cast, ViewCastInfo p_max_view_cast) {
  double min_angle = p_min_view_cast.angle;
  double max_angle = p_max_view_cast.angle;
  Vector3 min_point = Vector3(0, 0, 0);
  Vector3 max_point = Vector3(0, 0, 0);

  for (int i = 0; i < edge_resolve_iterations; i++) {
    double angle = (min_angle + max_angle) / 2.0;
    ViewCastInfo new_view_cast = view_cast(angle);

    bool edge_distance_threshold_exceeded =
        Math::abs(p_min_view_cast.distance - new_view_cast.distance) > edge_distance_threshold;

    if (new_view_cast.hit == p_min_view_cast.hit && !edge_distance_threshold_exceeded) {
      min_angle = angle;
      min_point = new_view_cast.point;
    } else {
      max_angle = angle;
      max_point = new_view_cast.point;
    }
  }

  return EdgeInfo(min_point, max_point);
}

/// @brief Draw the line of sight by casting rays and drawing lines between the points.
void LineOfSight3D::draw_line_of_sight() {
  List<Vector3> view_points_from = List<Vector3>();
  List<Vector3> view_points_to = List<Vector3>();
  ViewCastInfo old_view_cast_info = ViewCastInfo();

  int step_count = angle * resolution;
  double step_size = angle / step_count;
  double rotation_deg = get_global_rotation_degrees().z;

  for (int i = 0; i <= step_count; i++) {
    double current_angle = rotation_deg - (angle / 2.0) + (step_size * i);
    ViewCastInfo view_cast_info = view_cast(current_angle);

    // If we already have a previous view cast, check if the current view cast is different.
    if (i > 0) {
      bool edge_distance_threshold_exceeded =
          Math::abs(old_view_cast_info.distance - view_cast_info.distance) >
          edge_distance_threshold;

      bool diff_hit = old_view_cast_info.hit != view_cast_info.hit;
      bool both_hit = old_view_cast_info.hit && view_cast_info.hit;
      if (diff_hit || (both_hit && edge_distance_threshold_exceeded)) {
        EdgeInfo edge = find_edge(old_view_cast_info, view_cast_info);
        if (edge.point_A != Vector3(0, 0, 0)) {
          view_points_from.push_back(view_cast_info.origin - get_global_position());
          view_points_to.push_back(edge.point_A - get_global_position());
        }
        if (edge.point_B != Vector3(0, 0, 0)) {
          view_points_from.push_back(view_cast_info.origin - get_global_position());
          view_points_to.push_back(edge.point_B - get_global_position());
        }
      }
    }

    view_points_from.push_back(view_cast_info.origin - get_global_position());
    view_points_to.push_back(view_cast_info.point - get_global_position());

    old_view_cast_info = view_cast_info;
  }

  int vertex_count = view_points_to.size() + 1;
  SurfaceTool *st = new SurfaceTool();
  st->begin(Mesh::PRIMITIVE_TRIANGLE_STRIP);

  for (int i = 0; i < vertex_count - 1; i++) {
    // Add two vertices to the SurfaceTool for each view point.
    st->add_vertex(view_points_from[i]);
    st->add_vertex(view_points_to[i]);
  }

  Ref<ArrayMesh> m = st->commit();
  mesh->set_mesh(m);
}

void LineOfSight3D::set_resolution(double value) { resolution = value; }

double LineOfSight3D::get_resolution() const { return resolution; }

void LineOfSight3D::set_edge_resolve_iterations(const int p_edge_resolve_iterations) {
  edge_resolve_iterations = p_edge_resolve_iterations;
}

int LineOfSight3D::get_edge_resolve_iterations() const { return edge_resolve_iterations; }

void LineOfSight3D::set_edge_distance_threshold(double value) { edge_distance_threshold = value; }

double LineOfSight3D::get_edge_distance_threshold() const { return edge_distance_threshold; }

void LineOfSight3D::set_distance_from_origin(double value) { distance_from_origin = value; }

double LineOfSight3D::get_distance_from_origin() const { return distance_from_origin; }

void LineOfSight3D::set_angle(double value) { angle = value; }

double LineOfSight3D::get_angle() const { return angle; }

void LineOfSight3D::set_radius(double value) { radius = value; }

double LineOfSight3D::get_radius() const { return radius; }

void LineOfSight3D::set_mesh_creation_time(double value) { mesh_creation_time = value; }

double LineOfSight3D::get_mesh_creation_time() const { return mesh_creation_time; }
