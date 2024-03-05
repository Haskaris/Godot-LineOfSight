#include "lineofsight.h"
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void LineOfSight2D::_bind_methods() {
  // Bind getter and setter methods for private properties
  ClassDB::bind_method(D_METHOD("get_resolution"), &LineOfSight2D::get_resolution);
  ClassDB::bind_method(D_METHOD("set_resolution", "p_resolution"), &LineOfSight2D::set_resolution);
  ClassDB::add_property(
      "LineOfSight2D", PropertyInfo(Variant::FLOAT, "resolution", PROPERTY_HINT_RANGE, "0,100,0.1"),
      "set_resolution", "get_resolution"
  );

  ClassDB::bind_method(
      D_METHOD("get_edge_resolve_iterations"), &LineOfSight2D::get_edge_resolve_iterations
  );
  ClassDB::bind_method(
      D_METHOD("set_edge_resolve_iterations", "p_edge_resolve_iterations"),
      &LineOfSight2D::set_edge_resolve_iterations
  );
  ClassDB::add_property(
      "LineOfSight2D",
      PropertyInfo(Variant::INT, "edge_resolve_iterations", PROPERTY_HINT_RANGE, "1,100,1"),
      "set_edge_resolve_iterations", "get_edge_resolve_iterations"
  );

  ClassDB::bind_method(
      D_METHOD("get_edge_distance_threshold"), &LineOfSight2D::get_edge_distance_threshold
  );
  ClassDB::bind_method(
      D_METHOD("set_edge_distance_threshold", "p_edge_distance_threshold"),
      &LineOfSight2D::set_edge_distance_threshold
  );
  ClassDB::add_property(
      "LineOfSight2D",
      PropertyInfo(Variant::FLOAT, "edge_distance_threshold", PROPERTY_HINT_RANGE, "0,9999,0.1"),
      "set_edge_distance_threshold", "get_edge_distance_threshold"
  );

  ClassDB::bind_method(
      D_METHOD("get_distance_from_origin"), &LineOfSight2D::get_distance_from_origin
  );
  ClassDB::bind_method(
      D_METHOD("set_distance_from_origin", "p_distance_from_origin"),
      &LineOfSight2D::set_distance_from_origin
  );
  ClassDB::add_property(
      "LineOfSight2D",
      PropertyInfo(Variant::FLOAT, "distance_from_origin", PROPERTY_HINT_RANGE, "0,9999,0.1"),
      "set_distance_from_origin", "get_distance_from_origin"
  );

  ClassDB::bind_method(D_METHOD("get_angle"), &LineOfSight2D::get_angle);
  ClassDB::bind_method(D_METHOD("set_angle", "p_angle"), &LineOfSight2D::set_angle);
  ClassDB::add_property(
      "LineOfSight2D", PropertyInfo(Variant::FLOAT, "angle", PROPERTY_HINT_RANGE, "0,360,0.1"),
      "set_angle", "get_angle"
  );

  ClassDB::bind_method(D_METHOD("get_radius"), &LineOfSight2D::get_radius);
  ClassDB::bind_method(D_METHOD("set_radius", "p_radius"), &LineOfSight2D::set_radius);
  ClassDB::add_property(
      "LineOfSight2D", PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,9999,0.1"),
      "set_radius", "get_radius"
  );

  ClassDB::bind_method(D_METHOD("get_mesh_creation_time"), &LineOfSight2D::get_mesh_creation_time);
}

LineOfSight2D::LineOfSight2D() {
  resolution = 1;
  edge_resolve_iterations = 5;
  edge_distance_threshold = 0.5;
  distance_from_origin = 10;
  angle = 90;
  radius = 100;

  mesh_creation_time = 0;
  Callable mesh_creation_callable = Callable(this, StringName("get_mesh_creation_time"));
  performance = Performance::get_singleton();
  performance->add_custom_monitor(StringName("draw_time"), mesh_creation_callable);
}

LineOfSight2D::~LineOfSight2D() {
  // mesh->queue_free();
  // mesh = nullptr;
  performance = nullptr;
}

void LineOfSight2D::_enter_tree() {
  mesh = new MeshInstance2D();
  draw_line_of_sight();
  // Set the color of the circle to red.
  mesh->set_modulate(Color(1, 0, 0));
  // Add the MeshInstance2D as a child of this node.
  add_child(mesh);
}

void LineOfSight2D::_exit_tree() {
  // Remove the MeshInstance2D from this node.
  remove_child(mesh);
  mesh->queue_free();
}

void LineOfSight2D::_process(double delta) {
  // create timer to measure the time it takes to draw the line of sight.
  Time *time = Time::get_singleton();
  double start_time = time->get_unix_time_from_system();
  draw_line_of_sight();
  double end_time = time->get_unix_time_from_system();
  set_mesh_creation_time(end_time - start_time);
}

/// @brief Create a raycast from the center of the circle to the point at the given angle.
/// @param p_angle The angle at which to cast the ray in degrees.
/// @return A ViewCastInfo object containing the information about the raycast.
LineOfSight2D::ViewCastInfo LineOfSight2D::view_cast(const double p_angle) {
  double cos_angle = Math::cos(Math::deg_to_rad(p_angle));
  double sin_angle = Math::sin(Math::deg_to_rad(p_angle));
  Vector2 local_from =
      Vector2(get_distance_from_origin() * cos_angle, get_distance_from_origin() * sin_angle);
  Vector2 local_to = Vector2(get_radius() * cos_angle, get_radius() * sin_angle);
  Vector2 from = local_from + get_global_position();
  Vector2 to = local_to + get_global_position();
  Ref<PhysicsRayQueryParameters2D> parameters = PhysicsRayQueryParameters2D::create(from, to);

  Dictionary dict = get_world_2d()->get_direct_space_state()->intersect_ray(parameters);

  // If the raycast hit something, draw a line from the center of the circle to the point.
  if (dict.has("position")) {
    Vector2 position = dict["position"];
    return ViewCastInfo(true, from, position, position.distance_to(from), p_angle);
  } else {
    return ViewCastInfo(false, from, to, radius, p_angle);
  }
}

/// @brief Find the edge of the object that is between the two given view cast points.
/// @param p_min_view_cast The first view cast point.
/// @param p_max_view_cast The second view cast point.
/// @return An EdgeInfo object containing the information about the edge of the object.
LineOfSight2D::EdgeInfo
LineOfSight2D::find_edge(ViewCastInfo p_min_view_cast, ViewCastInfo p_max_view_cast) {
  double min_angle = p_min_view_cast.angle;
  double max_angle = p_max_view_cast.angle;
  Vector2 min_point = Vector2(0, 0);
  Vector2 max_point = Vector2(0, 0);

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

void LineOfSight2D::draw_line_of_sight() {
  int step_count = angle * resolution;
  double step_size = angle / step_count;

  List<Vector2> view_points_from = List<Vector2>();
  List<Vector2> view_points_to = List<Vector2>();

  ViewCastInfo old_view_cast_info = ViewCastInfo();

  double rotation_deg = get_global_rotation_degrees();

  for (int i = 0; i <= step_count; i++) {
    double current_angle = rotation_deg - (angle / 2.0) + (step_size * i);
    ViewCastInfo view_cast_info = view_cast(current_angle);

    if (i > 0) {
      bool edge_distance_threshold_exceeded =
          Math::abs(old_view_cast_info.distance - view_cast_info.distance) >
          edge_distance_threshold;

      bool diff_hit = old_view_cast_info.hit != view_cast_info.hit;
      bool both_hit = old_view_cast_info.hit && view_cast_info.hit;
      if (diff_hit || (both_hit && edge_distance_threshold_exceeded)) {
        EdgeInfo edge = find_edge(old_view_cast_info, view_cast_info);
        if (edge.point_A != Vector2(0, 0)) {
          view_points_to.push_back(to_local(edge.point_A));
        }
        if (edge.point_B != Vector2(0, 0)) {
          view_points_to.push_back(to_local(edge.point_B));
        }
      }
    }

    // Add the origin of the raycast to the list of view points
    view_points_from.push_back(view_cast_info.origin);

    // Add the point at which the raycast hit to the list of view points.
    Vector2 local_point = to_local(view_cast_info.point);
    view_points_to.push_back(local_point);

    old_view_cast_info = view_cast_info;
  }

  int vertex_count = view_points_to.size() + 1;
  Vector2 *vertices = new Vector2[vertex_count];
  int *triangles = new int[(vertex_count - 2) * 3];

  SurfaceTool *st = new SurfaceTool();
  st->begin(Mesh::PRIMITIVE_TRIANGLES);

  vertices[0] = Vector2(0, 0);
  st->add_vertex(Vector3(0, 0, 0));

  for (int i = 0; i < vertex_count - 1; i++) {
    vertices[i + 1] = view_points_to[i];
    st->add_vertex(Vector3(view_points_to[i].x, view_points_to[i].y, 0));

    if (i < vertex_count - 2) {
      triangles[i * 3] = 0;
      triangles[i * 3 + 1] = i + 1;
      triangles[i * 3 + 2] = i + 2;
      st->add_index(0);
      st->add_index(i + 1);
      st->add_index(i + 2);
    }
  }

  st->generate_normals();
  Ref<ArrayMesh> m = st->commit();
  mesh->set_mesh(m);
}

// Getter and setter methods for private properties
void LineOfSight2D::set_resolution(double value) { resolution = value; }

double LineOfSight2D::get_resolution() const { return resolution; }

void LineOfSight2D::set_edge_resolve_iterations(const int p_edge_resolve_iterations) {
  edge_resolve_iterations = p_edge_resolve_iterations;
}

int LineOfSight2D::get_edge_resolve_iterations() const { return edge_resolve_iterations; }

void LineOfSight2D::set_edge_distance_threshold(double value) { edge_distance_threshold = value; }

double LineOfSight2D::get_edge_distance_threshold() const { return edge_distance_threshold; }

void LineOfSight2D::set_distance_from_origin(double value) { distance_from_origin = value; }

double LineOfSight2D::get_distance_from_origin() const { return distance_from_origin; }

void LineOfSight2D::set_angle(double value) { angle = value; }

double LineOfSight2D::get_angle() const { return angle; }

void LineOfSight2D::set_radius(double value) { radius = value; }

double LineOfSight2D::get_radius() const { return radius; }

void LineOfSight2D::set_mesh_creation_time(double value) { mesh_creation_time = value; }

double LineOfSight2D::get_mesh_creation_time() const { return mesh_creation_time; }
