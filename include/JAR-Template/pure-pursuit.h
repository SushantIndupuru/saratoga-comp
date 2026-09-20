#pragma once
#include <vector>

/**
 * 2D field point in inches. Matches JAR Template coordinates:
 * +Y is forward at heading 0, +X is to the right, heading increases clockwise.
 */

struct Point {
  float x;
  float y;
};

/**
 * Pure Pursuit geometry helper. Given a polyline path, it finds the
 * lookahead point on the path and the remaining distance to the end.
 * Drive::follow_path() uses this to steer; you can also call it yourself
 * if you want a custom controller.
 */

class PurePursuit
{
private:
  std::vector<Point> path;
  float last_fractional_index;

  float project_on_segment(float robot_x, float robot_y, int index);
  float distance_to_projection(float robot_x, float robot_y, int index, float t);
  float remaining_from_index(float fractional_index);
  Point point_along_path(float fractional_index, float dist);
  bool intersect_lookahead(float robot_x, float robot_y, float closest, float radius, Point &out);

public:
  PurePursuit();

  void set_path(const std::vector<Point> &path);
  void reset();

  int path_size();
  Point get_end_point();
  Point get_point_at(float fractional_index);

  float get_closest_fractional_index(float robot_x, float robot_y);
  Point get_lookahead_point(float robot_x, float robot_y, float lookahead);
  float get_remaining_distance(float robot_x, float robot_y);
  bool is_at_end(float robot_x, float robot_y, float settle_error);
  float get_curvature(float robot_x, float robot_y, float heading_deg, Point lookahead);

  static float distance(Point a, Point b);
  static std::vector<Point> inject(const std::vector<Point> &path, float spacing);
};
