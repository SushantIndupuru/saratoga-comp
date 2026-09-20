#include "vex.h"

/**
 * Pure Pursuit constructor. last_fractional_index starts at the beginning
 * of the path so the first closest-point search does not skip waypoints.
 */

PurePursuit::PurePursuit() {
  last_fractional_index = 0;
}

/**
 * Loads a new polyline and resets progress along that path.
 *
 * @param path Waypoints in inches, field-centric.
 */

void PurePursuit::set_path(const std::vector<Point> &path) {
  this->path = path;
  last_fractional_index = 0;
}

/**
 * Resets progress to the start of the current path without changing waypoints.
 */

void PurePursuit::reset() {
  last_fractional_index = 0;
}

int PurePursuit::path_size() {
  return (int)path.size();
}

Point PurePursuit::get_end_point() {
  if (path.empty()) {
    return {0, 0};
  }
  return path.back();
}

/**
 * Euclidean distance between two points.
 *
 * @param a First point.
 * @param b Second point.
 * @return Distance in inches.
 */

float PurePursuit::distance(Point a, Point b) {
  return hypot(b.x - a.x, b.y - a.y);
}

/**
 * Interpolates a point along the polyline.
 * Integer part of fractional_index is the segment start waypoint,
 * fractional part is how far along that segment (0 to 1).
 *
 * @param fractional_index Position along the path.
 * @return Interpolated field point in inches.
 */

Point PurePursuit::get_point_at(float fractional_index) {
  if (path.empty()) {
    return {0, 0};
  }
  if (fractional_index <= 0) {
    return path[0];
  }
  if (fractional_index >= path.size() - 1) {
    return path.back();
  }
  int i = (int)fractional_index;
  float t = fractional_index - i;
  return {
    path[i].x + (path[i + 1].x - path[i].x) * t,
    path[i].y + (path[i + 1].y - path[i].y) * t
  };
}

/**
 * Projects the robot onto a path segment and returns t in unbounded
 * line-parameter space. t in [0, 1] is on the segment.
 *
 * @param robot_x Robot x in inches.
 * @param robot_y Robot y in inches.
 * @param index Segment start waypoint index.
 * @return Unclamped projection parameter t.
 */

float PurePursuit::project_on_segment(float robot_x, float robot_y, int index) {
  float dx = path[index + 1].x - path[index].x;
  float dy = path[index + 1].y - path[index].y;
  float length_sq = dx * dx + dy * dy;
  if (length_sq < 0.0001) {
    return 0;
  }
  return ((robot_x - path[index].x) * dx + (robot_y - path[index].y) * dy) / length_sq;
}

/**
 * Distance from the robot to a clamped projection on a segment.
 *
 * @param robot_x Robot x in inches.
 * @param robot_y Robot y in inches.
 * @param index Segment start waypoint index.
 * @param t Unclamped projection parameter.
 * @return Distance in inches.
 */

float PurePursuit::distance_to_projection(float robot_x, float robot_y, int index, float t) {
  float clamped_t = clamp(t, 0, 1);
  Point projected = {
    path[index].x + (path[index + 1].x - path[index].x) * clamped_t,
    path[index].y + (path[index + 1].y - path[index].y) * clamped_t
  };
  return hypot(robot_x - projected.x, robot_y - projected.y);
}

/**
 * Finds the closest point on the remaining path. Progress only moves
 * forward, so a later part of a looping path cannot steal the target.
 *
 * @param robot_x Robot x in inches.
 * @param robot_y Robot y in inches.
 * @return Fractional index of the closest remaining point.
 */

float PurePursuit::get_closest_fractional_index(float robot_x, float robot_y) {
  if (path.size() < 2) {
    return 0;
  }

  int last_segment = (int)path.size() - 2;
  int i = (int)last_fractional_index;
  if (i < 0) {
    i = 0;
  }
  if (i > last_segment) {
    i = last_segment;
  }

  while (i < last_segment) {
    float t = project_on_segment(robot_x, robot_y, i);
    if (t > 1) {
      i++;
      continue;
    }
    if (t < 0) {
      t = 0;
    }

    float next_t = project_on_segment(robot_x, robot_y, i + 1);
    float current_dist = distance_to_projection(robot_x, robot_y, i, t);
    float next_dist = distance_to_projection(robot_x, robot_y, i + 1, next_t);
    if (next_t >= 0 && next_dist < current_dist) {
      i++;
      continue;
    }

    last_fractional_index = i + t;
    return last_fractional_index;
  }

  float t = clamp(project_on_segment(robot_x, robot_y, last_segment), 0, 1);
  last_fractional_index = last_segment + t;
  return last_fractional_index;
}

float PurePursuit::remaining_from_index(float fractional_index) {
  if (path.size() < 2 || fractional_index >= path.size() - 1) {
    return 0;
  }
  int i = (int)fractional_index;
  float remaining = distance(get_point_at(fractional_index), path[i + 1]);
  for (int j = i + 1; j < (int)path.size() - 1; j++) {
    remaining += distance(path[j], path[j + 1]);
  }
  return remaining;
}

Point PurePursuit::point_along_path(float fractional_index, float dist) {
  if (path.size() < 2 || dist <= 0) {
    return get_point_at(fractional_index);
  }
  int i = (int)fractional_index;
  float t = fractional_index - i;
  float dist_left = dist;
  while (i < (int)path.size() - 1 && dist_left > 0) {
    float segment_length = distance(path[i], path[i + 1]);
    float remaining_on_segment = segment_length * (1 - t);
    if (segment_length < 0.0001 || dist_left <= remaining_on_segment) {
      float step = (segment_length < 0.0001) ? 0 : dist_left / segment_length;
      return get_point_at(i + t + step);
    }
    dist_left -= remaining_on_segment;
    i++;
    t = 0;
  }
  return path.back();
}

/**
 * Circle-line intersections with the remaining path. Writes the furthest
 * valid intersection to out and returns true if one exists.
 */

bool PurePursuit::intersect_lookahead(float robot_x, float robot_y, float closest, float radius, Point &out) {
  if (radius < 0.0001 || path.size() < 2) {
    return false;
  }

  float lookahead_index = closest;
  bool found = false;
  int start = (int)closest;
  int last_segment = (int)path.size() - 2;
  for (int i = start; i <= last_segment; i++) {
    float dx = path[i + 1].x - path[i].x;
    float dy = path[i + 1].y - path[i].y;
    float fx = path[i].x - robot_x;
    float fy = path[i].y - robot_y;

    float a = dx * dx + dy * dy;
    if (a < 0.0001) {
      continue;
    }
    float b = 2 * (fx * dx + fy * dy);
    float c = fx * fx + fy * fy - radius * radius;
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
      continue;
    }

    float sqrt_disc = sqrt(discriminant);
    float t1 = (-b - sqrt_disc) / (2 * a);
    float t2 = (-b + sqrt_disc) / (2 * a);
    float min_t = (i == start) ? (closest - i) : 0;

    if (t1 >= min_t && t1 <= 1 && i + t1 >= lookahead_index) {
      lookahead_index = i + t1;
      found = true;
    }
    if (t2 >= min_t && t2 <= 1 && i + t2 >= lookahead_index) {
      lookahead_index = i + t2;
      found = true;
    }
  }

  if (!found) {
    return false;
  }
  out = get_point_at(lookahead_index);
  return true;
}

/**
 * Finds the lookahead point as the furthest circle-line intersection
 * along the path.
 *
 * No intersection (off the path): steer to the closest point, or to a
 * carrot walked along the path if still far from the end.
 * End of path: always the last waypoint. Lookahead shrinks to remaining
 * distance so the circle still hits the final segment.
 *
 * @param robot_x Robot x in inches.
 * @param robot_y Robot y in inches.
 * @param lookahead Lookahead radius in inches.
 * @return Lookahead point in inches.
 */

Point PurePursuit::get_lookahead_point(float robot_x, float robot_y, float lookahead) {
  if (path.empty()) {
    return {robot_x, robot_y};
  }
  if (path.size() == 1) {
    return path[0];
  }
  if (lookahead < 0.0001) {
    lookahead = 0.0001;
  }

  float closest = get_closest_fractional_index(robot_x, robot_y);
  float remaining = remaining_from_index(closest);
  Point closest_point = get_point_at(closest);
  float crosstrack = distance({robot_x, robot_y}, closest_point);

  if (remaining < 0.001) {
    return path.back();
  }

  float radius = lookahead;
  if (remaining < radius) {
    radius = remaining;
  }
  if (radius < 0.25) {
    radius = 0.25;
  }

  Point intersection;
  if (intersect_lookahead(robot_x, robot_y, closest, radius, intersection)) {
    return intersection;
  }

  if (remaining <= lookahead) {
    return path.back();
  }

  if (crosstrack > lookahead) {
    return closest_point;
  }

  return point_along_path(closest, lookahead);
}

/**
 * Remaining arc length along the path from the closest point to the end.
 *
 * @param robot_x Robot x in inches.
 * @param robot_y Robot y in inches.
 * @return Remaining path distance in inches.
 */

float PurePursuit::get_remaining_distance(float robot_x, float robot_y) {
  if (path.size() < 2) {
    if (path.empty()) {
      return 0;
    }
    return distance({robot_x, robot_y}, path[0]);
  }
  return remaining_from_index(get_closest_fractional_index(robot_x, robot_y));
}

bool PurePursuit::is_at_end(float robot_x, float robot_y, float settle_error) {
  if (path.empty()) {
    return true;
  }
  float remaining = get_remaining_distance(robot_x, robot_y);
  float end_error = distance({robot_x, robot_y}, path.back());
  return remaining <= settle_error && end_error <= settle_error;
}

/**
 * Signed curvature of the arc from the robot pose through the lookahead
 * point. Positive curvature is a clockwise (right) turn, matching JAR
 * Template heading.
 *
 * @param robot_x Robot x in inches.
 * @param robot_y Robot y in inches.
 * @param heading_deg Robot heading in degrees.
 * @param lookahead Lookahead point.
 * @return Curvature in 1/inches.
 */

float PurePursuit::get_curvature(float robot_x, float robot_y, float heading_deg, Point lookahead) {
  float dx = lookahead.x - robot_x;
  float dy = lookahead.y - robot_y;
  float L_sq = dx * dx + dy * dy;
  if (L_sq < 0.0001) {
    return 0;
  }
  float heading = to_rad(heading_deg);
  float robot_right = dx * cos(heading) - dy * sin(heading);
  return 2 * robot_right / L_sq;
}

/**
 * Injects evenly spaced points along each segment so a sparse waypoint
 * list can be followed more smoothly. The original vertices are kept.
 *
 * @param path Input waypoints in inches.
 * @param spacing Desired spacing in inches.
 * @return Densified waypoint list.
 */

std::vector<Point> PurePursuit::inject(const std::vector<Point> &path, float spacing) {
  std::vector<Point> result;
  if (path.empty()) {
    return result;
  }
  if (spacing <= 0) {
    return path;
  }

  for (int i = 0; i < (int)path.size() - 1; i++) {
    float segment_length = distance(path[i], path[i + 1]);
    int points = (int)(segment_length / spacing);
    if (points < 1) {
      points = 1;
    }
    for (int j = 0; j < points; j++) {
      float t = (float)j / points;
      result.push_back({
        path[i].x + (path[i + 1].x - path[i].x) * t,
        path[i].y + (path[i + 1].y - path[i].y) * t
      });
    }
  }
  result.push_back(path.back());
  return result;
}
