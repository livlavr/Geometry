#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "line.hpp"

namespace linalg::geometry {

class Ray : public IShape {
 public:
  Ray() = default;

  Ray(const Point& point, const Point& other_point)
      : point_(point), direction_(other_point - point) {}

  void move(const Vector& direction) override { point_.move(direction); }

  bool contains_point(const Point& point) const override {
    const Vector to_point = point - point_;
    const Vector& direction = direction_;

    if ((direction ^ to_point) != 0) {
      return false;
    }
    return (direction * to_point) >= 0;
  }

  bool cross_segment(const Segment& segment) const override {
    const Line line = get_line_from_ray();
    if (!line.cross_segment(segment)) {
      return false;
    }
    return contains_point(segment.get_a()) || contains_point(segment.get_b());
  }

  Ray* clone() const override { return new Ray(*this); }

  Point get_a() const { return point_; }
  Vector get_vector() const { return direction_; }

 private:
  Line get_line_from_ray() const {
    return Line(point_, Point(point_.get_x() + direction_.get_x(),
                              point_.get_y() + direction_.get_y()));
  }

 private:
  Point point_;
  Vector direction_;
};

}
