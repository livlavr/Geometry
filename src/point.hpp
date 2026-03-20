#pragma once

#include <algorithm>

#include "vector.hpp"
#include "shape.hpp"

namespace linalg::geometry {

class Point : public IShape {
 public:
  friend class Segment;

 public:
  Point() = default;
  Point(std::int64_t x_component, std::int64_t y_component)
      : radius_vector_(x_component, y_component) {}

 public:
  bool operator==(const Point& point) const {
    return (get_x() == point.get_x()) && (get_y() == point.get_y());
  }

  void move(const Vector& direction) override { radius_vector_ += direction; }

  bool contains_point(const Point& point) const override {
    return *this == point;
  }

  bool cross_segment(const Segment& segment) const override {
    return point_cross_segment(segment);
  }

  Point* clone() const override { return new Point(*this); }

  std::int64_t get_x() const { return radius_vector_.get_x(); }
  std::int64_t get_y() const { return radius_vector_.get_y(); }

 private:
  bool point_cross_segment(const Segment& segment) const;

  bool is_between(const Point& lhs, const Point& rhs) const {
    return is_x_between(lhs, rhs) && is_y_between(lhs, rhs);
  }

  bool is_x_between(const Point& lhs, const Point& rhs) const {
    const std::int64_t x = get_x();
    const std::int64_t min_x = std::min(lhs.get_x(), rhs.get_x());
    const std::int64_t max_x = std::max(lhs.get_x(), rhs.get_x());
    return (min_x <= x) && (x <= max_x);
  }

  bool is_y_between(const Point& lhs, const Point& rhs) const {
    const std::int64_t y = get_y();
    const std::int64_t min_y = std::min(lhs.get_y(), rhs.get_y());
    const std::int64_t max_y = std::max(lhs.get_y(), rhs.get_y());
    return (min_y <= y) && (y <= max_y);
  }

  RadiusVector radius_vector_{0, 0};
};

inline Vector operator-(const Point& lhs, const Point& rhs) {
  return Vector(lhs.get_x() - rhs.get_x(), lhs.get_y() - rhs.get_y());
}

}
