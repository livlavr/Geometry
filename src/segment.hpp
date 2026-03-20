#pragma once

#include <cstdint>
#include <cstddef>

#include "point.hpp"

namespace linalg::geometry {

class Segment : public IShape {
 public:
  friend class Circle;

 public:
  Segment() = default;

  Segment(const Point& a_point, const Point& b_point)
      : a_coef_(a_point), b_coef_(b_point) {}

  void move(const Vector& direction) override {
    a_coef_.move(direction);
    b_coef_.move(direction);
  }

  bool contains_point(const Point& point) const override {
    const Point a_point = get_a();
    const Point b_point = get_b();

    if (point == a_point || point == b_point) {
      return true;
    }

    const Vector ab = b_point - a_point;
    const Vector ap = point - a_point;

    if (!ab.is_collinear_to(ap)) {
      return false;
    }

    return point.is_between(a_point, b_point);
  }

  bool cross_segment(const Segment& segment) const override {
    const Point this_a = get_a();
    const Point this_b = get_b();
    const Point other_a = segment.get_a();
    const Point other_b = segment.get_b();

    const Orientation ori1 = orientation(this_a, this_b, other_a);
    const Orientation ori2 = orientation(this_a, this_b, other_b);
    const Orientation ori3 = orientation(other_a, other_b, this_a);
    const Orientation ori4 = orientation(other_a, other_b, this_b);

    if (ori1 != ori2 && ori3 != ori4) {
      return true;
    }

    if (ori1 == Orientation::Collinear && other_a.is_between(this_a, this_b)) {
      return true;
    }
    if (ori2 == Orientation::Collinear && other_b.is_between(this_a, this_b)) {
      return true;
    }
    if (ori3 == Orientation::Collinear && this_a.is_between(other_a, other_b)) {
      return true;
    }

    return (ori4 == Orientation::Collinear)
           && this_b.is_between(other_a, other_b);
  }

  Segment* clone() const override { return new Segment(*this); }

  Point get_a() const { return a_coef_; }
  Point get_b() const { return b_coef_; }

 private:
  enum class Orientation : int { Collinear = 0, Positive = 1, Negative = 2 };

  static Orientation orientation(const Point& seg_start, const Point& seg_end,
                                 const Point& test_point) {
    const Vector ab_vector = seg_end - seg_start;
    const Vector ac_vector = test_point - seg_start;

    const std::int64_t value = (ab_vector ^ ac_vector);
    if (value == 0) {
      return Orientation::Collinear;
    }
    return (value > 0) ? Orientation::Positive : Orientation::Negative;
  }

  Vector direction() const { return get_b() - get_a(); }

  std::int64_t get_squared_distance() const {
    return direction().get_squared_norm();
  }

  Vector get_vector() const { return get_b() - get_a(); }

 private:
  Point a_coef_ = {};
  Point b_coef_ = {};
};

inline bool Point::point_cross_segment(const Segment& segment) const {
  return segment.contains_point(*this);
}

}  // namespace linalg::geometry
