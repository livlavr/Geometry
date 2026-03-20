#pragma once

#include <cstdint>
#include <cstddef>

#include "segment.hpp"

namespace linalg::geometry {
class Line : public IShape {
 private:
  static constexpr double kEps = 1e-9;
  static constexpr double kZero = 0;

 public:
  Line() = default;

  Line(const Point& point, const Point& other_point)
      : a_coef_(calc_a(point, other_point)),
        b_coef_(calc_b(point, other_point)),
        c_coef_(calc_c(point, other_point)) {}

  void move(const Vector& direction) override {
    c_coef_ -= (get_a() * direction.get_x())
               + (get_b() * direction.get_y());
  }

  bool contains_point(const Point& point) const override {
    return is_equal(substitute_point(point), kZero);
  }

  bool cross_segment(const Segment& segment) const override {
    const double a_point_substitution = substitute_point(segment.get_a());
    const double b_point_substitution = substitute_point(segment.get_b());

    return is_less_or_equal(a_point_substitution * b_point_substitution, kZero);
  }

  Line* clone() const override { return new Line(*this); }

  double get_a() const { return a_coef_; }
  double get_b() const { return b_coef_; }
  double get_c() const { return c_coef_; }

 private:
  double substitute_point(const Point& point) const {
    return (get_a() * point.get_x()) + (get_b() * point.get_y()) + get_c();
  }

  static bool is_equal(double expr, double other_expr) {
    return std::abs(expr - other_expr) < kEps;
  }

  static bool is_less_or_equal(double expr, double other_expr) {
    return (expr < other_expr) || is_equal(expr, other_expr);
  }
  static double calc_a(const Point& point, const Point& other_point) {
    return point.get_y() - other_point.get_y();
  }

  static double calc_b(const Point& point, const Point& other_point) {
    return other_point.get_x() - point.get_x();
  }

  static double calc_c(const Point& point, const Point& other_point) {
    return point.get_x() * other_point.get_y()
           - other_point.get_x() * point.get_y();
  }

 private:
  double a_coef_ = 0;
  double b_coef_ = 0;
  double c_coef_ = 0;
};

}  // namespace linalg::geometry
