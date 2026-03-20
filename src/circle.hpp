#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "segment.hpp"

namespace linalg::geometry {

class Circle : public IShape {
 public:
  Circle(const Point& center, std::size_t radius)
      : center_(center), radius_(radius) {}

  void move(const Vector& direction) override { get_centre().move(direction); }

  bool contains_point(const Point& point) const override {
    const Vector to_point = point - get_centre();
    return to_point.get_squared_norm() <= get_squared_radius();
  }

  bool cross_segment(const Segment& segment) const override {
    const std::int64_t radius_sq = get_squared_radius();
    const std::int64_t dist_a_sq =
        (segment.get_a() - get_centre()).get_squared_norm();
    const std::int64_t dist_b_sq =
        (segment.get_b() - get_centre()).get_squared_norm();

    if (dist_a_sq < radius_sq && dist_b_sq < radius_sq) {
      return false;
    }

    const Vector segment_vector = segment.get_vector();
    const std::int64_t segment_len_sq = segment.get_squared_distance();

    if (segment_len_sq == 0) {
      return false;
    }

    const Vector center_to_a = center_ - segment.get_a();
    const std::int64_t projection = center_to_a * segment_vector;

    if (projection < 0 || projection > segment_len_sq) {
      return false;
    }

    const std::int64_t cross_product = center_to_a ^ segment_vector;

    return (cross_product * cross_product)
           <= (get_squared_radius() * segment_len_sq);
  }

  Circle* clone() const override { return new Circle(*this); }

  Point& get_centre() { return center_; }
  const Point& get_centre() const { return center_; }

  std::size_t& get_radius() { return radius_; }
  std::size_t get_radius() const { return radius_; }

 private:
  std::int64_t get_squared_radius() const {
    return get_radius() * get_radius();
  }

 private:
  Point center_;
  std::size_t radius_ = 0;
};

}  // namespace linalg::geometry
