#pragma once

#include <cstddef>
#include <cstdint>

#include "fwd.hpp"

namespace linalg::geometry {

class Vector {
  friend class Segment;
  friend class Circle;

 public:
  Vector() = default;
  Vector(std::int64_t x_component, std::int64_t y_component)
      : x_component_(x_component), y_component_(y_component) {}

 public:
  friend std::int64_t operator*(const Vector& lhs, const Vector& rhs) {
    return lhs.scalar_multiply(rhs);
  }

  friend std::int64_t operator^(const Vector& lhs, const Vector& rhs) {
    return lhs.vector_multiply(rhs);
  }

  Vector& operator+=(const Vector& other) {
    x_component_ += other.get_x();
    y_component_ += other.get_y();
    return *this;
  }

  Vector& operator-=(const Vector& other) {
    x_component_ -= other.get_x();
    y_component_ -= other.get_y();
    return *this;
  }

  Vector& operator*=(std::int64_t scalar) {
    x_component_ *= scalar;
    y_component_ *= scalar;
    return *this;
  }

  Vector operator-() const { return Vector(-x_component_, -y_component_); }

  std::int64_t get_x() const { return x_component_; }
  std::int64_t get_y() const { return y_component_; }

 private:
  std::int64_t scalar_multiply(const Vector& other) const {
    return (get_x() * other.get_x()) + (get_y() * other.get_y());
  }

  std::int64_t vector_multiply(const Vector& other) const {
    return (get_x() * other.get_y()) - (get_y() * other.get_x());
  }

  bool is_collinear_to(const Vector& other) const {
    return ((*this ^ other) == 0);
  }

  std::int64_t get_squared_norm() const { return (*this) * (*this); }

 private:
  std::int64_t x_component_ = 0;
  std::int64_t y_component_ = 0;
};

inline Vector operator*(const Vector& other, std::int64_t scalar) {
  Vector copy(other);
  copy *= scalar;
  return copy;
}

inline Vector operator*(std::int64_t scalar, const Vector& other) {
  return other * scalar;
}

inline Vector operator+(Vector lhs, const Vector& rhs) {
  lhs += rhs;
  return lhs;
}

inline Vector operator-(Vector lhs, const Vector& rhs) {
  lhs -= rhs;
  return lhs;
}

class RadiusVector : public Vector {
 public:
  using Vector::Vector;
  RadiusVector() = default;
};

}
