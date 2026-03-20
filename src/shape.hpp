#pragma once

#include "fwd.hpp"

namespace linalg::geometry {

class IShape {
 public:
  virtual void move(const Vector& direction) = 0;
  virtual bool contains_point(const Point& point) const = 0;
  virtual bool cross_segment(const Segment& segment) const = 0;
  virtual IShape* clone() const = 0;
  virtual ~IShape() = default;
};

}
