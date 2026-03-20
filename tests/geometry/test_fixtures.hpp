#include <src/geometry.hpp>
#include <gtest/gtest.h>
#pragma once

class GeometryTest : public ::testing::Test {
 public:
  ~GeometryTest() override {
    delete shape_;
    delete moved_shape_;
    delete p1_;
    delete p2_;
  }
 protected:
  void TestContainsPoint(bool expected) {
    ASSERT_EQ(shape_->contains_point(*p1_), expected);
  }

  void TestCrossesSegment(bool expected) {
    auto segment = linalg::geometry::Segment(*p1_, *p2_);
    ASSERT_EQ(shape_->cross_segment(segment), expected);
  }

  void SetupMoveTest() {
    auto vector = *p2_ - *p1_;
    moved_shape_ = shape_->clone();
    moved_shape_->move(vector);
  }

  virtual void TestMove() = 0;

  static bool ArePointsEqual(const linalg::geometry::Point& p1, const linalg::geometry::Point& p2) {
    return p1.get_x() == p2.get_x() && p1.get_y() == p2.get_y();
  }

  static void AreVectorsSimilar(const linalg::geometry::Vector& v1, const linalg::geometry::Vector& v2) {
    if (v1.get_x() == 0 || v2.get_x() == 0) {
      ASSERT_DOUBLE_EQ(v1.get_x(), v2.get_x());
      return;
    }

    if (v1.get_y() == 0 || v2.get_y() == 0) {
      ASSERT_DOUBLE_EQ(v1.get_y(), v2.get_y());
      return;
    }

    ASSERT_DOUBLE_EQ(static_cast<double>(v1.get_x()) / static_cast<double>(v2.get_x()),
                     static_cast<double>(v1.get_y()) / static_cast<double>(v2.get_y()));

  }

  linalg::geometry::IShape* shape_ = nullptr;
  linalg::geometry::IShape* moved_shape_ = nullptr;
  linalg::geometry::Point* p1_ = nullptr;
  linalg::geometry::Point* p2_ = nullptr;
};

class PointTest : public GeometryTest {
 public:
  ~PointTest() override {
    delete expected_moved_point_;
  }
 protected:
  void ConstructTest(
      int64_t point_x,
      int64_t point_y,
      int64_t p1_x,
      int64_t p1_y,
      int64_t p2_x,
      int64_t p2_y,
      int64_t expected_x,
      int64_t expected_y) {
    shape_ = new linalg::geometry::Point(point_x, point_y);
    p1_ = new linalg::geometry::Point(p1_x, p1_y);
    p2_ = new linalg::geometry::Point(p2_x, p2_y);
    expected_moved_point_ = new linalg::geometry::Point(expected_x, expected_y);
  }

  void TestMove() override {
    SetupMoveTest();
    auto moved_point = dynamic_cast<linalg::geometry::Point*>(moved_shape_);
    ASSERT_TRUE(ArePointsEqual(*moved_point, *expected_moved_point_));
  }

  linalg::geometry::Point* expected_moved_point_;
};

class SegmentTest : public GeometryTest {
 public:
  ~SegmentTest() override {
    delete expected_moved_segment_;
  }
 protected:
  void ConstructTest(
      int64_t segment_p1_x,
      int64_t segment_p1_y,
      int64_t segment_p2_x,
      int64_t segment_p2_y,
      int64_t p1_x,
      int64_t p1_y,
      int64_t p2_x,
      int64_t p2_y,
      int64_t segment_expected_p1_x,
      int64_t segment_expected_p1_y,
      int64_t segment_expected_p2_x,
      int64_t segment_expected_p2_y) {
    shape_ = new linalg::geometry::Segment(
        linalg::geometry::Point(segment_p1_x, segment_p1_y),
        linalg::geometry::Point(segment_p2_x, segment_p2_y));
    p1_ = new linalg::geometry::Point(p1_x, p1_y);
    p2_ = new linalg::geometry::Point(p2_x, p2_y);
    expected_moved_segment_ = new linalg::geometry::Segment(
        linalg::geometry::Point(segment_expected_p1_x, segment_expected_p1_y),
        linalg::geometry::Point(segment_expected_p2_x, segment_expected_p2_y)
    );
  }

  static bool AreSegmentsEqual(
      const linalg::geometry::Segment& s1,
      const linalg::geometry::Segment& s2) {
    return ArePointsEqual(s1.get_a(), s2.get_a())
        && ArePointsEqual(s1.get_b(), s2.get_b());
  }

  void TestMove() override {
    SetupMoveTest();
    auto moved_segment = dynamic_cast<linalg::geometry::Segment*>(moved_shape_);
    ASSERT_TRUE(AreSegmentsEqual(*moved_segment, *expected_moved_segment_));
  }

  linalg::geometry::Segment* expected_moved_segment_;
};

class RayTest: public GeometryTest {
 public:
  ~RayTest() override {
    delete expected_moved_ray_point_;
    delete expected_moved_ray_vector_;
  }
 protected:
  void ConstructTest(
      int64_t ray_p1_x,
      int64_t ray_p1_y,
      int64_t ray_p2_x,
      int64_t ray_p2_y,
      int64_t p1_x,
      int64_t p1_y,
      int64_t p2_x,
      int64_t p2_y,
      int64_t ray_expected_p_x,
      int64_t ray_expected_p_y,
      int64_t ray_expected_v_x,
      int64_t ray_expected_v_y) {
    shape_ = new linalg::geometry::Ray(
        linalg::geometry::Point(ray_p1_x, ray_p1_y),
        linalg::geometry::Point(ray_p2_x, ray_p2_y));
    p1_ = new linalg::geometry::Point(p1_x, p1_y);
    p2_ = new linalg::geometry::Point(p2_x, p2_y);
    expected_moved_ray_point_ = new linalg::geometry::Point(ray_expected_p_x, ray_expected_p_y);
    expected_moved_ray_vector_ = new linalg::geometry::Vector(ray_expected_v_x, ray_expected_v_y);
  }


  void TestMove() override {
    SetupMoveTest();
    auto moved_ray = dynamic_cast<linalg::geometry::Ray*>(moved_shape_);
    ASSERT_TRUE(ArePointsEqual(*expected_moved_ray_point_, moved_ray->get_a()));
    AreVectorsSimilar(*expected_moved_ray_vector_, moved_ray->get_vector());
  }

  linalg::geometry::Point* expected_moved_ray_point_;
  linalg::geometry::Vector* expected_moved_ray_vector_;
};

class LineTest: public GeometryTest {
 protected:
  void ConstructTest(
      int64_t line_p1_x,
      int64_t line_p1_y,
      int64_t line_p2_x,
      int64_t line_p2_y,
      int64_t p1_x,
      int64_t p1_y,
      int64_t p2_x,
      int64_t p2_y,
      int64_t line_expected_a,
      int64_t line_expected_b,
      int64_t line_expected_c) {
    shape_ = new linalg::geometry::Line(
        linalg::geometry::Point(line_p1_x, line_p1_y),
        linalg::geometry::Point(line_p2_x, line_p2_y));
    p1_ = new linalg::geometry::Point(p1_x, p1_y);
    p2_ = new linalg::geometry::Point(p2_x, p2_y);
    line_expected_a_ = line_expected_a;
    line_expected_b_ = line_expected_b;
    line_expected_c_ = line_expected_c;
  }


  void TestMove() override {
    SetupMoveTest();
    auto moved_line = dynamic_cast<linalg::geometry::Line*>(moved_shape_);
    auto moved_line_a = moved_line->get_a();
    auto moved_line_b = moved_line->get_b();
    auto moved_line_c = moved_line->get_c();

    double a_k = 0;
    double b_k = 0;
    double c_k = 0;

    if (line_expected_a_ == 0 || moved_line_a == 0) {
      a_k = 0;
    } else {
      a_k = static_cast<double>(moved_line_a) / static_cast<double>(line_expected_a_);
    }

    if (line_expected_b_ == 0 || moved_line_b == 0) {
      b_k = 0;
    } else {
      b_k = static_cast<double>(moved_line_b) / static_cast<double>(line_expected_b_);
    }

    if (line_expected_c_ == 0 || moved_line_c == 0) {
      c_k = 0;
    } else {
      c_k = static_cast<double>(moved_line_c) / static_cast<double>(line_expected_c_);
    }

    if (a_k != 0 && b_k != 0) {
      ASSERT_DOUBLE_EQ(a_k, b_k);
    }

    if (a_k != 0 && c_k != 0) {
      ASSERT_DOUBLE_EQ(a_k, c_k);
    }

    if (b_k != 0 && c_k != 0) {
      ASSERT_DOUBLE_EQ(b_k, c_k);
    }
  }

  int64_t line_expected_a_;
  int64_t line_expected_b_;
  int64_t line_expected_c_;
};

class CircleTest : public GeometryTest {
 public:
  ~CircleTest() override {
    delete expected_moved_circle_;
  }
 protected:
  void ConstructTest(
      int64_t point_x,
      int64_t point_y,
      size_t r,
      int64_t p1_x,
      int64_t p1_y,
      int64_t p2_x,
      int64_t p2_y,
      int64_t expected_p_x,
      int64_t expected_p_y,
      size_t expected_r) {
    shape_ = new linalg::geometry::Circle(linalg::geometry::Point(point_x, point_y), r);
    p1_ = new linalg::geometry::Point(p1_x, p1_y);
    p2_ = new linalg::geometry::Point(p2_x, p2_y);
    expected_moved_circle_ = new linalg::geometry::Circle(
        linalg::geometry::Point(expected_p_x, expected_p_y), expected_r);
  }

  void TestMove() override {
    SetupMoveTest();
    auto moved_circle = dynamic_cast<linalg::geometry::Circle*>(moved_shape_);
    ASSERT_TRUE(ArePointsEqual(moved_circle->get_centre(), expected_moved_circle_->get_centre()));
    ASSERT_DOUBLE_EQ(moved_circle->get_radius(), expected_moved_circle_->get_radius());
  }

  linalg::geometry::Circle* expected_moved_circle_;
};
