#include <gtest/gtest.h>
#include <numbers>
#include <cmath>
#include <optional>

#include "intersections.hpp"
#include "geometry.hpp"

using namespace geometry;
using namespace geometry::intersections;
static constexpr double kEps = 1e-9;
static void ExpectOnCircle(const Point2D& p,const Circle&  c) {
    EXPECT_NEAR((p - c.center_p).Length(), c.radius, kEps);
}


TEST(IntersectionVisitor, LineLine_IntersectInside) {
    Line a{{0,0}, {2,0}};
    Line b{{1,-1}, {1, 1}};

    auto res = intersections::IntersectionVisitor{}(a, b);
    ASSERT_TRUE(res.has_value());
    EXPECT_DOUBLE_EQ(res->x, 1.0);
    EXPECT_DOUBLE_EQ(res->y, 0.0);

    Shape s1 = a, s2 = b;
    auto res2 = GetIntersectPoint(s1, s2);
    ASSERT_TRUE(res2);
    EXPECT_DOUBLE_EQ(res2->x, 1.0);
}

TEST(IntersectionVisitor, LineLine_NoIntersectParallel) {
    Line a{{0,0}, {2,0}};
    Line b{{0,1}, {2,1}};
    auto res = intersections::IntersectionVisitor{}(a, b);
    EXPECT_FALSE(res);

    Line c{{0,0}, {1,0}};
    Line d{{2,-1}, {2, 1}};
    EXPECT_FALSE(intersections::IntersectionVisitor{}(c, d));
}


TEST(IntersectionVisitor, LineCircle_IntersectTwoPointsReturnsFirstInside) {
    Circle circle{{0,0}, 1.0};
    Line   line {{-2,0}, { 2,0}};

    auto p = intersections::IntersectionVisitor{}(line, circle);
    ASSERT_TRUE(p);

    EXPECT_NEAR(p->x, -1.0, kEps);
    EXPECT_NEAR(p->y,  0.0, kEps);


    EXPECT_NEAR(intersections::IntersectionVisitor{}(circle, line)->x, -1.0, kEps);
}

TEST(IntersectionVisitor, LineCircle_Tangent) {
    Circle circle{{0,0}, 1.0};
    Line   line{{-1,1}, {1,1}};

    auto p = intersections::IntersectionVisitor{}(line, circle);
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->x, 0.0, kEps);
    EXPECT_NEAR(p->y, 1.0, kEps);
}

TEST(IntersectionVisitor, LineCircle_NoIntersect) {
    Circle circle{{0,0}, 1.0};
    Line   line{{2,2}, {3,3}};
    EXPECT_FALSE(intersections::IntersectionVisitor{}(line, circle));
}


TEST(IntersectionVisitor, CircleCircle_IntersectTwoPoints) {
    Circle circle1{{0,0}, 2.0};
    Circle circle2{{3,0}, 2.0};

    auto p = intersections::IntersectionVisitor{}(circle1, circle2);
    ASSERT_TRUE(p);

    ExpectOnCircle(*p, circle2);
    ExpectOnCircle(*p, circle1);
}

TEST(IntersectionVisitor, CircleCircle_TangentExternal) {
    Circle circle1{{0,0}, 1.0};
    Circle circle2{{2,0}, 1.0};

    auto p = intersections::IntersectionVisitor{}(circle1, circle2);
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->x, 1.0, kEps);
    EXPECT_NEAR(p->y, 0.0, kEps);
}

TEST(IntersectionVisitor, CircleCircle_NoIntersectDisjoint) {
    Circle circle1{{0,0}, 1.0};
    Circle circle2{{3,0}, 1.0};
    EXPECT_FALSE(intersections::IntersectionVisitor{}(circle1, circle2));
}

TEST(IntersectionVisitor, CircleCircle_ConcentricSameRadius) {
    Circle circle1{{0,0}, 1.0};
    Circle circle2{{0,0}, 1.0};
    EXPECT_FALSE(intersections::IntersectionVisitor{}(circle1, circle2));
}


TEST(IntersectionVisitor, UnsupportedCombinationThrows) {
    Line      line{{0,0}, {1,1}};
    Triangle  triangle{{0,0}, {1,0}, {0,1}};

    EXPECT_THROW(intersections::IntersectionVisitor{}(line, triangle), std::logic_error);
    EXPECT_THROW(intersections::IntersectionVisitor{}(triangle, line), std::logic_error);
    EXPECT_THROW(intersections::IntersectionVisitor{}(triangle, triangle), std::logic_error);

    Shape s1 = line,  s2 = triangle;
    EXPECT_FALSE(GetIntersectPoint(s1, s2).has_value());
}

