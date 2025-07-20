#include <gtest/gtest.h>
#include <numbers>
#include <ranges>
#include <vector>
#include <cmath>

#include "geometry.hpp"

using namespace ::testing;
using namespace geometry;

static constexpr double kEps = 1e-9;


TEST(Point2D, DefaultCtor) {
    Point2D p;
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
}

TEST(Point2D, CtorXY) {
    Point2D p{1.5, -2.3};
    EXPECT_DOUBLE_EQ(p.x, 1.5);
    EXPECT_DOUBLE_EQ(p.y, -2.3);
}

TEST(Point2D, ComparisonOperators) {
    Point2D a{1, 2}, b{1, 2}, c{2, 3}, d{2, 1};

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);

    EXPECT_TRUE(a < c);
    EXPECT_FALSE(c < a);
    EXPECT_FALSE(c < d);
}

TEST(Point2D, ArithmeticOperators) {
    Point2D a{1, 2}, b{3, 5};

    const Point2D ca{1, 2};

    auto sum = a + b;
    EXPECT_DOUBLE_EQ(sum.x, 4.0);
    EXPECT_DOUBLE_EQ(sum.y, 7.0);

    auto diff = ca - b;
    EXPECT_DOUBLE_EQ(diff.x, -2.0);
    EXPECT_DOUBLE_EQ(diff.y, -3.0);

    auto mul = a * 2.0;
    EXPECT_DOUBLE_EQ(mul.x, 2.0);
    EXPECT_DOUBLE_EQ(mul.y, 4.0);

    auto div = b / 2.0;
    EXPECT_DOUBLE_EQ(div.x, 1.5);
    EXPECT_DOUBLE_EQ(div.y, 2.5);
}

TEST(Point2D, GeometryOps) {
    Point2D a{1, 0}, b{0, 1};
    EXPECT_DOUBLE_EQ(a.Dot(b), 0.0);
    EXPECT_DOUBLE_EQ(a.Cross(b), 1.0);
    EXPECT_DOUBLE_EQ(a.Length(), 1.0);
    EXPECT_NEAR(a.DistanceTo(b), std::sqrt(2.0), kEps);

    Point2D zero{0, 0};
    auto n = zero.Normalize();
    EXPECT_DOUBLE_EQ(n.x, 0.0);
    EXPECT_DOUBLE_EQ(n.y, 0.0);
}

TEST(Point2D, NormalizeNonZero) {
    Point2D p{3, 4};
    Point2D n = p.Normalize();
    EXPECT_NEAR(n.x, 0.6, kEps);
    EXPECT_NEAR(n.y, 0.8, kEps);
    EXPECT_NEAR(n.Length(), 1.0, kEps);
}


TEST(BoundingBox, DefaultCtor) {
    BoundingBox bb;
    EXPECT_TRUE(std::isinf(bb.min_x));
    EXPECT_TRUE(std::isinf(bb.min_y));
    EXPECT_TRUE(std::isinf(bb.max_x));
    EXPECT_TRUE(std::isinf(bb.max_y));
}

TEST(BoundingBox, CtorTwoPoints) {
    BoundingBox bb{{0, 1}, {2, 3}};
    EXPECT_DOUBLE_EQ(bb.min_x, 0);
    EXPECT_DOUBLE_EQ(bb.min_y, 1);
    EXPECT_DOUBLE_EQ(bb.max_x, 2);
    EXPECT_DOUBLE_EQ(bb.max_y, 3);
}

TEST(BoundingBox, CtorRange) {
    std::vector<Point2D> pts{{1, 5}, {-2, 4}, {3, -1}};
    BoundingBox bb{pts};
    EXPECT_DOUBLE_EQ(bb.min_x, -2);
    EXPECT_DOUBLE_EQ(bb.min_y, -1);
    EXPECT_DOUBLE_EQ(bb.max_x, 3);
    EXPECT_DOUBLE_EQ(bb.max_y, 5);
}

TEST(BoundingBox, Overlaps) {
    BoundingBox a{0, 0, 2, 2};
    BoundingBox b{1, 1, 3, 3};
    BoundingBox c{3, 3, 4, 4};

    EXPECT_TRUE(a.Overlaps(b));
    EXPECT_FALSE(a.Overlaps(c));
}

TEST(BoundingBox, Contains) {
    BoundingBox bb{0, 0, 10, 10};
    EXPECT_TRUE(bb.Contains({5, 5}));
    EXPECT_TRUE(bb.Contains({0, 0}));
    EXPECT_TRUE(bb.Contains({10, 10}));
    EXPECT_FALSE(bb.Contains({-1, 0}));
    EXPECT_FALSE(bb.Contains({11, 10}));
}

TEST(BoundingBox, WidthHeightCenter) {
    BoundingBox bb{0, 0, 10, 20};
    EXPECT_DOUBLE_EQ(bb.Width(), 10);
    EXPECT_DOUBLE_EQ(bb.Height(), 20);
    auto c = bb.Center();
    EXPECT_DOUBLE_EQ(c.x, 5);
    EXPECT_DOUBLE_EQ(c.y, 10);
}

TEST(Line, Basic) {
    Line l{{0, 0}, {3, 4}};
    auto d = l.Direction();
    EXPECT_DOUBLE_EQ(d.x, 3);
    EXPECT_DOUBLE_EQ(d.y, 4);

    auto bb = l.BoundBox();
    EXPECT_DOUBLE_EQ(bb.min_x, 0);
    EXPECT_DOUBLE_EQ(bb.min_y, 0);
    EXPECT_DOUBLE_EQ(bb.max_x, 3);
    EXPECT_DOUBLE_EQ(bb.max_y, 4);

    EXPECT_DOUBLE_EQ(l.Height(), 4.0);

    auto c = l.Center();
    EXPECT_DOUBLE_EQ(c.x, 1.5);
    EXPECT_DOUBLE_EQ(c.y, 2.0);


    EXPECT_DOUBLE_EQ(l.Length(), 3 + 4);

    auto v = l.Vertices();
    EXPECT_EQ(v.size(), 2u);
    EXPECT_DOUBLE_EQ(v[0].x, 0);
    EXPECT_DOUBLE_EQ(v[1].x, 3);

    auto lines = l.Lines();
    EXPECT_EQ(lines.x.size(), 2u);
    EXPECT_EQ(lines.y.size(), 2u);
    EXPECT_DOUBLE_EQ(lines.x[0], 0);
    EXPECT_DOUBLE_EQ(lines.x[1], 3);
}

TEST(Triangle, AreaHeightCenterBBox) {
    Triangle t{{0, 0}, {4, 0}, {0, 3}};
    EXPECT_DOUBLE_EQ(t.Area(), 6.0);

    auto bb = t.BoundBox();
    EXPECT_DOUBLE_EQ(bb.min_x, 0);
    EXPECT_DOUBLE_EQ(bb.min_y, 0);
    EXPECT_DOUBLE_EQ(bb.max_x, 4);
    EXPECT_DOUBLE_EQ(bb.max_y, 3);

    EXPECT_DOUBLE_EQ(t.Height(), 3.0);

    auto c = t.Center();
    EXPECT_DOUBLE_EQ(c.x, 2.0);
    EXPECT_DOUBLE_EQ(c.y, 1.5);

    auto v = t.Vertices();
    ASSERT_EQ(v.size(), 3u);
    EXPECT_DOUBLE_EQ(v[0].x, 0);
    EXPECT_DOUBLE_EQ(v[1].x, 4);
    EXPECT_DOUBLE_EQ(v[2].x, 0);

    auto lines = t.Lines();
    EXPECT_EQ(lines.x.size(), 4u);
    EXPECT_EQ(lines.y.size(), 4u);
    EXPECT_DOUBLE_EQ(lines.x[0], t.a.x);
    EXPECT_DOUBLE_EQ(lines.x[3], t.a.x);
}

TEST(Rectangle, AreaBBoxVerticesLines) {
    Rectangle r{{1, 2}, 3, 4};
    EXPECT_DOUBLE_EQ(r.Area(), 12.0);

    auto bb = r.BoundBox();
    EXPECT_DOUBLE_EQ(bb.min_x, 1);
    EXPECT_DOUBLE_EQ(bb.min_y, 2);
    EXPECT_DOUBLE_EQ(bb.max_x, 4);
    EXPECT_DOUBLE_EQ(bb.max_y, 6);

    EXPECT_DOUBLE_EQ(r.Height(), 4);
    EXPECT_DOUBLE_EQ(r.Width(), 3);

    auto c = r.Center();
    EXPECT_DOUBLE_EQ(c.x, 2.5);
    EXPECT_DOUBLE_EQ(c.y, 4.0);

    auto v = r.Vertices();
    ASSERT_EQ(v.size(), 4u);
    EXPECT_DOUBLE_EQ(v[0].x, 1);
    EXPECT_DOUBLE_EQ(v[1].x, 4);
    EXPECT_DOUBLE_EQ(v[2].x, 4);
    EXPECT_DOUBLE_EQ(v[3].x, 1);

    auto lines = r.Lines();
    EXPECT_EQ(lines.x.size(), 4u);
    EXPECT_EQ(lines.y.size(), 4u);
    EXPECT_DOUBLE_EQ(lines.x[0], 1);
    EXPECT_DOUBLE_EQ(lines.y[2], 6);
}


TEST(RegularPolygon, VerticesBBoxCenterHeight) {
    RegularPolygon p{{0, 0}, 1.0, 4};

    auto verts = p.Vertices();
    ASSERT_EQ(verts.size(), 4u);
    for (auto& v : verts) {
        EXPECT_NEAR(std::hypot(v.x - 0.0, v.y - 0.0), 1.0, 1e-12);
    }

    auto bb = p.BoundBox();
    EXPECT_TRUE(bb.Width() > 0);
    EXPECT_TRUE(bb.Height() > 0);

    EXPECT_DOUBLE_EQ(p.Center().x, 0.0);
    EXPECT_DOUBLE_EQ(p.Center().y, 0.0);

    EXPECT_DOUBLE_EQ(p.Height(), bb.Height());
}


TEST(Circle, BoundBoxVerticesLines) {
    Circle c{{0, 0}, 2.0};

    auto bb = c.BoundBox();
    EXPECT_DOUBLE_EQ(bb.min_x, -2.0);
    EXPECT_DOUBLE_EQ(bb.min_y, -2.0);
    EXPECT_DOUBLE_EQ(bb.max_x,  2.0);
    EXPECT_DOUBLE_EQ(bb.max_y,  2.0);

    EXPECT_DOUBLE_EQ(c.Height(), c.center_p.y + c.radius);

    EXPECT_DOUBLE_EQ(c.Center().x, 0.0);
    EXPECT_DOUBLE_EQ(c.Center().y, 0.0);

    auto verts = c.Vertices(10);
    ASSERT_EQ(verts.size(), 10u);
    for (auto& v : verts) {
        EXPECT_NEAR(std::hypot(v.x, v.y), 2.0, 1e-12);
    }

    auto lines = c.Lines(10);
    EXPECT_EQ(lines.x.size(), 20u);
    EXPECT_EQ(lines.y.size(), 20u);
}


TEST(Polygon, CenterAreaBBoxVerticesLines) {
    std::vector<Point2D> pts{{0, 0}, {4, 0}, {4, 3}, {0, 3}};
    Polygon poly{pts};

    auto bb = poly.BoundBox();
    EXPECT_DOUBLE_EQ(bb.min_x, 0.0);
    EXPECT_DOUBLE_EQ(bb.min_y, 0.0);
    EXPECT_DOUBLE_EQ(bb.max_x, 4.0);
    EXPECT_DOUBLE_EQ(bb.max_y, 3.0);

    EXPECT_DOUBLE_EQ(poly.Height(), 3.0);

    auto c = poly.Center();
    EXPECT_DOUBLE_EQ(c.x, 2.0);
    EXPECT_DOUBLE_EQ(c.y, 1.5);

    auto v = poly.Vertices();
    ASSERT_EQ(v.size(), pts.size());
    EXPECT_EQ(v, pts);

    auto lines = poly.Lines();
    EXPECT_EQ(lines.x.size(), pts.size());
    EXPECT_EQ(lines.y.size(), pts.size());
}


TEST(RegularPolygon, Sides1or2AreDegenerate) {
    RegularPolygon p1{{0,0}, 1.0, 1};
    auto v1 = p1.Vertices();
    EXPECT_EQ(v1.size(), 1u);

    RegularPolygon p2{{0,0}, 1.0, 2};
    auto v2 = p2.Vertices();
    EXPECT_EQ(v2.size(), 2u);
}

TEST(Circle, ZeroRadius) {
    Circle c{{3, 4}, 0.0};
    auto bb = c.BoundBox();
    EXPECT_DOUBLE_EQ(bb.min_x, 3.0);
    EXPECT_DOUBLE_EQ(bb.min_y, 4.0);
    EXPECT_DOUBLE_EQ(bb.max_x, 3.0);
    EXPECT_DOUBLE_EQ(bb.max_y, 4.0);

    auto verts = c.Vertices(5);
    for (auto& v : verts) {
        EXPECT_DOUBLE_EQ(v.x, 3.0);
        EXPECT_DOUBLE_EQ(v.y, 4.0);
    }
}

TEST(BoundingBox, OverlapsTouchingEdges) {
    BoundingBox a{0, 0, 1, 1};
    BoundingBox b{1, 1, 2, 2};
    EXPECT_TRUE(a.Overlaps(b));
}


TEST(Lines2DDyn, ReservePushFront) {
    Lines2DDyn l;
    l.Reserve(2);
    l.PushBack(Point2D{1, 2});
    l.PushBack(3, 4);

    auto front = l.Front();
    EXPECT_DOUBLE_EQ(front.x, 1.0);
    EXPECT_DOUBLE_EQ(front.y, 2.0);
    ASSERT_EQ(l.x.size(), 2u);
    ASSERT_EQ(l.y.size(), 2u);
}
