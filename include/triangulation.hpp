#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <set>
#include <vector>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    bool ContainsPoint(const Point2D &p) const {
        Point2D center = Circumcenter();
        double radius = Circumradius();
        return center.DistanceTo(p) <= radius + 1e-10;
    }

    Point2D Circumcenter() const {
        double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (std::abs(d) < 1e-10) {
            return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        }

        double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                     (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                    d;

        double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                     (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                    d;

        return {ux, uy};
    }

    double Circumradius() const {
        Point2D center = Circumcenter();
        return center.DistanceTo(a);
    }

    bool SharesEdge(const DelaunayTriangle &other) const {
        std::vector<Point2D> this_points = {a, b, c};
        std::vector<Point2D> other_points = {other.a, other.b, other.c};

        int shared_count = 0;
        for (const Point2D &p1 : this_points) {
            for (const Point2D &p2 : other_points) {
                if (std::abs(p1.x - p2.x) < 1e-10 && std::abs(p1.y - p2.y) < 1e-10) {
                    shared_count++;
                    break;
                }
            }
        }

        return shared_count == 2;
    }

    bool operator==(const DelaunayTriangle &) const = default;
    std::vector<Point2D> vertices() const { return {a, b, c}; }
};

struct Edge {
    Point2D p1, p2;

    Edge(Point2D p1, Point2D p2) : p1(p1), p2(p2) {
        if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y)) {
            std::swap(this->p1, this->p2);
        }
    }

    bool operator<(const Edge &other) const {
        if (std::abs(p1.x - other.p1.x) > 1e-10)
            return p1.x < other.p1.x;
        if (std::abs(p1.y - other.p1.y) > 1e-10)
            return p1.y < other.p1.y;
        if (std::abs(p2.x - other.p2.x) > 1e-10)
            return p2.x < other.p2.x;
        return p2.y < other.p2.y;
    }

    bool operator==(const Edge &other) const {
        return std::abs(p1.x - other.p1.x) < 1e-10 && std::abs(p1.y - other.p1.y) < 1e-10 &&
               std::abs(p2.x - other.p2.x) < 1e-10 && std::abs(p2.y - other.p2.y) < 1e-10;
    }
};

inline GeometryResult<std::vector<DelaunayTriangle>> DelaunayTriangulation(std::span<const Point2D> points) {

    if (points.size() < 3) {
        return std::unexpected(GeometryError::InvalidInput);
    }
    BoundingBox pointsBoundingBox{points};
    /*
    Триангуляция Делоне алгоритмом Боуэра-Ватсона

    - wiki с описанием триангуляции Делоне    - https://en.wikipedia.org/wiki/Delaunay_triangulation
    - wiki с описанием алгоритма и псевдокода - https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
    */

    // Создаём список для хранения текущей триангуляции и добавляем в него "Супер-треугольник",
    // содержащий внутри себя все точки
    const auto center = pointsBoundingBox.Center();
    const auto delta = std::max(pointsBoundingBox.Height(), pointsBoundingBox.Width()) * 10;
    Point2D super1{center.x - delta*2, center.y};
    Point2D super2{center.x - delta*2, center.y + delta*2};
    Point2D super3{center.x, center.y + delta*2};
    std::vector<DelaunayTriangle> triangulation;
    triangulation.insert(triangulation.end(), {super1, super2, super3});

    std::ranges::for_each(points, [&](auto &point) {
        std::vector<DelaunayTriangle> bad_triangles;
        std::ranges::for_each(triangulation, [&](auto &triangle) {
            if (triangle.ContainsPoint(point)) {
                bad_triangles.push_back(triangle);
            }
        });
        std::set<Edge> polygon;
        std::ranges::for_each(bad_triangles, [&](auto &triangle) {
            std::array<Edge, 3> local_edges = {
                Edge(triangle.a, triangle.b),
                Edge(triangle.b, triangle.c),
                Edge(triangle.c, triangle.a)
            };
            std::ranges::for_each(local_edges, [&](auto &edge) {
                if (const auto find = polygon.find(edge); find != polygon.end()) {
                    polygon.erase(find);
                } else {
                    polygon.insert(edge);
                }
            });
        });
        std::erase_if(triangulation, [&](auto &triangle) {
            return std::ranges::contains(bad_triangles, triangle);
        });
        std::ranges::for_each(polygon, [&](auto &edge) {
            triangulation.emplace_back(edge.p1, edge.p2, point);
        });
    });
    return triangulation;
}
}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};
