#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <filesystem>
#include <optional>
#include <variant>

namespace geometry::queries {

template <class... Ts>
struct Multilambda : Ts... {
    using Ts::operator()...;
};

/*
 * Класс для поиска расстояния от точки до фигуры
 *
 * Требуется организовать возможность нахождения расстояния для всех возможных фигур типа-суммы Shape
 */
struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    double operator()(const Line& l) const {
        const Point2D AB = l.end - l.start;
        const Point2D AP = point - l.start;
        const auto AB_len2 = AB.Dot(AB);

        if (AB_len2 == 0.0) {
            return point.DistanceTo(l.start);
        }

        auto scalar_projection = AP.Dot(AB) / AB_len2;

        scalar_projection = std::clamp(scalar_projection, 0.0, 1.0);
        const Point2D closest_point = l.start + AB * scalar_projection;
        return point.DistanceTo(closest_point);
    }

    double operator()(const Circle& c) const {
        return std::max(0.0, point.DistanceTo(c.center_p) - c.radius);
    }

    double operator()(const Triangle& t) const {
        const auto& vertices = t.Vertices();
        return get_minimal_distance(vertices);
    }

    double operator()(const Rectangle& r) const {
        const auto& vertices = r.Vertices();
        return get_minimal_distance(vertices);
    }

    double operator()(const RegularPolygon& r) const {
        const auto& vertices = r.Vertices();
        return get_minimal_distance(vertices);
    }

    double operator()(const Polygon& p) const {
        const auto& vertices = p.Vertices();
        return get_minimal_distance(vertices);
    }
private:
    [[nodiscard]] bool PointInsidePolygon(std::span<const Point2D> poly) const {
        constexpr double kEps = 1e-12;
        const std::size_t n = poly.size();
        if (n < 3) return false;

        auto on_edge = std::ranges::any_of(
            std::views::iota(std::size_t{0}, n),
            [&](const std::size_t& i) {
                const auto& A = poly[i];
                const auto& B = poly[(i + 1) % n];
                const Point2D AB = B - A;
                const Point2D AP = point - A;
                return std::abs(AB.Cross(AP)) < kEps
                    && std::min(A.x, B.x) - kEps <= point.x && point.x <= std::max(A.x, B.x) + kEps
                    && std::min(A.y, B.y) - kEps <= point.y && point.y <= std::max(A.y, B.y) + kEps;
            });

        if (on_edge)
            return true;


        auto crossings = std::views::iota(std::size_t{0}, n)
            | std::views::transform([&](std::size_t i) {
                  const auto& A = poly[i];
                  const auto& B = poly[(i + 1) % n];

                  if (const bool condY = (A.y > point.y) != (B.y > point.y); not condY)
                      return 0;

                  const double x_int = A.x + (point.y - A.y) * (B.x - A.x) / (B.y - A.y);
                  return (x_int > point.x + kEps) ? 1 : 0;
              });

        const int parity = std::ranges::fold_left(crossings, 0, std::bit_xor<>{});
        return parity == 1;
    }

    [[nodiscard]] double get_minimal_distance(const std::span<const Point2D>& Points) const {
        if (PointInsidePolygon(Points))
            return 0.0;

        auto lines = Points | std::views::adjacent<2>;
        return std::ranges::min(lines | std::views::transform([&](const std::pair<Point2D, Point2D>& p) {
            return this->operator()(Line{ p.first, p.second });
        }));
    }
};

/*
 * Класс для поиска расстояния между двумя фигурами
 *
 * Требуется организовать возможность нахождения расстояния только для следующих комбинаций фигур:
 *    - Any    & Point
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется вернуть пустое значение
 */
struct ShapeToShapeDistanceVisitor {

    [[nodiscard]] std::optional<double> operator()(const auto &shape, const Point2D &point) const noexcept {
        return std::visit(PointToShapeDistanceVisitor{point}, shape);
    }

    [[nodiscard]] std::optional<double> operator()(const Point2D &point, const auto &shape) const noexcept {
        return this->operator()(shape, point);
    }

    [[nodiscard]] std::optional<double> operator()(const Line &lhs, const Line &rhs) const noexcept {
        return std::min({
            PointToShapeDistanceVisitor{lhs.start}(rhs),
            PointToShapeDistanceVisitor{lhs.end}(rhs),
            PointToShapeDistanceVisitor{rhs.start}(lhs),
            PointToShapeDistanceVisitor{rhs.end}(lhs)
        });
    }

    [[nodiscard]] std::optional<double> operator()(const Circle &lhs, const Circle &rhs) const noexcept {
        const auto distance = lhs.Center().DistanceTo(rhs.Center());

        if (const auto sum_of_radius = lhs.radius + rhs.radius; distance > sum_of_radius) {
            return distance - sum_of_radius;
        }

        return 0.0;
    }

    [[nodiscard]] std::optional<double> operator()(const auto &, const auto &) const noexcept { return std::nullopt; }
};

/*
 * Функции-помощники
 */
inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    return std::visit(PointToShapeDistanceVisitor{point}, shape);
}

inline BoundingBox GetBoundBox(const Shape &shape) {
    return std::visit([](const auto &s) { return s.BoundBox(); }, shape);
}

inline double GetHeight(const Shape &shape) {
    return std::visit([](const auto &s) { return s.Height(); }, shape);
}

inline bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2) {
    return GetBoundBox(shape1).Overlaps(GetBoundBox(shape2));
}

inline std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    return std::visit(ShapeToShapeDistanceVisitor{}, shape1, shape2);
}

}  // namespace geometry::queries