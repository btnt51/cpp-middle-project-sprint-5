#pragma once
#include "geometry.hpp"
#include <cmath>
#include <optional>

namespace geometry::intersections {

/*
 * Класс для поиска пересечений между двумя фигурами
 *
 * Требуется организовать возможность нахождения пересечений только для следующих комбинаций фигур:
 *    - Line   & Line
 *    - Line   & Circle
 *    - Circle & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется выбросить исключение std::logic_error
 */
class IntersectionVisitor {
static constexpr double kEps = 1e-9;
public:
    std::optional<Point2D> operator()(const Line& l1, const Line& l2) const {
        const Point2D p = l1.start;
        const Point2D r = l1.end - l1.start;

        const Point2D q = l2.start;
        const Point2D s = l2.end - l2.start;

        const double rxs = r.Cross(s);
        if (std::abs(rxs) < kEps) {
            return std::nullopt;
        }

        const double t = (q - p).Cross(s) / rxs;
        const double u = (q - p).Cross(r) / rxs;

        if (t < -kEps || t > 1.0 + kEps || u < -kEps || u > 1.0 + kEps) {
            return std::nullopt;
        }
        return p + r * t;
    }

    std::optional<Point2D> operator()(const Line& line, const Circle& circle) const {
        const Point2D d = line.end - line.start;
        const Point2D f = line.start - circle.center_p;

        const double a = d.Dot(d);
        const double b = 2.0 * f.Dot(d);
        const double c = f.Dot(f) - circle.radius * circle.radius;

        double D = b * b - 4 * a * c;
        if (D < 0) return std::nullopt;

        D = std::sqrt(D);
        const double t1 = (-b - D) / (2 * a);
        const double t2 = (-b + D) / (2 * a);

        auto inside = [&](double t) { return t >= -kEps && t <= 1.0 + kEps; };

        if (inside(t1)) return line.start + d * t1;
        if (inside(t2)) return line.start + d * t2;
        return std::nullopt;
    }

    std::optional<Point2D> operator()(const Circle& circle, const Line& line) const {
        return operator()(line, circle);
    }


    std::optional<Point2D> operator()(const Circle& c1, const Circle& c2) const {
        const Point2D dVec = c2.center_p - c1.center_p;
        const double d = dVec.Length();


        if (d > c1.radius + c2.radius + kEps) return std::nullopt;
        if (d < std::abs(c1.radius - c2.radius) - kEps) return std::nullopt;
        if (d < kEps && std::abs(c1.radius - c2.radius) < kEps) return std::nullopt;


        const double a = (c1.radius * c1.radius - c2.radius * c2.radius + d * d) / (2 * d);
        const double h2 = c1.radius * c1.radius - a * a;
        if (h2 < -kEps) return std::nullopt;

        const Point2D p2 = c1.center_p + dVec * (a / d);

        if (std::abs(h2) < kEps) {
            return p2;
        }

        const double h = std::sqrt(h2);
        Point2D offset{ -dVec.y * (h / d), dVec.x * (h / d) };

        return p2 + offset;
    }

    std::optional<Point2D> operator()(const auto&, const auto&) const {
        throw std::logic_error("Trying to get IntersectPoint between 2 unsupported objects");
    }

};

inline std::optional<Point2D> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    try {
        return std::visit(IntersectionVisitor{}, shape1, shape2);
    } catch (std::logic_error &e) {
        std::println("Случилась ошибка: {}", e.what());
    }
    return std::nullopt;
}

}  // namespace geometry::intersections