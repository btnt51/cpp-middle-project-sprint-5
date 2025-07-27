#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <format>
#include <numbers>
#include <optional>
#include <print>
#include <ranges>
#include <variant>
#include <vector>
#include <format>

namespace geometry {

/*
 * В коде везде используется ReplaceMe. Ваша задача - удалить ReplaceMe и везде вместо него
 использовать наиболее подходящий тип для решения задачи
 */
// struct ReplaceMe {
//     ReplaceMe(std::vector<Shape>) {}
// };

/*
 * Добавьте к методам класса Point2D и Lines2DDyn все необходимые аттрибуты и спецификаторы
 * Важно: Возвращаемый тип и принимаемые аргументы менять не нужно
 */
struct Point2D {
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    bool operator<(const Point2D &other) const { return x < other.x && y < other.y; }
    bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }

    // Binary math operators
    Point2D operator+(const Point2D &other) { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D &other) { return {x - other.x, y - other.y}; }
    Point2D operator*(double value) { return {x * value, y * value}; }
    Point2D operator/(double value) { return {x / value, y / value}; }

    Point2D operator+(const Point2D &other) const { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D &other) const { return {x - other.x, y - other.y}; }
    Point2D operator*(double value) const { return {x * value, y * value}; }
    Point2D operator/(double value) const { return {x / value, y / value}; }

    // Binary geometry operations
    double Dot(const Point2D &other) { return x * other.x + y * other.y; }
    double Cross(const Point2D &other) { return x * other.y - y * other.x; }
    double Length() { return std::sqrt(x * x + y * y); }
    double DistanceTo(const Point2D &other) { return (*this - other).Length(); }

    const double Dot(const Point2D &other) const { return x * other.x + y * other.y; }
    const double Cross(const Point2D &other) const { return x * other.y - y * other.x; }
    const double Length() const { return std::sqrt(x * x + y * y); }
    const double DistanceTo(const Point2D &other) const { return (*this - other).Length(); }

    Point2D Normalize() {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};

template <size_t N>
struct Lines2D {
    std::array<double, N> x;
    std::array<double, N> y;
};

struct Lines2DDyn {
    std::vector<double> x;
    std::vector<double> y;

    void Reserve(size_t n) {
        x.reserve(n);
        y.reserve(n);
    }
    void PushBack(Point2D p) {
        x.push_back(p.x);
        y.push_back(p.y);
    }
    void PushBack(double px, double py) {
        x.push_back(px);
        y.push_back(py);
    }
    Point2D Front() { return {x.front(), y.front()}; }
};

struct BoundingBox {
    double min_x, min_y, max_x, max_y;

    constexpr BoundingBox() : min_x(INF), min_y(INF), max_x(-INF), max_y(-INF)  {}
    constexpr BoundingBox(double min_x, double min_y, double max_x, double max_y) : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {}

    constexpr BoundingBox(const Point2D &p1, const Point2D &p2) : min_x(p1.x), min_y(p1.y), max_x(p2.x), max_y(p2.y) {}

    template <std::ranges::range R>
    constexpr BoundingBox(R&& r) {
        min_x = std::ranges::min(r, std::less{}, &Point2D::x).x;
        min_y = std::ranges::min(r, std::less{}, &Point2D::y).y;
        max_x = std::ranges::max(r, std::less{}, &Point2D::x).x;
        max_y = std::ranges::max(r, std::less{}, &Point2D::y).y;

    }

    constexpr BoundingBox(std::initializer_list<Point2D> il)
        : BoundingBox(std::ranges::subrange(il.begin(), il.end())) {}

    [[nodiscard]] bool Overlaps(const BoundingBox &other) const {
        return not (max_x < other.min_x or other.max_x < min_x or max_y < other.min_y or other.max_y < min_y);
    }

    [[nodiscard]] double Width() const {
        return max_x - min_x;
    }

    [[nodiscard]] double Height() const {
        return max_y - min_y;
    }

    [[nodiscard]] Point2D Center() const {
       return {(min_x + max_x) / 2, (min_y + max_y) / 2};
    }

    [[nodiscard]] bool Contains(const Point2D& point) const {
        return (min_x <= point.x and min_y <= point.y) and (max_x >= point.x and max_y >= point.y);
    }

private:
    static constexpr auto INF = std::numeric_limits<double>::infinity();
};

struct Line {
    Point2D start, end;

    /* ваш код здесь */
    [[nodiscard]] Point2D Direction() const {
        return end - start;
    }

    [[nodiscard]] BoundingBox BoundBox() const {
        return BoundingBox{start, end};
    }

    [[nodiscard]] double Height() const {
        return BoundBox().Height();
    }

    [[nodiscard]] Point2D Center() const {
        return BoundBox().Center();
    }

    [[nodiscard]] double Length() const {
        return std::abs(end.x - start.x) + std::abs(end.y - start.y);
    }

    [[nodiscard]] std::array<Point2D, 2> Vertices() const {
        return {start, end};
    }

    [[nodiscard]] Lines2D<2> Lines() const {
        return {{start.x, end.x}, {start.y, start.y}};
    }

};

struct Triangle {
    Point2D a, b, c;

    //
    // Обратите внимание! В методе Lines(), в отличие от Vertices(), координаты точек замыкаются на начало:
    // a.x, b.x, c.x а затем идёт снова первая вершина a.x
    //
    // Это необходимо для правильного рисования фигур через gnuplot, который формирует линии используя пары точек.
    // В случае с  Triangle будут составлены такие пары точек:
    //      - { a, b }
    //      - { b, c }
    //      - { c, a }
    //
    std::array<Point2D, 3> Vertices() const { return {a, b, c}; }
    Lines2D<4> Lines() const { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }

    /* ваш код здесь */
    [[nodiscard]] double Area() const {
        return 0.5 * std::abs(a.x * (b.y-c.y) + b.x * (c.y-a.y) + c.x * (a.y-b.y));
    }

    [[nodiscard]] BoundingBox BoundBox() const {
        return BoundingBox{std::initializer_list{a,b,c}};
    }

    [[nodiscard]] Point2D Center() const {
        return BoundBox().Center();
    }

    [[nodiscard]] double Height() const {
        return BoundBox().Height();
    }
};

struct Rectangle {
    Point2D bottom_left;
    double width, height;

    /* ваш код здесь */
    [[nodiscard]] double Area() const {
        return width * height;
    }

    [[nodiscard]] BoundingBox BoundBox() const {
        return BoundingBox{bottom_left, {bottom_left.x + width, bottom_left.y + height}};
    }

    [[nodiscard]] Point2D Center() const {
        return BoundBox().Center();
    }

    [[nodiscard]] double Height() const {
        return BoundBox().Height();
    }

    [[nodiscard]] double Width() const {
        return BoundBox().Width();
    }

    [[nodiscard]] Lines2D<4> Lines() const {
        const double x0 = bottom_left.x;
        const double y0 = bottom_left.y;
        const double x1 = x0 + width;
        const double y1 = y0 + height;

        return Lines2D<4>{
            .x = {x0, x1, x1, x0},
            .y = {y0, y0, y1, y1},
        };
    }


    [[nodiscard]] std::array<Point2D, 4> Vertices() const {
        return {
            bottom_left,
            Point2D{bottom_left.x + width, bottom_left.y},
            Point2D{bottom_left.x + width, bottom_left.y + height},
            Point2D{bottom_left.x, bottom_left.y + height},
        };
    }
};

struct RegularPolygon {
    Point2D center_p;
    double radius;
    int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    std::vector<Point2D> Vertices() const {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }

    [[nodiscard]] BoundingBox BoundBox() const {
        return BoundingBox{Vertices()};
    }

    [[nodiscard]] double Height() const {
        return BoundBox().Height();
    }

    [[nodiscard]] Point2D Center() const {
        return center_p;
    }

    [[nodiscard]] Lines2D<4> Lines() const {
        auto vertices = Vertices();
        Lines2D<4> lines;
        for (size_t i = 0; i < 4; ++i) {
            lines.x[i] = vertices[i].x;
            lines.y[i] = vertices[i].y;
        }
        return lines;
    }
};

struct Circle {
    Point2D center_p;
    double radius;

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    [[nodiscard]] BoundingBox BoundBox() const {
        return {center_p.x - radius, center_p.y - radius, center_p.x + radius, center_p.y + radius};
    }
    [[nodiscard]] double Height() const { return center_p.y + radius; }
    [[nodiscard]] Point2D Center() const { return center_p; }

    //
    // Должны быть сделана по аналогии с RegularPolygon::Vertices
    //
    [[nodiscard]] std::vector<Point2D> Vertices(size_t N = 30) const {
        std::vector<Point2D> points;
        points.reserve(N);

        for (size_t i = 0; i < N; ++i) {
            const double angle = 2 * std::numbers::pi * i / N;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }

    [[nodiscard]] Lines2DDyn Lines(size_t N = 100) const {
        auto vertices = Vertices(N);
        Lines2DDyn lines;
        lines.Reserve(N);
        for (size_t i = 0; i < N; ++i) {
            auto line = Line{vertices[i], vertices[(i + 1) % N]};
            lines.PushBack(line.start);
            lines.PushBack(line.end);
        }
        return lines;
    }
};

class Polygon {
public:
    Polygon() = default;

    explicit Polygon(std::vector<Point2D> vertices) : points_(std::move(vertices)), bounding_box_{points_} {
    }

    [[nodiscard]] BoundingBox BoundBox() const { return bounding_box_; }

    [[nodiscard]] Point2D Center() const {
        return std::ranges::fold_left(points_, Point2D{0, 0}, std::plus<>{}) / points_.size();
    }

    [[nodiscard]] double Height() const {
        return BoundBox().Height();
    }

    [[nodiscard]] std::vector<Point2D> Vertices() const { return points_; }

    [[nodiscard]] Lines2DDyn Lines() const {
        Lines2DDyn lines;
        lines.Reserve(points_.size());
        for (size_t i = 0; i < points_.size(); ++i) {
            lines.PushBack(points_[i]);
        }
        return lines;
    }

private:
    std::vector<Point2D> points_;
    BoundingBox bounding_box_;
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    constexpr auto format(geometry::Point2D p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};

template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) {
        constexpr std::string_view kTag = "new_line}";

        auto it = ctx.begin();
        if (it != ctx.end() && *it == ':') {
            ++it;
            if (std::ranges::equal(it, ctx.end(), kTag.begin(), kTag.end())) {
                use_new_line = true;
                it = ctx.end();
            } else {
                throw std::format_error("Only {:new_line} is supported");
            }
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {
        auto out = ctx.out();
        if (use_new_line) {
            out = std::format_to(out, "[\n");
            for (const auto &p : v) {
                out = std::format_to(out, "\t{},\n", p);
            }
            out = std::format_to(out, "]");
        } else {
            out = std::format_to(out, "[");
            bool first = true;
            for (const auto &p : v) {
                if (!first) {
                    out = std::format_to(out, ", ");
                }
                first = false;
                out = std::format_to(out, "{}", p);
            }
            out = std::format_to(out, "]");
        }
        return out;
    }
};

template <>
struct std::formatter<geometry::Line> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Line &l, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Line({}, {})", l.start, l.end);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center_p, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
        auto format(const geometry::Rectangle &r, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.bottom_left, r.width,
                              r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};

template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.a, t.b, t.c);
    }
};

template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) const {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};