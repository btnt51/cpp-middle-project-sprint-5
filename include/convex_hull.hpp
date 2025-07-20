#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <ranges>
#include <stack>
#include <vector>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2);

class StackForGrahamScan {
public:
    void Push(const Point2D &p) { s.push_back(p); }
    void Pop() { s.pop_back(); }

    [[nodiscard]] size_t Size() { return s.size(); }
    [[nodiscard]] Point2D Top() const { return s.back(); }
    [[nodiscard]] Point2D NextToTop() const { return *std::prev(s.end(), 2); }

    std::vector<Point2D> &&Extract() && { return std::move(s); }

    [[nodiscard]] bool MakesRightTurn(const Point2D& next_point) const {
        if (s.size() < 2) return false;
        return CrossProduct(NextToTop(), Top(), next_point) <= 0;
    }

private:
    std::vector<Point2D> s;
};

GeometryResult<std::vector<Point2D>> GrahamScan(const std::span<const Point2D>& points);

}  // namespace geometry::convex_hull