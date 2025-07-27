#include "convex_hull.hpp"
#include <algorithm>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.Cross(new_p2);
}


GeometryResult<std::vector<Point2D>> GrahamScan(const std::span<const Point2D>& points) {
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InvalidInput};
    }

    Point2D pivot = *std::ranges::min_element(points, [](const auto& a, const auto& b) {
        return a < b;
    });

    std::vector sorted_points(points.begin(), points.end());

    std::ranges::sort(sorted_points, [&](const Point2D& a, const Point2D& b) {
        if (a == pivot) return true;
        if (b == pivot) return false;
        const double cross = CrossProduct(a, pivot, b);
        if (std::abs(cross) < 1e-10) {
            return pivot.DistanceTo(a) < pivot.DistanceTo(b);
        }
        return cross > 0;
    });

    StackForGrahamScan stack;
    for (const auto& p : sorted_points) {
        while (stack.Size() >= 2 && stack.MakesRightTurn(p)) {
            stack.Pop();
        }
        stack.Push(p);
    }

    return std::move(stack).Extract();
}

}  // namespace geometry::convex_hull