#pragma once
#include "geometry.hpp"
#include "queries.hpp"
#include <print>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

namespace geometry::utils {

class ShapeGenerator {
public:
    ShapeGenerator(double min_coord = -100.0, double max_coord = 100.0, double min_size = 1.0, double max_size = 20.0)
        : gen(20), coord_dist(min_coord, max_coord), size_dist(min_size, max_size), sides_dist(3, 12), type_dist(0, 4) {
    }

    Shape GenerateRandomShape() {
        Point2D center{coord_dist(gen), coord_dist(gen)};
        double size = size_dist(gen);

        switch (type_dist(gen)) {
        case 0: {
            Point2D end{center.x + size, center.y + size};
            return Line{center, end};
        }
        case 1: {
            Point2D a{center.x, center.y};
            Point2D b{center.x + size, center.y};
            Point2D c{center.x + size / 2, center.y + size};
            return Triangle{a, b, c};
        }
        case 2: {
            return Rectangle{center, size, size * 0.8};
        }
        case 3: {
            int sides = sides_dist(gen);
            return RegularPolygon{center, size, sides};
        }
        case 4: {
            return Circle{center, size};
        }
        }
        return Circle{center, size};
    }

    std::vector<Shape> GenerateShapes(size_t count) {
        std::vector<Shape> shapes;
        shapes.reserve(count);

        for (auto _ : std::views::iota(0u, count)) {
            shapes.push_back(GenerateRandomShape());
        }

        return shapes;
    }

private:
    std::mt19937 gen;
    std::uniform_real_distribution<double> coord_dist;
    std::uniform_real_distribution<double> size_dist;
    std::uniform_int_distribution<int> sides_dist;
    std::uniform_int_distribution<int> type_dist;
};

std::vector<std::pair<Shape, Shape>> FindAllCollisions(const std::span<const Shape>& shapes) {
    using Pair = std::pair<Shape, Shape>;
    std::vector<Pair> collisions;
    const auto n = shapes.size();
    collisions.reserve(n);

    auto indices = std::views::iota(0u, n);

    std::ranges::for_each(indices, [&](const unsigned i) {
        auto rest = std::views::iota(i + 1u, n);
        std::ranges::for_each(rest, [&](unsigned j) {
            if (queries::BoundingBoxesOverlap(shapes[i], shapes[j])) {
                collisions.emplace_back(shapes[i], shapes[j]);
            }
        });
    });
    collisions.shrink_to_fit();
    return collisions;
}

std::optional<size_t> FindHighestShape(const std::span<const Shape>& shapes) {
    if (shapes.empty()) {
        return std::nullopt;
    }

    return *std::ranges::max_element(std::views::iota(0u, shapes.size()),
        {}, [&](const size_t i) { return queries::GetHeight(shapes[i]); });
}

}  // namespace geometry::utils