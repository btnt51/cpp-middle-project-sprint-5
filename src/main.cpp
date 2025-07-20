#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <print>
#include <ranges>

using namespace geometry;

namespace rng = std::ranges;
namespace views = std::ranges::views;

void PrintAllIntersections(const Shape &shape, const std::span<Shape>& shapes) {
    std::println("\n=== Intersections ===");

    rng::for_each(shapes, [&](const Shape& other) {
        if (&shape == &other) return;
        geometry::intersections::GetIntersectPoint(shape, other).and_then([&](const auto& point) {
            std::visit(
                    [&point](const auto &shape, const auto &other) {
                        std::println("Пересечение найдено в точке {} между фигурами {} и {}", point, shape, other);
                    }, shape, other);
            return std::optional<std::monostate>{std::in_place};
        })
        .or_else([&] {
            std::visit(
            [](const auto &shape, const auto &other) {
                std::println("Фигуры {} и {} не пересекаются", shape, other);
            },
            shape, other);
            return std::optional<std::monostate>{std::in_place};
        });
    });
}

void PrintDistancesFromPointToShapes(Point2D point, const std::span<const Shape>& shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("Testing point: {} ", point);

    rng::for_each(shapes | std::views::take(5), [&](const Shape& shape) {
        const auto distance = geometry::queries::DistanceToPoint(shape, point);
        std::visit(
            [&point, &distance](const auto &shape) {
                std::println("Расстояние от точки {} до фигуры {} равно {}", point, shape, distance);
            },
            shape);
    });
}

void PerformShapeAnalysis(const std::span<const Shape>& shapes) {
    std::println("\n=== Shape Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Найти все пересечения между фигурами используя метод Bounding Box
     *     - Найти самую высокую фигуру (чья высота наибольшая)
     *     - Вывести расстояние между любыми двумя фигурами, которые поддерживают данную функциональность
     */

    rng::for_each(utils::FindAllCollisions(shapes), [](const std::pair<Shape, Shape>& pair) {
        std::visit(
            [](const auto &shape, const auto &other) {
                std::println("BoundingBox пересечение между {} и {}", shape, other);
            },
            pair.first, pair.second);
    });

    if (auto highest = utils::FindHighestShape(shapes)) {
        std::println("Фигура с наибольшей высотой: индекс {}, высота {}", *highest, queries::GetHeight(shapes[*highest]));
    }
    auto index_pairs = views::cartesian_product(
                           views::iota(0u, shapes.size()),
                           views::iota(0u, shapes.size()))
                     | views::filter([](auto tup) {
                           auto [i, j] = tup;
                           return i < j;
                       });

    for (auto [i, j] : index_pairs) {
            queries::DistanceBetweenShapes(shapes[i], shapes[j]).and_then([&](const auto& distance) {
            std::visit(
                    [&distance](const auto &shape, const auto &other) {
                        std::println("Расстояние между фигурами {} и {} равно {}", shape, other, distance);
                    },
                    shapes[i], shapes[j]);
                return std::optional<std::monostate>{std::in_place};
        });
    }
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");

    auto above_50 = shapes | std::views::filter([](const Shape& s) {
        return queries::GetHeight(s) > 50.0;
    }) | std::views::take(3);

    rng::for_each(above_50, [](const Shape& s) {
        std::visit([](const auto &shape) {std::println("Фигура выше 50.0: {}", shape);},s);
    });

    if (!shapes.empty()) {
        auto [min_it, max_it] = rng::minmax_element(shapes, {}, queries::GetHeight);

        std::visit([](const auto &min_shape_hieght, const auto &max_shape_height) {
            std::println("Фигура с минимальной высотой: {}", min_shape_hieght);
            std::println("Фигура с максимальной высотой: {}", max_shape_height);
        }, *min_it, *max_it);
    }
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    std::vector<Shape> shapes = generator.GenerateShapes(15);

    std::println("Generated {} random shapes", shapes.size());

    // Выведите индекс каждой фигуры и её высоту
    auto indexes = views::iota(0u, static_cast<unsigned>(shapes.size()));
    rng::for_each(indexes | views::transform([&](unsigned i) {
            return std::pair{i, queries::GetHeight(shapes[i])};
        }),
        [](const auto& indexed_height) {
            std::println("Фигура {} имеет высоту {}", indexed_height.first, indexed_height.second);
        }
    );

    PrintAllIntersections(shapes[0], shapes);

    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);

    PerformShapeAnalysis(shapes);

    PerformExtraShapeAnalysis(shapes);

    //
    // Рисуем все фигуры
    //
    // Важно: после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 2ой график
    //
    geometry::visualization::Draw(shapes);

    std::vector<Point2D> points;
    //
    // Формируем список из вершин всех фигур
    //
    for (const auto& shape : shapes) {
        std::visit([&](const auto& s) {
            const auto& vertices = s.Vertices();
            points.insert(points.end(), vertices.begin(), vertices.end());
        }, shape);
    }

    // Строим выпуклую оболочку через GrahamScan
    if (auto hull_result = convex_hull::GrahamScan(points); hull_result) {
        shapes.emplace_back(Polygon{ *hull_result });
        geometry::visualization::Draw(shapes);
    }

    // Демонстрация Делоне-триангуляции
    {
        std::vector<Point2D> test_points = {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};

        if (const auto result = triangulation::DelaunayTriangulation(test_points); result) {
            std::vector<Shape> delaunay_triangles;
            for (const auto& tri : *result) {
                delaunay_triangles.emplace_back(Polygon{ tri.vertices() });
            }
            geometry::visualization::Draw(delaunay_triangles);
        }
    }
    return 0;
}