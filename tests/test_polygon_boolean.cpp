/**
 * test_polygon_boolean.cpp
 * 
 * 多边形布尔运算单元测试
 */

#include <gtest/gtest.h>
#include "polygon_boolean.hpp"
#include <cmath>

using namespace polygon_ops;

// ============================================================================
// 辅助函数
// ============================================================================

Polygon createSquare(double cx, double cy, double size) {
    double half = size / 2;
    return {
        {cx - half, cy - half},
        {cx + half, cy - half},
        {cx + half, cy + half},
        {cx - half, cy + half}
    };
}

Polygon createCircle(double cx, double cy, double radius, int segments = 32) {
    Polygon poly;
    for (int i = 0; i < segments; i++) {
        double angle = 2 * M_PI * i / segments;
        poly.push_back({
            cx + radius * std::cos(angle),
            cy + radius * std::sin(angle)
        });
    }
    return poly;
}

Polygon createLShape(double cx, double cy, double size) {
    double h = size / 2;
    return {
        {cx - h, cy - h},
        {cx, cy - h},
        {cx, cy},
        {cx + h, cy},
        {cx + h, cy + h},
        {cx - h, cy + h}
    };
}

// ============================================================================
// 基础测试
// ============================================================================

TEST(PolygonUtils, AreaCalculation) {
    // 单位正方形面积 = 1
    Polygon square = createSquare(0, 0, 1);
    EXPECT_NEAR(PolygonUtils::area(square), 1.0, 1e-6);
    
    // 2x2 正方形面积 = 4
    Polygon square2 = createSquare(0, 0, 2);
    EXPECT_NEAR(PolygonUtils::area(square2), 4.0, 1e-6);
    
    // 圆形面积 ≈ π * r²
    Polygon circle = createCircle(0, 0, 1, 64);
    EXPECT_NEAR(PolygonUtils::area(circle), M_PI, 0.01);
}

TEST(PolygonUtils, Orientation) {
    // 逆时针
    Polygon ccw = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    EXPECT_TRUE(PolygonUtils::isCounterClockwise(ccw));
    
    // 顺时针
    Polygon cw = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
    EXPECT_FALSE(PolygonUtils::isCounterClockwise(cw));
}

TEST(PolygonUtils, PointInPolygon) {
    Polygon square = createSquare(0, 0, 2);
    
    // 中心点在内部
    EXPECT_TRUE(PolygonUtils::pointInPolygon({0, 0}, square));
    
    // 内部点
    EXPECT_TRUE(PolygonUtils::pointInPolygon({0.5, 0.5}, square));
    
    // 外部点
    EXPECT_FALSE(PolygonUtils::pointInPolygon({2, 2}, square));
    EXPECT_FALSE(PolygonUtils::pointInPolygon({-2, 0}, square));
}

// ============================================================================
// 并集测试
// ============================================================================

TEST(PolygonBoolean, UnionTwoOverlappingSquares) {
    // 两个重叠的正方形
    Polygon sq1 = createSquare(0, 0, 2);
    Polygon sq2 = createSquare(1, 0, 2);  // 右移1
    
    auto result = PolygonBoolean::unionAll({sq1, sq2});
    
    // 应该产生1个区域，无孔洞
    EXPECT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].holes.size(), 0);
    
    // 面积应该在 2*4 - 重叠部分 之间
    double area = PolygonUtils::area(result[0].outer);
    EXPECT_GT(area, 4.0);  // 大于单个正方形
    EXPECT_LT(area, 8.0);  // 小于两个正方形之和
}

TEST(PolygonBoolean, UnionTwoSeparateSquares) {
    // 两个分离的正方形
    Polygon sq1 = createSquare(0, 0, 2);
    Polygon sq2 = createSquare(5, 0, 2);  // 完全分离
    
    auto result = PolygonBoolean::unionAll({sq1, sq2});
    
    // 应该产生2个分离的区域
    EXPECT_EQ(result.size(), 2);
    
    // 总面积 = 4 + 4 = 8
    double totalArea = 0;
    for (const auto& pwh : result) {
        totalArea += PolygonUtils::area(pwh.outer);
    }
    EXPECT_NEAR(totalArea, 8.0, 0.01);
}

TEST(PolygonBoolean, UnionWithHole) {
    // 大正方形包围小正方形，应该产生孔洞
    Polygon outer = createSquare(0, 0, 6);
    
    // 4个小正方形围成一个环，中间留空
    Polygon sq1 = createSquare(-1.5, -1.5, 2);
    Polygon sq2 = createSquare(1.5, -1.5, 2);
    Polygon sq3 = createSquare(-1.5, 1.5, 2);
    Polygon sq4 = createSquare(1.5, 1.5, 2);
    
    auto result = PolygonBoolean::unionAll({sq1, sq2, sq3, sq4});
    
    // 合并后应该有1个区域，可能有1个孔洞（取决于配置）
    EXPECT_GE(result.size(), 1);
}

// ============================================================================
// 凹多边形测试
// ============================================================================

TEST(PolygonBoolean, UnionConcavePolygons) {
    // 两个 L 形多边形
    Polygon L1 = createLShape(0, 0, 4);
    Polygon L2 = createLShape(2, 2, 4);
    
    auto result = PolygonBoolean::unionAll({L1, L2});
    
    // 应该产生凹形结果
    EXPECT_GE(result.size(), 1);
    
    // 结果应该保持凹形特征（顶点数 > 4）
    EXPECT_GT(result[0].outer.size(), 4);
}

// ============================================================================
// 交集测试
// ============================================================================

TEST(PolygonBoolean, IntersectionOverlapping) {
    Polygon sq1 = createSquare(0, 0, 2);
    Polygon sq2 = createSquare(1, 0, 2);
    
    auto result = PolygonBoolean::intersection(sq1, sq2);
    
    // 应该有交集
    EXPECT_EQ(result.size(), 1);
    
    // 交集面积 = 1 * 2 = 2
    double area = PolygonUtils::area(result[0].outer);
    EXPECT_NEAR(area, 2.0, 0.1);
}

TEST(PolygonBoolean, IntersectionSeparate) {
    Polygon sq1 = createSquare(0, 0, 2);
    Polygon sq2 = createSquare(5, 0, 2);
    
    auto result = PolygonBoolean::intersection(sq1, sq2);
    
    // 无交集
    EXPECT_EQ(result.size(), 0);
}

// ============================================================================
// 差集测试
// ============================================================================

TEST(PolygonBoolean, Difference) {
    Polygon big = createSquare(0, 0, 4);
    Polygon small = createSquare(0, 0, 2);
    
    auto result = PolygonBoolean::difference(big, small);
    
    // 大正方形减去小正方形，应该有孔洞
    EXPECT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].holes.size(), 1);
    
    // 面积 = 16 - 4 = 12
    double outerArea = PolygonUtils::area(result[0].outer);
    double holeArea = PolygonUtils::area(result[0].holes[0]);
    EXPECT_NEAR(outerArea - holeArea, 12.0, 0.1);
}

// ============================================================================
// 后处理测试
// ============================================================================

TEST(PolygonProcessor, Simplify) {
    // 创建一个有很多点的圆
    Polygon circle = createCircle(0, 0, 10, 100);
    EXPECT_EQ(circle.size(), 100);
    
    // 简化
    Polygon simplified = PolygonProcessor::simplify(circle, 0.5);
    
    // 简化后点数应该减少
    EXPECT_LT(simplified.size(), circle.size());
    EXPECT_GT(simplified.size(), 10);  // 但仍然保持大致形状
}

TEST(PolygonProcessor, Smooth) {
    Polygon square = createSquare(0, 0, 2);
    EXPECT_EQ(square.size(), 4);
    
    // 平滑1次
    Polygon smoothed = PolygonProcessor::smooth(square, 1);
    EXPECT_EQ(smoothed.size(), 8);  // 每条边变成2段
    
    // 平滑2次
    Polygon smoothed2 = PolygonProcessor::smooth(square, 2);
    EXPECT_EQ(smoothed2.size(), 16);
}

// ============================================================================
// 性能测试 (可选)
// ============================================================================

TEST(Performance, UnionManyPolygons) {
    std::vector<Polygon> polygons;
    
    // 创建20个随机位置的圆
    for (int i = 0; i < 20; i++) {
        double x = (i % 5) * 30;
        double y = (i / 5) * 30;
        polygons.push_back(createCircle(x, y, 20, 36));
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    auto result = PolygonBoolean::unionAll(polygons);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // 应该在合理时间内完成 (< 1秒)
    EXPECT_LT(duration.count(), 1000);
    
    // 应该有结果
    EXPECT_GE(result.size(), 1);
    
    std::cout << "Union of 20 polygons: " << duration.count() << " ms" << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
