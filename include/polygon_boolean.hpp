/**
 * polygon_boolean.hpp
 * 
 * 真正的多边形布尔运算实现 - 基于 Clipper2 库
 * 完整支持凹多边形、多连通域、孔洞检测
 * 
 * 依赖: Clipper2 (https://github.com/AngusJohnson/Clipper2)
 * 
 * 安装 Clipper2:
 *   git clone https://github.com/AngusJohnson/Clipper2.git
 *   cd Clipper2/CPP
 *   mkdir build && cd build
 *   cmake ..
 *   make && sudo make install
 * 
 * 编译本文件:
 *   g++ -std=c++17 -O2 your_program.cpp -lClipper2 -o your_program
 */

#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

// Clipper2 头文件
#include "clipper2/clipper.h"

namespace polygon_ops {

// ============================================================================
// 基础数据结构
// ============================================================================

struct Point2D {
    double x, y;
    
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    Point2D operator+(const Point2D& o) const { return {x + o.x, y + o.y}; }
    Point2D operator-(const Point2D& o) const { return {x - o.x, y - o.y}; }
    Point2D operator*(double s) const { return {x * s, y * s}; }
    
    double dot(const Point2D& o) const { return x * o.x + y * o.y; }
    double cross(const Point2D& o) const { return x * o.y - y * o.x; }
    double length() const { return std::sqrt(x * x + y * y); }
};

// 简单多边形（无孔洞）
using Polygon = std::vector<Point2D>;

// 带孔洞的多边形
struct PolygonWithHoles {
    Polygon outer;              // 外边界（逆时针）
    std::vector<Polygon> holes; // 孔洞列表（顺时针）
};

// 多区域结果（可能包含多个分离的多边形）
using MultiPolygon = std::vector<PolygonWithHoles>;

// ============================================================================
// Clipper2 类型别名
// ============================================================================

using ClipperPath = Clipper2Lib::PathD;
using ClipperPaths = Clipper2Lib::PathsD;
using ClipperPoint = Clipper2Lib::PointD;

// ============================================================================
// 坐标转换
// ============================================================================

class CoordinateConverter {
public:
    // 我们的类型 -> Clipper2 类型
    static ClipperPath toClipperPath(const Polygon& poly) {
        ClipperPath path;
        path.reserve(poly.size());
        for (const auto& p : poly) {
            path.push_back(ClipperPoint(p.x, p.y));
        }
        return path;
    }
    
    static ClipperPaths toClipperPaths(const std::vector<Polygon>& polygons) {
        ClipperPaths paths;
        paths.reserve(polygons.size());
        for (const auto& poly : polygons) {
            paths.push_back(toClipperPath(poly));
        }
        return paths;
    }
    
    // Clipper2 类型 -> 我们的类型
    static Polygon fromClipperPath(const ClipperPath& path) {
        Polygon poly;
        poly.reserve(path.size());
        for (const auto& p : path) {
            poly.push_back(Point2D(p.x, p.y));
        }
        return poly;
    }
    
    static std::vector<Polygon> fromClipperPaths(const ClipperPaths& paths) {
        std::vector<Polygon> polygons;
        polygons.reserve(paths.size());
        for (const auto& path : paths) {
            polygons.push_back(fromClipperPath(path));
        }
        return polygons;
    }
};

// ============================================================================
// 多边形工具函数
// ============================================================================

class PolygonUtils {
public:
    /**
     * 计算多边形面积（带符号）
     * 正值 = 逆时针（外边界）
     * 负值 = 顺时针（孔洞）
     */
    static double signedArea(const Polygon& poly) {
        if (poly.size() < 3) return 0.0;
        
        double area = 0.0;
        size_t n = poly.size();
        for (size_t i = 0; i < n; i++) {
            size_t j = (i + 1) % n;
            area += poly[i].x * poly[j].y;
            area -= poly[j].x * poly[i].y;
        }
        return area / 2.0;
    }
    
    static double area(const Polygon& poly) {
        return std::abs(signedArea(poly));
    }
    
    /**
     * 计算多边形周长
     */
    static double perimeter(const Polygon& poly) {
        if (poly.size() < 2) return 0.0;
        
        double perim = 0.0;
        size_t n = poly.size();
        for (size_t i = 0; i < n; i++) {
            size_t j = (i + 1) % n;
            perim += (poly[j] - poly[i]).length();
        }
        return perim;
    }
    
    /**
     * 判断多边形方向
     */
    static bool isCounterClockwise(const Polygon& poly) {
        return signedArea(poly) > 0;
    }
    
    /**
     * 确保多边形为指定方向
     */
    static Polygon ensureOrientation(const Polygon& poly, bool ccw) {
        bool isCCW = isCounterClockwise(poly);
        if (isCCW == ccw) {
            return poly;
        }
        Polygon reversed = poly;
        std::reverse(reversed.begin(), reversed.end());
        return reversed;
    }
    
    /**
     * 点是否在多边形内（射线法）
     */
    static bool pointInPolygon(const Point2D& pt, const Polygon& poly) {
        bool inside = false;
        size_t n = poly.size();
        
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            if (((poly[i].y > pt.y) != (poly[j].y > pt.y)) &&
                (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / 
                        (poly[j].y - poly[i].y) + poly[i].x)) {
                inside = !inside;
            }
        }
        
        return inside;
    }
    
    /**
     * 计算多边形边界框
     */
    struct BoundingBox {
        double minX, minY, maxX, maxY;
        
        double width() const { return maxX - minX; }
        double height() const { return maxY - minY; }
        Point2D center() const { return {(minX + maxX) / 2, (minY + maxY) / 2}; }
    };
    
    static BoundingBox boundingBox(const Polygon& poly) {
        BoundingBox bbox = {
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest()
        };
        
        for (const auto& p : poly) {
            bbox.minX = std::min(bbox.minX, p.x);
            bbox.minY = std::min(bbox.minY, p.y);
            bbox.maxX = std::max(bbox.maxX, p.x);
            bbox.maxY = std::max(bbox.maxY, p.y);
        }
        
        return bbox;
    }
};

// ============================================================================
// 多边形布尔运算（核心类）
// ============================================================================

class PolygonBoolean {
public:
    /**
     * 计算多个多边形的并集
     * 
     * @param polygons 输入多边形列表
     * @return 合并后的多区域结果（可能包含多个分离区域和孔洞）
     */
    static MultiPolygon unionAll(const std::vector<Polygon>& polygons) {
        if (polygons.empty()) return {};
        if (polygons.size() == 1) {
            PolygonWithHoles pwh;
            pwh.outer = PolygonUtils::ensureOrientation(polygons[0], true);
            return {pwh};
        }
        
        // 转换为 Clipper2 格式
        ClipperPaths subjects = CoordinateConverter::toClipperPaths(polygons);
        
        // 执行并集运算
        ClipperPaths solution;
        Clipper2Lib::ClipperD clipper;
        clipper.AddSubject(subjects);
        clipper.Execute(Clipper2Lib::ClipType::Union, 
                       Clipper2Lib::FillRule::NonZero, 
                       solution);
        
        // 转换结果并分类外边界/孔洞
        return classifyResult(solution);
    }
    
    /**
     * 计算两个多边形的并集
     */
    static MultiPolygon unionTwo(const Polygon& a, const Polygon& b) {
        return unionAll({a, b});
    }
    
    /**
     * 计算两个多边形的交集
     */
    static MultiPolygon intersection(const Polygon& a, const Polygon& b) {
        ClipperPaths subjects, clips;
        subjects.push_back(CoordinateConverter::toClipperPath(a));
        clips.push_back(CoordinateConverter::toClipperPath(b));
        
        ClipperPaths solution;
        Clipper2Lib::ClipperD clipper;
        clipper.AddSubject(subjects);
        clipper.AddClip(clips);
        clipper.Execute(Clipper2Lib::ClipType::Intersection,
                       Clipper2Lib::FillRule::NonZero,
                       solution);
        
        return classifyResult(solution);
    }
    
    /**
     * 计算多边形差集 (a - b)
     */
    static MultiPolygon difference(const Polygon& a, const Polygon& b) {
        ClipperPaths subjects, clips;
        subjects.push_back(CoordinateConverter::toClipperPath(a));
        clips.push_back(CoordinateConverter::toClipperPath(b));
        
        ClipperPaths solution;
        Clipper2Lib::ClipperD clipper;
        clipper.AddSubject(subjects);
        clipper.AddClip(clips);
        clipper.Execute(Clipper2Lib::ClipType::Difference,
                       Clipper2Lib::FillRule::NonZero,
                       solution);
        
        return classifyResult(solution);
    }
    
    /**
     * 计算多边形异或（对称差）
     */
    static MultiPolygon xorOp(const Polygon& a, const Polygon& b) {
        ClipperPaths subjects, clips;
        subjects.push_back(CoordinateConverter::toClipperPath(a));
        clips.push_back(CoordinateConverter::toClipperPath(b));
        
        ClipperPaths solution;
        Clipper2Lib::ClipperD clipper;
        clipper.AddSubject(subjects);
        clipper.AddClip(clips);
        clipper.Execute(Clipper2Lib::ClipType::Xor,
                       Clipper2Lib::FillRule::NonZero,
                       solution);
        
        return classifyResult(solution);
    }
    
    /**
     * 多边形偏移（膨胀/收缩）
     * 
     * @param poly 输入多边形
     * @param delta 偏移量（正值膨胀，负值收缩）
     * @param joinType 连接类型：Round, Square, Miter
     */
    static MultiPolygon offset(const Polygon& poly, double delta,
                               Clipper2Lib::JoinType joinType = Clipper2Lib::JoinType::Round) {
        ClipperPaths input;
        input.push_back(CoordinateConverter::toClipperPath(poly));
        
        ClipperPaths solution;
        solution = Clipper2Lib::InflatePaths(input, delta, joinType, 
                                             Clipper2Lib::EndType::Polygon);
        
        return classifyResult(solution);
    }

private:
    /**
     * 将 Clipper2 结果分类为外边界和孔洞
     */
    static MultiPolygon classifyResult(const ClipperPaths& paths) {
        if (paths.empty()) return {};
        
        // 转换所有路径
        std::vector<Polygon> allPolygons;
        std::vector<double> areas;
        
        for (const auto& path : paths) {
            Polygon poly = CoordinateConverter::fromClipperPath(path);
            double area = PolygonUtils::signedArea(poly);
            allPolygons.push_back(poly);
            areas.push_back(area);
        }
        
        // 分类：正面积为外边界，负面积为孔洞
        std::vector<size_t> outerIndices, holeIndices;
        for (size_t i = 0; i < areas.size(); i++) {
            if (areas[i] > 0) {
                outerIndices.push_back(i);
            } else if (areas[i] < 0) {
                holeIndices.push_back(i);
            }
        }
        
        // 构建结果：将每个孔洞分配给包含它的外边界
        MultiPolygon result;
        
        for (size_t outerIdx : outerIndices) {
            PolygonWithHoles pwh;
            pwh.outer = allPolygons[outerIdx];
            
            // 找到属于这个外边界的孔洞
            for (size_t holeIdx : holeIndices) {
                // 检查孔洞的某个点是否在外边界内
                if (!allPolygons[holeIdx].empty()) {
                    Point2D testPoint = allPolygons[holeIdx][0];
                    if (PolygonUtils::pointInPolygon(testPoint, pwh.outer)) {
                        // 孔洞需要反转为逆时针以便后续处理
                        Polygon hole = allPolygons[holeIdx];
                        std::reverse(hole.begin(), hole.end());
                        pwh.holes.push_back(hole);
                    }
                }
            }
            
            result.push_back(pwh);
        }
        
        return result;
    }
};

// ============================================================================
// 多边形简化与平滑
// ============================================================================

class PolygonProcessor {
public:
    /**
     * Douglas-Peucker 简化算法
     */
    static Polygon simplify(const Polygon& poly, double epsilon) {
        if (poly.size() < 3 || epsilon <= 0) return poly;
        
        // 使用 Clipper2 的简化
        ClipperPath path = CoordinateConverter::toClipperPath(poly);
        ClipperPaths simplified = Clipper2Lib::SimplifyPaths({path}, epsilon);
        
        if (simplified.empty() || simplified[0].size() < 3) {
            return poly;
        }
        
        return CoordinateConverter::fromClipperPath(simplified[0]);
    }
    
    /**
     * Chaikin 平滑算法
     */
    static Polygon smooth(const Polygon& poly, int iterations = 2) {
        if (poly.size() < 3 || iterations <= 0) return poly;
        
        Polygon result = poly;
        
        for (int iter = 0; iter < iterations; iter++) {
            Polygon smoothed;
            size_t n = result.size();
            smoothed.reserve(n * 2);
            
            for (size_t i = 0; i < n; i++) {
                const Point2D& p0 = result[i];
                const Point2D& p1 = result[(i + 1) % n];
                
                smoothed.push_back(p0 * 0.75 + p1 * 0.25);
                smoothed.push_back(p0 * 0.25 + p1 * 0.75);
            }
            
            result = smoothed;
        }
        
        return result;
    }
    
    /**
     * 处理整个 MultiPolygon
     */
    static MultiPolygon simplifyAll(const MultiPolygon& mp, double epsilon) {
        MultiPolygon result;
        for (const auto& pwh : mp) {
            PolygonWithHoles simplified;
            simplified.outer = simplify(pwh.outer, epsilon);
            for (const auto& hole : pwh.holes) {
                simplified.holes.push_back(simplify(hole, epsilon));
            }
            result.push_back(simplified);
        }
        return result;
    }
    
    static MultiPolygon smoothAll(const MultiPolygon& mp, int iterations) {
        MultiPolygon result;
        for (const auto& pwh : mp) {
            PolygonWithHoles smoothed;
            smoothed.outer = smooth(pwh.outer, iterations);
            for (const auto& hole : pwh.holes) {
                smoothed.holes.push_back(smooth(hole, iterations));
            }
            result.push_back(smoothed);
        }
        return result;
    }
};

// ============================================================================
// 统计信息
// ============================================================================

struct PolygonStats {
    size_t regionCount;      // 分离区域数量
    size_t totalHoleCount;   // 总孔洞数量
    double totalArea;        // 总面积（减去孔洞）
    double totalPerimeter;   // 总周长
    
    static PolygonStats compute(const MultiPolygon& mp) {
        PolygonStats stats = {0, 0, 0.0, 0.0};
        
        stats.regionCount = mp.size();
        
        for (const auto& pwh : mp) {
            double outerArea = PolygonUtils::area(pwh.outer);
            double holeArea = 0.0;
            
            for (const auto& hole : pwh.holes) {
                holeArea += PolygonUtils::area(hole);
                stats.totalPerimeter += PolygonUtils::perimeter(hole);
            }
            
            stats.totalHoleCount += pwh.holes.size();
            stats.totalArea += outerArea - holeArea;
            stats.totalPerimeter += PolygonUtils::perimeter(pwh.outer);
        }
        
        return stats;
    }
};

} // namespace polygon_ops
