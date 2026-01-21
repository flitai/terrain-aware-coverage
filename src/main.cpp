/**
 * radar_coverage_complete.cpp
 * 
 * 完整的雷达覆盖区域合并实现 - 使用真正的多边形布尔运算
 * 
 * 编译方法:
 *   1. 首先安装 Clipper2:
 *      git clone https://github.com/AngusJohnson/Clipper2.git
 *      cd Clipper2/CPP && mkdir build && cd build
 *      cmake .. && make && sudo make install
 * 
 *   2. 编译本程序:
 *      g++ -std=c++17 -O2 radar_coverage_complete.cpp -lClipper2 -o radar_demo
 * 
 *   3. 运行:
 *      ./radar_demo
 */

#include "polygon_boolean.hpp"
#include "radar_coverage.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

using namespace polygon_ops;

// ============================================================================
// 地形模型
// ============================================================================

struct TerrainObstacle {
    Point2D center;
    double rx, ry;   // 椭圆半径
    double height;   // 高度 (米)
    
    double getElevationAt(const Point2D& p) const {
        double dx = (p.x - center.x) / rx;
        double dy = (p.y - center.y) / ry;
        double distSq = dx * dx + dy * dy;
        
        if (distSq >= 1.0) return 0.0;
        return height * std::exp(-3.0 * distSq);
    }
};

class TerrainModel {
public:
    std::vector<TerrainObstacle> obstacles;
    
    double getElevation(double x, double y) const {
        double maxH = 0.0;
        for (const auto& obs : obstacles) {
            maxH = std::max(maxH, obs.getElevationAt({x, y}));
        }
        return maxH;
    }
    
    bool isLineOfSightBlocked(const Point2D& radar, double radarHeight,
                              const Point2D& target, double targetHeight = 0) const {
        const int steps = 40;
        double dx = (target.x - radar.x) / steps;
        double dy = (target.y - radar.y) / steps;
        double totalDist = (target - radar).length();
        
        for (int i = 1; i < steps; i++) {
            double x = radar.x + dx * i;
            double y = radar.y + dy * i;
            double progress = static_cast<double>(i) / steps;
            
            // 视线高度（线性插值）
            double losHeight = radarHeight * (1.0 - progress) + targetHeight * progress;
            
            // 地球曲率修正
            double curvatureDrop = (progress * totalDist) * (progress * totalDist) / 12740000.0 * 0.3;
            double effectiveLos = losHeight - curvatureDrop;
            
            // 地形高度
            double terrainH = getElevation(x, y);
            
            if (terrainH > effectiveLos) {
                return true;
            }
        }
        return false;
    }
};

// ============================================================================
// 雷达参数
// ============================================================================

struct RadarParams {
    int id;
    std::string name;
    Point2D position;
    double range;       // 最大探测距离
    double height;      // 天线高度
};

// ============================================================================
// 覆盖多边形生成
// ============================================================================

Polygon generateCoveragePolygon(const RadarParams& radar, 
                                 const TerrainModel& terrain,
                                 int numRays = 72) {
    Polygon poly;
    poly.reserve(numRays);
    
    double angleStep = 2.0 * M_PI / numRays;
    
    for (int i = 0; i < numRays; i++) {
        double angle = i * angleStep;
        
        // 二分搜索最大可视距离
        double lo = 0.0, hi = radar.range;
        while (hi - lo > radar.range * 0.02) {
            double mid = (lo + hi) / 2.0;
            Point2D target = {
                radar.position.x + std::cos(angle) * mid,
                radar.position.y + std::sin(angle) * mid
            };
            
            if (terrain.isLineOfSightBlocked(radar.position, radar.height, target)) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        
        poly.push_back({
            radar.position.x + std::cos(angle) * lo,
            radar.position.y + std::sin(angle) * lo
        });
    }
    
    return poly;
}

// ============================================================================
// SVG 导出
// ============================================================================

void exportToSVG(const std::string& filename,
                 const std::vector<RadarParams>& radars,
                 const std::vector<Polygon>& coverages,
                 const MultiPolygon& merged,
                 const TerrainModel& terrain,
                 int width = 800, int height = 600) {
    
    std::ofstream svg(filename);
    
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" height=\"" << height << "\" "
        << "viewBox=\"0 0 " << width << " " << height << "\">\n"
        << "  <defs>\n"
        << "    <linearGradient id=\"mergedGrad\" x1=\"0%\" y1=\"0%\" x2=\"100%\" y2=\"100%\">\n"
        << "      <stop offset=\"0%\" stop-color=\"#06b6d4\" stop-opacity=\"0.3\"/>\n"
        << "      <stop offset=\"100%\" stop-color=\"#8b5cf6\" stop-opacity=\"0.3\"/>\n"
        << "    </linearGradient>\n"
        << "  </defs>\n"
        << "  <rect width=\"100%\" height=\"100%\" fill=\"#0a0f1a\"/>\n";
    
    // 绘制地形
    for (const auto& obs : terrain.obstacles) {
        svg << "  <ellipse cx=\"" << obs.center.x << "\" cy=\"" << obs.center.y 
            << "\" rx=\"" << obs.rx << "\" ry=\"" << obs.ry 
            << "\" fill=\"#92400e\" fill-opacity=\"0.5\"/>\n";
    }
    
    // 绘制合并覆盖区域（包含孔洞）
    for (const auto& pwh : merged) {
        // 外边界
        svg << "  <polygon points=\"";
        for (size_t i = 0; i < pwh.outer.size(); i++) {
            if (i > 0) svg << " ";
            svg << std::fixed << std::setprecision(1) 
                << pwh.outer[i].x << "," << pwh.outer[i].y;
        }
        svg << "\" fill=\"url(#mergedGrad)\" stroke=\"#06b6d4\" stroke-width=\"2.5\"/>\n";
        
        // 孔洞（用背景色填充）
        for (const auto& hole : pwh.holes) {
            svg << "  <polygon points=\"";
            for (size_t i = 0; i < hole.size(); i++) {
                if (i > 0) svg << " ";
                svg << std::fixed << std::setprecision(1) 
                    << hole[i].x << "," << hole[i].y;
            }
            svg << "\" fill=\"#0a0f1a\" stroke=\"#ef4444\" stroke-width=\"1.5\" "
                << "stroke-dasharray=\"4,2\"/>\n";
        }
    }
    
    // 绘制独立覆盖区域
    const char* colors[] = {"#3b82f6", "#10b981", "#f59e0b", "#ef4444", "#8b5cf6"};
    for (size_t i = 0; i < coverages.size(); i++) {
        const auto& poly = coverages[i];
        const char* color = colors[i % 5];
        
        svg << "  <polygon points=\"";
        for (size_t j = 0; j < poly.size(); j++) {
            if (j > 0) svg << " ";
            svg << std::fixed << std::setprecision(1) 
                << poly[j].x << "," << poly[j].y;
        }
        svg << "\" fill=\"" << color << "\" fill-opacity=\"0.1\" "
            << "stroke=\"" << color << "\" stroke-opacity=\"0.5\" "
            << "stroke-width=\"1\" stroke-dasharray=\"4,2\"/>\n";
    }
    
    // 绘制雷达
    for (size_t i = 0; i < radars.size(); i++) {
        const auto& radar = radars[i];
        const char* color = colors[i % 5];
        
        svg << "  <circle cx=\"" << radar.position.x << "\" cy=\"" << radar.position.y 
            << "\" r=\"14\" fill=\"#0a0f1a\" stroke=\"" << color << "\" stroke-width=\"2\"/>\n";
        svg << "  <circle cx=\"" << radar.position.x << "\" cy=\"" << radar.position.y 
            << "\" r=\"5\" fill=\"" << color << "\"/>\n";
        svg << "  <text x=\"" << radar.position.x << "\" y=\"" << (radar.position.y - 20) 
            << "\" fill=\"" << color << "\" font-size=\"12\" text-anchor=\"middle\" "
            << "font-family=\"sans-serif\">" << radar.name << "</text>\n";
    }
    
    svg << "</svg>\n";
    svg.close();
    
    std::cout << "已导出: " << filename << std::endl;
}

// ============================================================================
// GeoJSON 导出（用于 GIS 系统）
// ============================================================================

void exportToGeoJSON(const std::string& filename, const MultiPolygon& merged) {
    std::ofstream json(filename);
    
    json << "{\n"
         << "  \"type\": \"FeatureCollection\",\n"
         << "  \"features\": [\n";
    
    for (size_t i = 0; i < merged.size(); i++) {
        const auto& pwh = merged[i];
        
        json << "    {\n"
             << "      \"type\": \"Feature\",\n"
             << "      \"properties\": {\n"
             << "        \"type\": \"radar_coverage\",\n"
             << "        \"region_id\": " << i << ",\n"
             << "        \"hole_count\": " << pwh.holes.size() << "\n"
             << "      },\n"
             << "      \"geometry\": {\n"
             << "        \"type\": \"Polygon\",\n"
             << "        \"coordinates\": [\n";
        
        // 外边界
        json << "          [";
        for (size_t j = 0; j < pwh.outer.size(); j++) {
            if (j > 0) json << ", ";
            json << "[" << pwh.outer[j].x << ", " << pwh.outer[j].y << "]";
        }
        // 闭合
        json << ", [" << pwh.outer[0].x << ", " << pwh.outer[0].y << "]]";
        
        // 孔洞
        for (const auto& hole : pwh.holes) {
            json << ",\n          [";
            for (size_t j = 0; j < hole.size(); j++) {
                if (j > 0) json << ", ";
                json << "[" << hole[j].x << ", " << hole[j].y << "]";
            }
            json << ", [" << hole[0].x << ", " << hole[0].y << "]]";
        }
        
        json << "\n        ]\n"
             << "      }\n"
             << "    }";
        
        if (i < merged.size() - 1) json << ",";
        json << "\n";
    }
    
    json << "  ]\n}\n";
    json.close();
    
    std::cout << "已导出: " << filename << std::endl;
}

// ============================================================================
// 主程序
// ============================================================================

int main() {
    std::cout << "======================================\n";
    std::cout << " 雷达覆盖区域合并 - 完整多边形布尔运算\n";
    std::cout << "======================================\n\n";
    
    // 1. 创建地形
    std::cout << "[1] 配置地形...\n";
    TerrainModel terrain;
    terrain.obstacles = {
        {{400, 280}, 100, 80, 800},   // 中央大山
        {{250, 400}, 50, 60, 400},    // 左下小山
        {{550, 420}, 60, 50, 450},    // 右下小山
    };
    std::cout << "    - 添加了 " << terrain.obstacles.size() << " 个地形障碍\n";
    
    // 2. 创建雷达
    std::cout << "[2] 配置雷达...\n";
    std::vector<RadarParams> radars = {
        {1, "雷达 A", {200, 200}, 180, 80},
        {2, "雷达 B", {600, 180}, 160, 100},
        {3, "雷达 C", {150, 400}, 140, 70},
        {4, "雷达 D", {650, 380}, 150, 90},
        {5, "雷达 E", {400, 500}, 170, 85},
    };
    std::cout << "    - 添加了 " << radars.size() << " 部雷达\n";
    
    // 3. 生成各雷达覆盖多边形
    std::cout << "[3] 生成覆盖多边形...\n";
    std::vector<Polygon> coverages;
    for (const auto& radar : radars) {
        Polygon coverage = generateCoveragePolygon(radar, terrain, 72);
        coverages.push_back(coverage);
        
        double area = PolygonUtils::area(coverage);
        std::cout << "    - " << radar.name << ": " << coverage.size() 
                  << " 顶点, 面积 = " << std::fixed << std::setprecision(0) 
                  << area << "\n";
    }
    
    // 4. 执行多边形并集运算
    std::cout << "[4] 执行多边形布尔运算 (并集)...\n";
    MultiPolygon merged = PolygonBoolean::unionAll(coverages);
    
    // 5. 后处理：简化和平滑
    std::cout << "[5] 后处理 (简化 + 平滑)...\n";
    merged = PolygonProcessor::simplifyAll(merged, 2.0);
    merged = PolygonProcessor::smoothAll(merged, 1);
    
    // 6. 输出统计信息
    std::cout << "\n[统计信息]\n";
    PolygonStats stats = PolygonStats::compute(merged);
    std::cout << "    - 分离区域数量: " << stats.regionCount << "\n";
    std::cout << "    - 总孔洞数量:   " << stats.totalHoleCount << "\n";
    std::cout << "    - 总覆盖面积:   " << std::fixed << std::setprecision(0) 
              << stats.totalArea << "\n";
    std::cout << "    - 总周长:       " << std::fixed << std::setprecision(0) 
              << stats.totalPerimeter << "\n";
    
    // 详细信息
    std::cout << "\n[区域详情]\n";
    for (size_t i = 0; i < merged.size(); i++) {
        const auto& pwh = merged[i];
        double outerArea = PolygonUtils::area(pwh.outer);
        std::cout << "    区域 " << (i + 1) << ": "
                  << pwh.outer.size() << " 顶点, "
                  << pwh.holes.size() << " 孔洞, "
                  << "面积 = " << std::fixed << std::setprecision(0) << outerArea << "\n";
    }
    
    // 7. 导出文件
    std::cout << "\n[7] 导出文件...\n";
    exportToSVG("radar_coverage_result.svg", radars, coverages, merged, terrain);
    exportToGeoJSON("radar_coverage_result.geojson", merged);
    
    std::cout << "\n======================================\n";
    std::cout << " 完成!\n";
    std::cout << "======================================\n";
    
    return 0;
}
