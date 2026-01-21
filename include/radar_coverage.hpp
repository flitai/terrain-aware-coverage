/**
 * radar_coverage.hpp
 * 
 * 雷达覆盖区域计算模块
 * 包含地形模型、视线遮挡计算、覆盖多边形生成
 * 
 * 依赖: polygon_boolean.hpp
 */

#pragma once

#include "polygon_boolean.hpp"
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <functional>

namespace radar_coverage {

// 使用 polygon_ops 命名空间中的类型
using polygon_ops::Point2D;
using polygon_ops::Polygon;
using polygon_ops::MultiPolygon;
using polygon_ops::PolygonWithHoles;
using polygon_ops::PolygonBoolean;
using polygon_ops::PolygonUtils;
using polygon_ops::PolygonProcessor;
using polygon_ops::PolygonStats;

// ============================================================================
// 地形障碍物
// ============================================================================

struct TerrainObstacle {
    Point2D center;         // 中心位置
    double rx, ry;          // 椭圆半径
    double height;          // 峰值高度 (米)
    std::string name;       // 名称 (可选)
    
    TerrainObstacle() : rx(50), ry(50), height(500) {}
    
    TerrainObstacle(Point2D c, double r_x, double r_y, double h, 
                    const std::string& n = "")
        : center(c), rx(r_x), ry(r_y), height(h), name(n) {}
    
    /**
     * 获取某点的地形高度贡献（高斯山峰模型）
     */
    double getElevationAt(const Point2D& p) const {
        double dx = (p.x - center.x) / rx;
        double dy = (p.y - center.y) / ry;
        double distSq = dx * dx + dy * dy;
        
        if (distSq >= 1.0) return 0.0;
        
        return height * std::exp(-3.0 * distSq);
    }
};

// ============================================================================
// 地形模型
// ============================================================================

class TerrainModel {
public:
    using ElevationFunction = std::function<double(double, double)>;
    
    TerrainModel() : earth_radius_(6371000.0) {}
    
    void addObstacle(const TerrainObstacle& obs) {
        obstacles_.push_back(obs);
    }
    
    void addObstacle(Point2D center, double rx, double ry, double height) {
        obstacles_.push_back({center, rx, ry, height});
    }
    
    void clearObstacles() {
        obstacles_.clear();
    }
    
    void setElevationFunction(ElevationFunction func) {
        custom_elevation_ = func;
    }
    
    double getElevation(double x, double y) const {
        double h = 0.0;
        
        if (custom_elevation_) {
            h = custom_elevation_(x, y);
        }
        
        for (const auto& obs : obstacles_) {
            h = std::max(h, obs.getElevationAt({x, y}));
        }
        
        return h;
    }
    
    bool isLineOfSightBlocked(
        const Point2D& radarPos,
        double radarHeight,
        const Point2D& targetPos,
        double targetHeight = 0.0,
        int numSamples = 40
    ) const {
        Point2D delta = targetPos - radarPos;
        double totalDist = delta.length();
        
        if (totalDist < 1e-6) return false;
        
        Point2D step = delta * (1.0 / numSamples);
        
        for (int i = 1; i < numSamples; i++) {
            Point2D samplePos = radarPos + step * i;
            double progress = static_cast<double>(i) / numSamples;
            
            double losHeight = radarHeight * (1.0 - progress) + 
                              targetHeight * progress;
            
            double arcDist = progress * totalDist;
            double curvatureDrop = (arcDist * arcDist) / (2.0 * earth_radius_);
            
            double effectiveLos = losHeight - curvatureDrop * 0.5;
            double terrainH = getElevation(samplePos.x, samplePos.y);
            
            if (terrainH > effectiveLos) {
                return true;
            }
        }
        
        return false;
    }
    
    double computeMaxVisibleRange(
        const Point2D& radarPos,
        double radarHeight,
        double azimuth,
        double maxRange,
        double targetHeight = 0.0
    ) const {
        Point2D dir(std::cos(azimuth), std::sin(azimuth));
        
        double lo = 0.0, hi = maxRange;
        
        while (hi - lo > maxRange * 0.01) {
            double mid = (lo + hi) / 2.0;
            Point2D target = radarPos + dir * mid;
            
            if (isLineOfSightBlocked(radarPos, radarHeight, target, targetHeight)) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        
        return lo;
    }
    
    const std::vector<TerrainObstacle>& getObstacles() const {
        return obstacles_;
    }

private:
    std::vector<TerrainObstacle> obstacles_;
    ElevationFunction custom_elevation_;
    double earth_radius_;
};

// ============================================================================
// 雷达参数
// ============================================================================

struct RadarParams {
    int id;
    std::string name;
    Point2D position;
    double range;
    double height;
    double minElevation;
    double maxElevation;
    double azimuthStart;
    double azimuthEnd;
    
    RadarParams() 
        : id(0), name("Radar"), range(50000), height(10),
          minElevation(-0.01), maxElevation(0.7),
          azimuthStart(0), azimuthEnd(2 * M_PI) {}
    
    RadarParams(int id_, const std::string& name_, Point2D pos, 
                double range_, double height_)
        : id(id_), name(name_), position(pos), range(range_), height(height_),
          minElevation(-0.01), maxElevation(0.7),
          azimuthStart(0), azimuthEnd(2 * M_PI) {}
    
    bool isOmnidirectional() const {
        return std::abs(azimuthEnd - azimuthStart - 2 * M_PI) < 0.01;
    }
};

// ============================================================================
// 覆盖多边形生成
// ============================================================================

inline Polygon generateCoveragePolygon(
    const RadarParams& radar, 
    const TerrainModel& terrain,
    int numRays = 72
) {
    Polygon polygon;
    polygon.reserve(numRays);
    
    double azimuthSpan = radar.azimuthEnd - radar.azimuthStart;
    double azimuthStep = azimuthSpan / numRays;
    
    for (int i = 0; i < numRays; i++) {
        double azimuth = radar.azimuthStart + i * azimuthStep;
        
        double range = terrain.computeMaxVisibleRange(
            radar.position,
            radar.height,
            azimuth,
            radar.range
        );
        
        Point2D vertex(
            radar.position.x + range * std::cos(azimuth),
            radar.position.y + range * std::sin(azimuth)
        );
        
        polygon.push_back(vertex);
    }
    
    if (!radar.isOmnidirectional()) {
        polygon.push_back(radar.position);
    }
    
    return polygon;
}

// ============================================================================
// 覆盖合并管理器
// ============================================================================

class CoverageMergeManager {
public:
    CoverageMergeManager() 
        : numRays_(72), simplifyEpsilon_(5.0), smoothIterations_(1) {}
    
    TerrainModel& terrain() { return terrain_; }
    const TerrainModel& terrain() const { return terrain_; }
    
    void addRadar(const RadarParams& radar) {
        radars_.push_back(radar);
        dirty_ = true;
    }
    
    void updateRadar(int id, const RadarParams& params) {
        for (auto& r : radars_) {
            if (r.id == id) {
                r = params;
                dirty_ = true;
                break;
            }
        }
    }
    
    void removeRadar(int id) {
        radars_.erase(
            std::remove_if(radars_.begin(), radars_.end(),
                [id](const RadarParams& r) { return r.id == id; }),
            radars_.end()
        );
        dirty_ = true;
    }
    
    void clearRadars() {
        radars_.clear();
        dirty_ = true;
    }
    
    void setNumRays(int n) { numRays_ = n; dirty_ = true; }
    void setSimplifyEpsilon(double eps) { simplifyEpsilon_ = eps; dirty_ = true; }
    void setSmoothIterations(int n) { smoothIterations_ = n; dirty_ = true; }
    
    const std::vector<Polygon>& getIndividualCoverages() {
        updateIfDirty();
        return individualCoverages_;
    }
    
    const MultiPolygon& getMergedCoverage() {
        updateIfDirty();
        return mergedCoverage_;
    }
    
    PolygonStats getStats() {
        updateIfDirty();
        return PolygonStats::compute(mergedCoverage_);
    }
    
    void invalidate() { dirty_ = true; }

private:
    void updateIfDirty() {
        if (!dirty_) return;
        
        individualCoverages_.clear();
        individualCoverages_.reserve(radars_.size());
        
        for (const auto& radar : radars_) {
            Polygon coverage = generateCoveragePolygon(radar, terrain_, numRays_);
            individualCoverages_.push_back(coverage);
        }
        
        mergedCoverage_ = PolygonBoolean::unionAll(individualCoverages_);
        
        if (simplifyEpsilon_ > 0) {
            mergedCoverage_ = PolygonProcessor::simplifyAll(mergedCoverage_, simplifyEpsilon_);
        }
        if (smoothIterations_ > 0) {
            mergedCoverage_ = PolygonProcessor::smoothAll(mergedCoverage_, smoothIterations_);
        }
        
        dirty_ = false;
    }
    
    TerrainModel terrain_;
    std::vector<RadarParams> radars_;
    
    std::vector<Polygon> individualCoverages_;
    MultiPolygon mergedCoverage_;
    
    int numRays_;
    double simplifyEpsilon_;
    int smoothIterations_;
    bool dirty_ = true;
};

// ============================================================================
// 导出函数
// ============================================================================

inline std::string exportToSVG(
    const std::vector<RadarParams>& radars,
    const std::vector<Polygon>& coverages,
    const MultiPolygon& merged,
    const TerrainModel& terrain,
    int width = 800, int height = 600
) {
    std::ostringstream svg;
    
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" height=\"" << height << "\">\n"
        << "  <rect width=\"100%\" height=\"100%\" fill=\"#0a0f1a\"/>\n";
    
    for (const auto& obs : terrain.getObstacles()) {
        svg << "  <ellipse cx=\"" << obs.center.x << "\" cy=\"" << obs.center.y 
            << "\" rx=\"" << obs.rx << "\" ry=\"" << obs.ry 
            << "\" fill=\"#92400e\" fill-opacity=\"0.5\"/>\n";
    }
    
    for (const auto& pwh : merged) {
        svg << "  <polygon points=\"";
        for (size_t i = 0; i < pwh.outer.size(); i++) {
            if (i > 0) svg << " ";
            svg << pwh.outer[i].x << "," << pwh.outer[i].y;
        }
        svg << "\" fill=\"#06b6d4\" fill-opacity=\"0.3\" "
            << "stroke=\"#06b6d4\" stroke-width=\"2\"/>\n";
        
        for (const auto& hole : pwh.holes) {
            svg << "  <polygon points=\"";
            for (size_t i = 0; i < hole.size(); i++) {
                if (i > 0) svg << " ";
                svg << hole[i].x << "," << hole[i].y;
            }
            svg << "\" fill=\"#0a0f1a\" stroke=\"#ef4444\" stroke-width=\"1\"/>\n";
        }
    }
    
    const char* colors[] = {"#3b82f6", "#10b981", "#f59e0b", "#ef4444", "#8b5cf6"};
    for (size_t i = 0; i < radars.size(); i++) {
        const auto& r = radars[i];
        svg << "  <circle cx=\"" << r.position.x << "\" cy=\"" << r.position.y 
            << "\" r=\"10\" fill=\"" << colors[i % 5] << "\"/>\n";
    }
    
    svg << "</svg>\n";
    return svg.str();
}

} // namespace radar_coverage
