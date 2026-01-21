# 雷达覆盖区域合并算法 - 集成指南

## 1. 概述

本方案提供了考虑地形遮挡的多部雷达探测覆盖区域合并算法，基于多边形布尔运算实现。

### 核心组件

```
┌─────────────────────────────────────────────────────────────────┐
│                    CoverageMergeManager                         │
│  ┌──────────────┐  ┌───────────────────┐  ┌─────────────────┐  │
│  │ TerrainModel │  │ CoveragePolygon   │  │ PolygonBoolean  │  │
│  │              │─▶│ Generator         │─▶│ Ops (Clipper2)  │  │
│  │ - DEM数据    │  │ - 射线追踪        │  │ - Union         │  │
│  │ - 遮挡计算   │  │ - 多边形生成      │  │ - 后处理        │  │
│  └──────────────┘  └───────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## 2. 依赖安装

### 2.1 Clipper2 (推荐)

```bash
# 从 GitHub 下载
git clone https://github.com/AngusJohnson/Clipper2.git
cd Clipper2/CPP

# 使用 CMake 构建
mkdir build && cd build
cmake ..
make
sudo make install
```

### 2.2 替代方案: Boost.Geometry

如果项目已使用 Boost：

```cpp
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/union.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::polygon<point_t> polygon_t;

// 计算并集
std::vector<polygon_t> result;
bg::union_(poly1, poly2, result);
```

## 3. AFSIM 集成

### 3.1 架构建议

```
AFSIM Simulation
      │
      ▼
┌─────────────────────┐     ┌──────────────────────┐
│  WSF_SENSOR         │────▶│  CoverageMergeManager │
│  (雷达传感器)        │     │  (覆盖合并模块)        │
└─────────────────────┘     └──────────────────────┘
      │                              │
      ▼                              ▼
┌─────────────────────┐     ┌──────────────────────┐
│  WSF_TERRAIN        │────▶│  TerrainModel        │
│  (AFSIM地形)         │     │  (遮挡计算)           │
└─────────────────────┘     └──────────────────────┘
```

### 3.2 与 AFSIM 地形接口

```cpp
// 封装 AFSIM 地形查询
class AFSIMTerrainAdapter {
public:
    AFSIMTerrainAdapter(WsfTerrain* terrain) : terrain_(terrain) {}
    
    double getElevation(double lat, double lon) const {
        WsfGeoPoint point(lat, lon, 0);
        return terrain_->Elevation(point);
    }
    
    // 转换为 TerrainModel 的高程函数
    radar_coverage::TerrainModel::ElevationFunction 
    getElevationFunction() const {
        return [this](double x, double y) {
            // 假设 x, y 是局部坐标，需要转换为经纬度
            auto [lat, lon] = localToGeo(x, y);
            return getElevation(lat, lon);
        };
    }
    
private:
    WsfTerrain* terrain_;
};

// 使用
void setupCoverageManager(CoverageMergeManager& manager, 
                          WsfTerrain* afsim_terrain) {
    AFSIMTerrainAdapter adapter(afsim_terrain);
    manager.terrain().setElevationFunction(adapter.getElevationFunction());
}
```

### 3.3 传感器参数提取

```cpp
RadarParams extractRadarParams(WsfSensor* sensor) {
    RadarParams params;
    
    params.id = sensor->Id();
    
    // 位置
    WsfGeoPoint pos = sensor->Platform()->Location();
    auto [x, y] = geoToLocal(pos.Latitude(), pos.Longitude());
    params.position = Point2D(x, y);
    
    // 天线高度 (平台高度 + 天线偏移)
    params.antenna_height = pos.Altitude() + sensor->AntennaOffset().Z();
    
    // 探测范围
    params.max_range = sensor->MaxRange();
    
    // 角度限制
    params.min_elevation = sensor->MinElevation();
    params.max_elevation = sensor->MaxElevation();
    params.azimuth_start = sensor->MinAzimuth();
    params.azimuth_end = sensor->MaxAzimuth();
    
    return params;
}
```

## 4. 性能优化

### 4.1 空间索引 (R-tree)

当雷达数量较多时，使用 R-tree 加速查询：

```cpp
#include <boost/geometry/index/rtree.hpp>

namespace bgi = boost::geometry::index;

class SpatialIndex {
public:
    using Box = bg::model::box<point_t>;
    using Value = std::pair<Box, int>;  // (包围盒, 雷达ID)
    using RTree = bgi::rtree<Value, bgi::quadratic<16>>;
    
    void insert(int radar_id, const Polygon& coverage) {
        Box bbox = computeBoundingBox(coverage);
        rtree_.insert(std::make_pair(bbox, radar_id));
    }
    
    // 查询与给定区域相交的雷达
    std::vector<int> queryIntersecting(const Box& region) {
        std::vector<Value> results;
        rtree_.query(bgi::intersects(region), std::back_inserter(results));
        
        std::vector<int> ids;
        for (const auto& v : results) {
            ids.push_back(v.second);
        }
        return ids;
    }
    
private:
    RTree rtree_;
};
```

### 4.2 增量更新

仅重算变化的部分：

```cpp
class IncrementalCoverageManager {
public:
    void onRadarMoved(int radar_id) {
        // 1. 找到受影响的相邻雷达
        auto affected = spatial_index_.queryNearby(
            getRadarPosition(radar_id), 
            getRadarRange(radar_id) * 2
        );
        
        // 2. 仅重算这些雷达的局部合并
        invalidateRegion(affected);
    }
    
    void onRadarAdded(int radar_id) {
        // 增量添加到现有合并结果
        Polygon new_coverage = computeCoverage(radar_id);
        merged_ = PolygonBooleanOps::computeUnion({merged_, new_coverage});
    }
    
private:
    SpatialIndex spatial_index_;
    Polygon merged_;
};
```

### 4.3 多线程并行

```cpp
#include <execution>
#include <algorithm>

std::vector<Polygon> computeAllCoverages(
    const std::vector<RadarParams>& radars,
    const TerrainModel& terrain
) {
    std::vector<Polygon> results(radars.size());
    
    // 并行计算各雷达覆盖
    std::transform(
        std::execution::par,
        radars.begin(), radars.end(),
        results.begin(),
        [&terrain](const RadarParams& radar) {
            CoveragePolygonGenerator gen(terrain);
            return gen.generateCoveragePolygon(radar);
        }
    );
    
    return results;
}
```

## 5. 显示集成

### 5.1 输出格式

| 格式 | 用途 | 方法 |
|------|------|------|
| SVG | Web显示 | `exportToSVG()` |
| GeoJSON | GIS系统 | `exportToGeoJSON()` |
| WKT | 数据库 | `exportToWKT()` |
| 顶点数组 | OpenGL/DirectX | `getVertices()` |

### 5.2 WebGL 渲染

```javascript
// 从 GeoJSON 加载到 Three.js
function loadCoverageToThreeJS(geojson) {
    const shape = new THREE.Shape();
    
    const coords = geojson.features[0].geometry.coordinates[0];
    shape.moveTo(coords[0][0], coords[0][1]);
    
    for (let i = 1; i < coords.length; i++) {
        shape.lineTo(coords[i][0], coords[i][1]);
    }
    
    const geometry = new THREE.ShapeGeometry(shape);
    const material = new THREE.MeshBasicMaterial({
        color: 0x2196F3,
        opacity: 0.4,
        transparent: true
    });
    
    return new THREE.Mesh(geometry, material);
}
```

## 6. 测试验证

### 6.1 单元测试

```cpp
TEST(CoverageMerge, TwoOverlappingRadars) {
    CoverageMergeManager manager;
    
    RadarParams r1, r2;
    r1.position = Point2D(0, 0);
    r1.max_range = 100;
    r2.position = Point2D(80, 0);  // 重叠
    r2.max_range = 100;
    
    manager.addRadar(r1);
    manager.addRadar(r2);
    
    auto merged = manager.getMergedCoverage();
    
    // 验证合并后面积小于两个独立面积之和
    double merged_area = PolygonProcessor::area(merged[0]);
    double sum_area = M_PI * 100 * 100 * 2;  // 两个圆
    
    EXPECT_LT(merged_area, sum_area);
}

TEST(CoverageMerge, TerrainOcclusion) {
    CoverageMergeManager manager;
    
    // 在雷达和目标之间添加山
    manager.terrain().addObstacle({
        Point2D(50, 0), 20, 20, 1000
    });
    
    RadarParams r;
    r.position = Point2D(0, 0);
    r.antenna_height = 10;
    r.max_range = 100;
    manager.addRadar(r);
    
    auto coverage = manager.getIndividualCoverages()[0];
    
    // 验证被遮挡方向的范围小于最大范围
    // (检查朝向 x=100 方向的顶点)
    // ...
}
```

## 7. 文件清单

```
radar-coverage/
├── radar_coverage_merge.hpp   # 核心算法头文件
├── example_usage.cpp          # 使用示例
├── radar-coverage-merge.jsx   # React 可视化演示
└── INTEGRATION_GUIDE.md       # 本文档
```

## 8. 常见问题

**Q: 覆盖多边形有自相交怎么办？**

A: 地形遮挡可能产生复杂边界。使用 Clipper2 的 `SimplifyPolygon()` 或设置 `FillRule::NonZero` 可自动处理。

**Q: 如何处理不同高度层的覆盖？**

A: 为每个高度层单独计算覆盖多边形，然后分层显示。可以用不同透明度或颜色区分。

**Q: 实时更新性能不足？**

A: 1) 降低射线数量 (72→36)；2) 使用增量更新；3) 对远处雷达使用简化模型；4) 多线程计算。
