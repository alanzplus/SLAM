// The MIT License (MIT)

// Copyright (c) 2014.4 JZ Xuan <jzxuanuni@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef __RGBD_SLAM_POSE_GRAPH__
#define __RGBD_SLAM_POSE_GRAPH__
#include <unordered_map>
#include <functional>
#include <utility>
#include <string>
#include <Eigen/Core>

// #include "helper.hpp"
#include "settings.hpp"
#include "rgbdvo/fm.hpp"

#include "g2o/core/sparse_optimizer.h"

namespace rgbdslam {

const size_t FRAME_WIDTH = 640;
const size_t FRAME_HEIGHT = 480;
const size_t NUM_PIXELS = FRAME_WIDTH * FRAME_HEIGHT;

typedef uint32_t VertexId;
struct EdgeId {
  EdgeId() {}
  EdgeId(VertexId v_from, VertexId v_to) : from(v_from), to(v_to) {}
  VertexId from;
  VertexId to;
};

struct Vertex {
  Vertex() {}
  Vertex(const VertexId& v_id, const Eigen::Matrix4d& v_pose)
    : id(v_id), pose(v_pose) {}
  VertexId id;
  Eigen::Matrix4d pose;
};

struct VertexData {
  VertexData() : id(-1) {}
  VertexId id;
  uint8_t bgr[NUM_PIXELS * 3];
  uint16_t raw_depth[NUM_PIXELS];
  VertexData(const VertexData& rhs) = delete;
  const VertexData& operator=(const VertexData& rhs) = delete;
};

struct Edge {
  Edge() {}
  Edge(VertexId v_from, VertexId v_to,
       const Eigen::Matrix4d v_motion,
       const Eigen::Matrix<double, 6, 6> v_cov)
    : from(v_from), to(v_to), motion(v_motion), covariance(v_cov) {}
  VertexId from;
  VertexId to;
  Eigen::Matrix4d motion;
  Eigen::Matrix<double, 6, 6> covariance;
};

} // namespace rgbdslam

namespace std {
template<>
struct hash<rgbdslam::EdgeId> {
  typedef rgbdslam::EdgeId argument_type;
  typedef size_t result_type;
  // NOTE: may overflow
  result_type operator()(const argument_type& x) const {
    return (x.from + x.to) * (x.from + x.to + 1) / 2 + x.to;
  }
};

template<>
struct equal_to<rgbdslam::EdgeId> {
  typedef rgbdslam::EdgeId first_argument_type;
  typedef rgbdslam::EdgeId second_argument_type;
  typedef bool result_type;
  bool operator()(const first_argument_type& x,
                  const second_argument_type& y) const {
    return (x.from == y.from) && (x.to == y.to);
  }
};

} // namespace std

namespace rgbdslam {

class PoseGraphOptimizer {
 public:
  PoseGraphOptimizer();
  ~PoseGraphOptimizer();
  void PrintSupportedTypes(void) const;

  void PrintSupportedSolvers(void) const;

  void AddVertex(const Vertex& vertex);
  void AddEdge(const Edge& edge);
  void SaveGraph(const std::string& fn);
  void LoadGraph(const std::string& fn);
  void FindGuage(void);
  void Optimize(int32_t iteration = 10);

  bool GetVertexById(const VertexId& id, Vertex* v) const;

  int32_t VertexNum() const {
    return optimizer_.vertices().size();
  }

  int32_t EdgeNum() const {
    return optimizer_.edges().size();
  }

  void Clear(void) {
    optimizer_.clear();
  }

 private:
  g2o::SparseOptimizer optimizer_;
};

class PoseGraph {
 public:
  typedef std::unordered_map<VertexId, Vertex> VertexMap;
  typedef std::unordered_map<VertexId, VertexData*> VertexDataMap;
  typedef std::unordered_map<EdgeId, Edge> EdgeMap;

 public:
  static bool Create(Settings settings = GetPoseGraphDefulatSettings());

  static PoseGraph* instance() {
    assert(instance_);
    return instance_;
  }

  void ClearVertices() {
    vertices_.clear();
  }

  void ClearVertexDatas() {
    // vertex_data_.clear();
    assert(nullptr);
  }

  void ClearEdges() {
    edges_.clear();
  }

  void Clear() {
    ClearVertices();
    ClearVertexDatas();
    ClearEdges();
  }

  size_t NumVertex() const {
    return vertices_.size();
  }

  bool IsVertexMapEmpty() const {
    return NumVertex() == 0;
  }

  size_t NumEdge() const {
    return edges_.size();
  }

  void InsertVertex(const Vertex& vertex) {
    assert(vertex.id >= 0 && !IsInVertexMap(vertex.id));
    vertices_[vertex.id] = vertex;
    if (!use_single_trhead_optimization_) return;
    optimizer_->AddVertex(vertex);
  }

  void InsertVertexData(const VertexData* vertex_data) {
    assert(vertex_data->id >= 0 && !IsInVertexDataMap(vertex_data->id));
    VertexData* data = new VertexData();
    data->id = vertex_data->id;
    memcpy(data->bgr, vertex_data->bgr, NUM_PIXELS * 3);
    memcpy(data->raw_depth, vertex_data->raw_depth,
           NUM_PIXELS * sizeof(uint16_t));
    vertex_data_[data->id] = data;
  }


  void InsertEdge(const Edge& edge) {
    EdgeId id(edge.from, edge.to);
    assert(id.from >= 0 && id.to >= 0);
    assert(!IsInEdgeMap(id));
    edges_[id] = edge;
    if (!use_single_trhead_optimization_) return;
    optimizer_->AddEdge(edge);
  }

  void Optimize(int32_t iteration = 10) {
    if (!use_single_trhead_optimization_) return;
    optimizer_->Optimize(iteration);
  }

  void UpdateOptimizedVertices() {
     if (!use_single_trhead_optimization_) return;
    LoadVerticesFromOptimizer();
  }

  // void InsertEdge(const EdgeId& id, const Edge& edge) {
  //   assert(id.from >= 0 && id.to >= 0);
  //   assert(!IsInEdgeMap(id));
  //   edges_[id] = edge;
  // }

  bool IsInVertexMap(const VertexId& id) const {
    return vertices_.count(id) > 0;
  }

  bool IsInVertexDataMap(const VertexId& id) const {
    return vertex_data_.count(id) > 0;
  }

  bool IsInEdgeMap(const EdgeId& id) const {
    return edges_.count(id) > 0;
  }

  const VertexMap& GetVertexMap() const {
    return vertices_;
  }

  VertexMap& GetVertexMap() {
    return vertices_;
  }

  const VertexDataMap& GetVertexDataMap() const {
    return vertex_data_;
  }

  VertexDataMap& GetVertexDataMap() {
    return vertex_data_;
  }

  const EdgeMap& GetEdgeMap() const {
    return edges_;
  }

  EdgeMap& GetEdgeMap() {
    return edges_;
  }

  const Vertex& GetVertexById(const VertexId& id) const {
    assert(IsInVertexMap(id));
    return vertices_.at(id);
  }

  Vertex& GetVertexById(const VertexId& id) {
    assert(IsInVertexMap(id));
    return vertices_.at(id);
  }

  const VertexData* GetVertexDataById(const VertexId& id) const {
    assert(IsInVertexDataMap(id));
    return vertex_data_.at(id);
  }

  VertexData* GetVertexDataById(const VertexId& id) {
    assert(IsInVertexDataMap(id));
    return vertex_data_.at(id);
  }

  void SetFrameMatcher(rgbdvo::FrameMatcher* fm) {
    frame_matcher_.reset(fm);
  }

  bool LinearLoopClosure(const VertexId& ref_id);

  // Save a vertex to a file,
  // if the file already exist, append the vertex to the end of the file
  void SaveVertexById(const VertexId& id, const std::string& fn);

  // Load a vertex from file
  bool LoadVertexById(const std::string& fn, const VertexId& id, Vertex* v);

  void SaveVertices(const std::string& fn);
  void LoadVertices(const std::string& fn);

  void LoadVerticesFromOptimizer();
 private:
  PoseGraph(Settings settings);
  ~PoseGraph();

 private:
  static PoseGraph* instance_;
  VertexMap vertices_;
  VertexDataMap vertex_data_;
  EdgeMap edges_;
  std::unique_ptr<PoseGraphOptimizer> optimizer_;

  std::unique_ptr<rgbdvo::FrameMatcher> frame_matcher_;

  // settings
  double lc_dis_th_, lc_ang_th_;
  int32_t lc_lin_win_sz_;
  bool use_single_trhead_optimization_;
};


} // namespace rgbdslam

#endif
