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

#include <iostream>
#include <fstream>
#include <algorithm>

#include "pose_graph.hpp"
#include "helper.hpp"
#include "ext_utility.hpp"

#include "g2o/config.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/types/slam3d/types_slam3d.h"

using namespace std;
namespace rgbdslam {

PoseGraph* PoseGraph::instance_ = nullptr;

bool PoseGraph::
Create(Settings settings) {
  if (instance_) return true;
  instance_ = new PoseGraph(settings);
  return true;
}

PoseGraph::PoseGraph(Settings settings)
  : frame_matcher_(nullptr) {
  assert(Setting::GetDouble(settings,
                            "loop-closure-distance-threshold",
                            &lc_dis_th_));
  assert(Setting::GetDouble(settings,
                            "loop-closure-angle-threshold",
                            &lc_ang_th_));
  assert(Setting::GetInt(settings,
                         "loop-closure-linear-window-size",
                         &lc_lin_win_sz_));
  assert(Setting::GetBool(settings,
                          "use-single-thread-optimization",
                          &use_single_trhead_optimization_));

  if (use_single_trhead_optimization_) {
    optimizer_.reset(new PoseGraphOptimizer);
  }
}

PoseGraph::~PoseGraph() {}

bool PoseGraph::LinearLoopClosure(const VertexId& ref_id) {
  if (ref_id == 0) return false;
#ifdef PRINT_TRACE
  cerr << "LinearLoopClosure:" << ref_id << endl;
#endif
  assert(frame_matcher_.get());
  static int32_t width = frame_matcher_->GetInputCameraParameters().width;
  static int32_t height = frame_matcher_->GetInputCameraParameters().height;
  static uint8_t* ref_gray = new uint8_t[width * height];
  static uint8_t* tar_gray = new uint8_t[width * height];

  VertexId upper =
    (lc_lin_win_sz_ == -1) ? (ref_id - 1) : min(ref_id - 1, ref_id + lc_lin_win_sz_);

  const auto ref_data = GetVertexDataById(ref_id);

  ConvertBGR2Gray(width, height,
                  ref_data->bgr,
                  ref_gray);
  frame_matcher_->SetRefFrame(ref_gray, ref_data->raw_depth);
  Eigen::Isometry3d ref_pose(GetVertexById(ref_id).pose);
  auto ref_translation = ref_pose.translation();
  auto ref_angles = ref_pose.rotation().eulerAngles(0, 1, 2);
  // Match current vertex against previous vertex
  bool ret = false;

  for (VertexId tar_id = 0; tar_id <= upper; ++tar_id) {
    Eigen::Isometry3d tar_pose(GetVertexById(tar_id).pose);
    auto tar_translation = tar_pose.translation();
    auto tar_angles = tar_pose.rotation().eulerAngles(0, 1, 2);

    if ((tar_translation - ref_translation).norm() > lc_dis_th_) {
      continue;
    }

    auto angle_diff = tar_angles - ref_angles;
    if (abs(angle_diff(0)) > lc_ang_th_ ||
        abs(angle_diff(1)) > lc_ang_th_ ||
        abs(angle_diff(2)) > lc_ang_th_) {
      continue;
    }

    const auto tar_data = GetVertexDataById(tar_id);
    ConvertBGR2Gray(width, height,
                    tar_data->bgr,
                    tar_gray);

    if (!frame_matcher_->Match(tar_gray, tar_data->raw_depth)) {
      continue;
    }

    // EdgeId edge_id(ref_id, tar_id);
    Edge edge(ref_id, tar_id,
              frame_matcher_->GetMotionEstimate().matrix(),
              Eigen::Matrix<double, 6, 6>(frame_matcher_->GetCov().data()));
    InsertEdge(edge);
    // InsertEdge(edge_id, edge);
    ret = true;
  }
  return ret;
}

void PoseGraph::
SaveVertexById(const VertexId& id, const std::string& fn) {
  ofstream outfile(fn, ofstream::binary | ofstream::app);
  ext_utility::IsFileOpenOrExit(outfile, fn);
  const auto& v = GetVertexById(id);
  outfile.write((const char*)&v, sizeof(v));
  outfile.close();
}

bool PoseGraph::
LoadVertexById(const std::string& fn, const VertexId& id, Vertex* v) {
  ifstream infile(fn, ifstream::binary);
  ext_utility::IsFileOpenOrExit(infile, fn);
  size_t f_sz = ext_utility::GetFstreamSize(infile);
  size_t total_read = 0;
  bool ret = false;
  while (total_read < f_sz) {
    infile.read((char*)v, sizeof(Vertex));
    total_read += sizeof(Vertex);
    if (v->id != id) continue;
    ret = true;
    break;
  }
  infile.close();
  return ret;
}

void PoseGraph::
SaveVertices(const std::string& fn) {
  ofstream outfile(fn, ofstream::binary);
  ext_utility::IsFileOpenOrExit(outfile, fn);
  for (const auto& vmap : vertices_) {
    const auto& v = vmap.second;
    outfile.write((char*)&v, sizeof(Vertex));
  }
  outfile.close();
}

void PoseGraph::
LoadVertices(const std::string& fn) {
  assert(IsVertexMapEmpty());
  ifstream infile(fn, ifstream::binary);
  ext_utility::IsFileOpenOrExit(infile, fn);
  size_t f_sz = ext_utility::GetFstreamSize(infile);
  size_t total_read = 0;
  while (total_read < f_sz) {
    Vertex v;
    infile.read((char*)&v, sizeof(Vertex));
    InsertVertex(v);
    total_read += sizeof(Vertex);
  }
  infile.close();
}

void PoseGraph::
LoadVerticesFromOptimizer() {
  if (!use_single_trhead_optimization_) return;
  for (auto vmap : vertices_) {
    auto v = vmap.second;
    auto v_id(v.id);
    optimizer_->GetVertexById(v_id, &v);
  }
}

PoseGraphOptimizer::
PoseGraphOptimizer() {
  g2o::OptimizationAlgorithmFactory* solve_factory =
    g2o::OptimizationAlgorithmFactory::instance();

  g2o::OptimizationAlgorithmProperty solve_property;
  const string solver_str = "gn_var";
  optimizer_.setAlgorithm(
    solve_factory->construct(solver_str, solve_property));
  if (!optimizer_.solver()) {
    cerr << "Error in allocating solver: " << solver_str << ", exit." << endl;
    exit(-1);
  }

  string solve_properties;
  if (solve_properties.size() > 0) {
    if (!optimizer_.solver()->updatePropertiesFromString(solve_properties)) {
      cerr << "Error in updating solver properties: "
           << solve_properties << ", exit." << endl;
      exit(-1);
    }
  }

#ifdef PRINT_TRACE
  //optimizer_.solver()->printProperties(cerr);
#endif

}

PoseGraphOptimizer::
~PoseGraphOptimizer() {}

void
PoseGraphOptimizer::AddVertex(const Vertex& vertex) {
  g2o::VertexSE3* v = new g2o::VertexSE3();
  v->setId(vertex.id);
  v->setEstimate(Eigen::Isometry3d(vertex.pose));
  if (vertex.id == 0) {
    v->setFixed(true);
  }
  optimizer_.addVertex(v);
}

void
PoseGraphOptimizer::AddEdge(const Edge& edge) {
  auto ref_v = optimizer_.vertex(edge.from);
  auto tar_v = optimizer_.vertex(edge.to);
  Eigen::Isometry3d t(edge.motion);
  g2o::EdgeSE3* e = new g2o::EdgeSE3();
  if (e == nullptr) {
    cerr << "Cannot allocate new edge in optimizer, exit." << endl;
    exit(-1);
  }
  e->setVertex(0, ref_v);
  e->setVertex(1, tar_v);
  Eigen::MatrixXd info = edge.covariance.inverse();
  e->setMeasurement(t);
  e->setInformation(info);
  optimizer_.addEdge(e);
}

void
PoseGraphOptimizer::Optimize(int32_t iteration) {
#ifdef PRINT_TRACE
  optimizer_.setVerbose(true);
#endif
  optimizer_.initializeOptimization();
  optimizer_.computeActiveErrors();
  double loadChi = optimizer_.chi2();
  g2o::EstimatePropagatorCostOdometry cost_function(&optimizer_);
  optimizer_.computeInitialGuess(cost_function);
  int32_t ret = optimizer_.optimize(iteration);
}

void
PoseGraphOptimizer::SaveGraph(const std::string& fn) {
  ofstream outfile(fn);
  ext_utility::IsFileOpenOrExit(outfile, fn);
  optimizer_.save(outfile);
}

void
PoseGraphOptimizer::LoadGraph(const std::string& fn) {
  ifstream infile(fn);
  ext_utility::IsFileOpenOrExit(infile, fn);
  optimizer_.clear();
  optimizer_.load(infile);
#ifdef PRINT_TRACE
  cerr << "Loaded " << optimizer_.vertices().size() << " vertices." << endl;
  cerr << "Loaded " << optimizer_.edges().size() << " edges." << endl;
#endif
}

bool
PoseGraphOptimizer::GetVertexById(const VertexId& id, Vertex* v) const {
  g2o::VertexSE3* vertex =
    (g2o::VertexSE3*)optimizer_.vertex(id);
  if (!vertex) return false;
  v->pose = vertex->estimate().matrix();
  return true;
}

void
PoseGraphOptimizer::PrintSupportedTypes(void) const {
  g2o::Factory::instance()->printRegisteredTypes(std::cout, true);
}

void
PoseGraphOptimizer::PrintSupportedSolvers(void) const {
  g2o::OptimizationAlgorithmFactory::instance()->listSolvers(std::cout);
}

} // namespace rgbd_slam