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

#include "rgbdvo/camera.hpp"
#include "rgbdvo/vo.hpp"
#include "sensor.hpp"
#include "gl.hpp"
#include "pose_graph.hpp"
#include "helper.hpp"
#include <GLUT/glut.h>
#include <memory>
#include <string>

using namespace std;
using namespace sensor;
using namespace rgbdvo;
using namespace rgbdslam;

static mutex _visual_odometry_lock;
static mutex _pose_graph_lock;

static const int32_t GL_MAIN_WINDOW_WIDTH = 640;
static const int32_t GL_MAIN_WINDOW_HEIGHT = 480;
void PackVertexData(const VertexId& id,
                    const uint8_t* bgr,
                    const uint16_t* depth,
                    VertexData* data) {
  assert(data);
  data->id = id;
  memcpy(data->bgr, bgr, NUM_PIXELS * 3);
  memcpy(data->raw_depth, depth, NUM_PIXELS * 2);
}

bool IsDepthValid(float val, float lo = 0.3, float hi = 5.0) {
  return (val >= lo && val < hi);
}

size_t PackGLArray(const uint8_t* bgr, const uint16_t* depth, float* data) {
  static auto& params = XtionProLive::instance()->GetIntrinsicParamters();
  static DepthToPointCloud converter(params.h_fov,
                                     params.v_fov,
                                     params.width,
                                     params.height);
  size_t valid_points = 0;
  size_t idx = 0;
  size_t sz = params.width * params.height * 3;
  for (int32_t i = 0; i < params.height; ++i) {
    for (int32_t j = 0; j < params.width; ++j) {
      float z = converter.GetZ(depth, j, i);
      if (!IsDepthValid(z)) {
        continue;
      }
      float x = converter.GetX(depth, j, i);
      float y = converter.GetY(depth, j, i);
      size_t idx1 = (i * params.width + j) * 3;
      data[idx + 0] = bgr[idx1 + 2] / 255.0; // r
      data[idx + 1] = bgr[idx1 + 1] / 255.0; // g
      data[idx + 2] = bgr[idx1 + 0] / 255.0; // b

      data[idx + 3] = -x; // mirro x
      data[idx + 4] = y;
      data[idx + 5] = z;
      ++valid_points;
      idx += 6;
    }
  }
  return valid_points;
}

void ShowKeyFrameCamPose() {
  auto graph = PoseGraph::instance();
  _pose_graph_lock.lock();
  const auto& vmaps = graph->GetVertexMap();
  for (const auto& vmap: vmaps) {
    auto v = vmap.second;
    glPushMatrix();
      glRotated(180, 0, 0, 1);
      glMultMatrixd(v.pose.data());
      glRotated(180, 0, 0, 1);
      GL::DrawCameraVolume();
      GL::DrawCameraOri();
    glPopMatrix();
  }
  _pose_graph_lock.unlock();
}

void ShowKeyFramePointCloud() {
  static float* render_buffer = new float[640*480*6];
  auto graph = PoseGraph::instance();
  _pose_graph_lock.lock();
  const auto& vdatamaps = graph->GetVertexDataMap();
  for (const auto& vdatamap : vdatamaps) {
    const auto& vdata = vdatamap.second;
    auto pose = graph->GetVertexById(vdata->id).pose.data();
    size_t valid_points
      = PackGLArray(vdata->bgr, vdata->raw_depth, render_buffer);
    glPointSize(1.0);
    glInterleavedArrays(GL_C3F_V3F, 0, render_buffer);
    glPushMatrix();
      glRotated(180, 0, 0, 1);
      glMultMatrixd(pose);
      glRotated(180, 0, 0, 1);
      glDrawArrays(GL_POINTS, 0, valid_points);
    glPopMatrix();
  }
  _pose_graph_lock.unlock();
}


void ExitCleanup(void) {}

void ShowTracking(cv::Mat bgr) {
  auto estimator = RGBDVO::instance()->GetMotionEstimator();
  auto matches = estimator->GetMatches();
  int32_t number_matches = estimator->GetNumMatches();
  int32_t number_inliers = estimator->GetNumInliers();
  vector<cv::Point> ref_inliers;
  vector<cv::Point> tar_inliers;
  vector<cv::Point> ref_outliers;
  vector<cv::Point> tar_outliers;
  for (int32_t i = 0; i < number_matches; ++i) {
    if (matches[i].inlier) {
      ref_inliers.push_back(
        cv::Point(matches[i].ref_keypoint->base_uv[0],
                  matches[i].ref_keypoint->base_uv[1]));
      tar_inliers.push_back(
        cv::Point(matches[i].target_keypoint->base_uv[0],
                  matches[i].target_keypoint->base_uv[1]));
    } else {
      ref_outliers.push_back(
        cv::Point(matches[i].ref_keypoint->base_uv[0],
                  matches[i].ref_keypoint->base_uv[1]));
      tar_outliers.push_back(
        cv::Point(matches[i].target_keypoint->base_uv[0],
                  matches[i].target_keypoint->base_uv[1]));
    }
  }
  for (int32_t i = 0; i < ref_inliers.size(); ++i) {
    cv::line(bgr, ref_inliers[i], tar_inliers[i], cv::Scalar(0,255,0),1);
    cv::circle(bgr, tar_inliers[i], 2, cv::Scalar(0,255,0),2);
  }
  cv::imshow("Feature Tracking", bgr);
  cv::waitKey(1);
}

void display(void) {
  static bool first = true;
  if (first) {
    first = false;
    return;
  }
  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
    glLoadIdentity();
    GL::ViewControler* view_controler = GL::ViewControler::instance();
    view_controler->LookAt();
    ShowKeyFramePointCloud();
  glPopMatrix();
  glutSwapBuffers();
}

void idel(void) {
  glutPostRedisplay();
}

void InitGL(int32_t argc, char* argv[]) {
  // Init
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

  // GL Main Window Setting
  glutInitWindowSize(GL_MAIN_WINDOW_WIDTH, GL_MAIN_WINDOW_HEIGHT);
  glutInitWindowPosition(100,100);
  glutCreateWindow("OpenGL Main Window");

  // Background color
  glClearColor(0.4, 0.47, 0.55, 0.0);

  //
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);

  // Configure Lighting
  GLfloat LightAmbient[] = {0.9f, 0.9f, 0.9f, 1.0f};
  GLfloat LightDiffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
  GLfloat LightSpecular[] = {0.5f, 0.5f, 0.5f, 1.0f};

  glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
  glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 90.0);

  // Enable various rending attributes
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_COLOR_MATERIAL);

  // Add specularity to all objects
  float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specReflection);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 30.0);

  // Configure Appearance of POINTS
  glPointSize(2.0);

  // Regist callback
  glutReshapeFunc(GL::Reshape);
  glutKeyboardFunc(GL::KeyboardCallback);
  glutMouseFunc(GL::MouseCallback);
  glutMotionFunc(GL::MouseMotionCallback);
  glutDisplayFunc(display);
  glutIdleFunc(idel);
}

void Tracking(void) {
  auto sensor = XtionProLive::instance();

  auto vo = RGBDVO::instance();
  auto graph = PoseGraph::instance();

  vo->SetInitPose();

  VertexId id = 0;
  unique_ptr<VertexData> v_data_buff(new VertexData);
  unique_ptr<uint8_t[]> gray(new uint8_t[640 * 480]);
  while (sensor->PrepareAll()) {
    ConvertBGR2Gray(640, 480, sensor->BGR().data, gray.get());
    _visual_odometry_lock.lock();
    vo->ProcessFrame(gray.get(), (const uint16_t*)sensor->RawDepth().data);
    _visual_odometry_lock.unlock();
    _pose_graph_lock.lock();
    if (vo->IsKeyFrameChanged()) {
      Vertex v(id, vo->GetPose().matrix());
      graph->InsertVertex(v);
      PackVertexData(id, sensor->BGR().data,
                     (const uint16_t*)sensor->RawDepth().data,
                     v_data_buff.get());
      graph->InsertVertexData(v_data_buff.get());

      if (id > 0) {
        Edge e(id-1, id,
               vo->GetCurrToRefMotionEstimate().matrix(),
               vo->GetCurrToRefCov());
        graph->InsertEdge(e);
      }

      if (graph->LinearLoopClosure(id)) {
        cout << "Loop found." << endl;
        graph->Optimize();
      }
      ++id;
    }
    _pose_graph_lock.unlock();
    cv::Mat bgr = sensor->BGR().clone();
    ShowTracking(bgr);
  }
}

void TestOptimizer() {
  PoseGraphOptimizer opt;
  opt.LoadGraph("graph.sample");
  opt.Optimize();
}

void TestAll(int argc, char *argv[]) {
  assert(XtionProLive::Connect());
  const auto& pa = XtionProLive::instance()->GetIntrinsicParamters();
  IntrinsicParameters params;
  params.width  = pa.width;
  params.height = pa.height;
  params.depth_width = pa.depth_width;
  params.depth_height = pa.depth_height;
  params.fx = pa.fx;
  params.fy = pa.fy;
  params.cx = pa.cx;
  params.cy = pa.cy;
  Camera camera(params);
  Options opt = GetRGBDVODefaultOptions();
  RGBDVO::Create(camera, opt);

  assert(PoseGraph::Create());
  PoseGraph::instance()->SetFrameMatcher(new rgbdvo::FrameMatcher(camera));

  InitGL(argc, argv);
  GL::ViewControler::
  Create(Eigen::Vector3d(0,0,-2),
         Eigen::Vector3d(0,0,1),
         Eigen::Vector3d(0,1,0),
         0.01, 300.0, ExitCleanup);
  thread TrackingTread(Tracking);
  glutMainLoop();
}

int main(int argc, char *argv[]) {
  TestAll(argc, argv);
}