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

#include "gl.hpp"
#include <GLUT/glut.h>
using namespace std;

namespace rgbdvo {

bool GlobalFlags::start_loop_clourse_detection = false;

bool GlobalConfig::use_depth_filter = true;
float GlobalConfig::depth_lower_bound = 1.0f;
float GlobalConfig::depth_upper_bound = 3.5f;
int32_t GlobalConfig::frame_sequence_size = 400;
bool GlobalConfig::save_frame_sequence = false;
bool GlobalConfig::save_transformation = false;
std::string GlobalConfig::frame_sequence_fn = "frame.data";
std::string GlobalConfig::transformation_fn = "transformation.data";
bool GlobalConfig::save_optimized_frame_sequence = false;
std::string GlobalConfig::optimized_frame_sequence_fn = "optimized_frame.data";


void ConvertRawDepthToPointClound(const uint16_t* raw_depth,
                                  float* point_cloud) {
  double XtoZ = 2 * tan(1.01447 / 2);
  double YtoZ = 2 * tan(0.789808 / 2);
  double X_RES = static_cast<double>(640);
  double Y_RES = static_cast<double>(480);

  /*
    X_realworld = (X_project / X_RES - 0.5) * z * XtoZ
    Y_realworld = (0.5 - Y_project / Y_RES) * z * YtoZ
   */
  for (int32_t y = 0; y < Y_RES; ++y) {
    for (int32_t x = 0; x < X_RES; ++x) {
      int32_t idx = y * X_RES + x;
      float z = raw_depth[idx] * 0.001;
      if (isnan(z)) {
        point_cloud[idx * 3 + 0] = 0;
        point_cloud[idx * 3 + 1] = 0;
        point_cloud[idx * 3 + 2] = 0;
      } else {
        point_cloud[idx * 3 + 0] =
          (static_cast<double>(x) / X_RES - 0.5) * z * XtoZ;
        point_cloud[idx * 3 + 1] =
          (0.5 - static_cast<double>(y) / Y_RES) * z * YtoZ;
        point_cloud[idx * 3 + 2] = z;
      }
    }
  }
}

namespace GL{

static int32_t _last_x = 0;
static int32_t _last_y = 0;
static int32_t _old_left_state = GLUT_UP;
void MouseCallback(int32_t button, int32_t state, int32_t x, int32_t y) {
  if (button == GLUT_LEFT_BUTTON && _old_left_state == GLUT_UP &&
      state == GLUT_DOWN) {
    _old_left_state = GLUT_DOWN;
    _last_x = x;
    _last_y = y;
  } else if (button == GLUT_LEFT_BUTTON && _old_left_state == GLUT_DOWN &&
           state == GLUT_UP) {
    _old_left_state = GLUT_UP;
  }
}

void MouseMotionCallback(int32_t x, int32_t y) {
  ViewControler* view_controler = ViewControler::instance();
  if (_old_left_state == GLUT_DOWN) {
    view_controler->SetViewDir(view_controler->GetViewDir() * 500 +
                     view_controler->GetRightDir()*(float)(-x + _last_x));
    _last_x = x;
  }
  if (_old_left_state == GLUT_DOWN) {
    view_controler->SetViewDir(view_controler->GetViewDir() * 500 -
                     view_controler->GetUpDir()*(float)(y - _last_y));
    _last_y = y;
  }
  glutPostRedisplay();
}

void KeyboardCallback(uint8_t key, int32_t x, int32_t y) {
  ViewControler* view_controler = ViewControler::instance();
  Eigen::Vector3d temp;
  Eigen::Vector3d ivec(1.0,0.0,0.0);
  Eigen::Vector3d jvec(0.0,0.0,1.0);
  float speed = 0.4f;
  switch (key) {
    // ESC
    case 27:
      view_controler->exit_cleanup();
      exit(1);
      break;
    // Move Left
    case 'a':
      view_controler->SetPosition(
        view_controler->GetPosition() - view_controler->GetRightDir()*speed);
      break;
    // Move Right
    case 'd':
      view_controler->SetPosition(
        view_controler->GetPosition() + view_controler->GetRightDir()*speed);
      break;
    // Move Forward
    case 'w':
      temp = (ivec*view_controler->GetViewDir().dot(ivec) +
              jvec*view_controler->GetViewDir().dot(jvec));
      temp = temp.normalized();
      view_controler->SetPosition(view_controler->GetPosition() + temp*speed);
      break;
    // Move Backward
    case 's':
      temp = (ivec*view_controler->GetViewDir().dot(ivec) +
              jvec*view_controler->GetViewDir().dot(jvec));
      temp = temp.normalized();
      view_controler->SetPosition(view_controler->GetPosition() - temp*speed);
      break;
    // Go Up
    case 'q':
      view_controler->SetPosition(view_controler->GetPosition() +
                                  view_controler->GetUpDir()*speed);
      break;
    // Go Down
    case 'e':
      view_controler->SetPosition(view_controler->GetPosition() -
                                  view_controler->GetUpDir()*speed);
      break;

    // Reset the view_controler to initial position
    case 'r':
      view_controler->Reset();
      break;

    // Perform Loop Detection
    case 'l':
      GlobalFlags::start_loop_clourse_detection = true;
      view_controler->Call("loop clourse detection");
      break;
  }
  glutPostRedisplay();
}

void Reshape(int32_t w, int32_t h) {
  ViewControler* view_controler = ViewControler::instance();
  view_controler->resize(w, h);
}

void DrawCameraOri(void) {
  // x
  glColor3ub(255, 0, 0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.05, 0.0, 0.0);
  glEnd();

  // y
  glColor3ub(0, 255, 0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.05, 0.0);
  glEnd();

  // z
  glColor3ub(255, 255, 255);
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.05);
  glEnd();
}

void CreateWorldMask(void) {
  static bool first_time = true;
  static float w = 20;
  static float h = 20;
  static float y = 0;
  static float unit = 0.5;
  static int x_direction_points = (static_cast<int>(w / unit) + 1) * 2;
  static int z_direction_points = (static_cast<int>(h / unit) + 1) * 2;
  static float *x_points =
      new float[x_direction_points * 3];
  static float *z_points =
      new float[z_direction_points * 3];

  if (first_time) {
    first_time = false;
    Eigen::Vector3f v1(-w/2, y, -h/2);
    Eigen::Vector3f v2(-w/2, y, h/2);
    for (int i = 0; i < x_direction_points / 2; ++i) {
      x_points[i*6] = v1(0);
      x_points[i*6+1] = v1(1);
      x_points[i*6+2] = v1(2);
      x_points[i*6+3] = v2(0);
      x_points[i*6+4] = v2(1);
      x_points[i*6+5] = v2(2);
      v1(0) += unit;
      v2(0) += unit;
    }

    Eigen::Vector3f v3(-w/2, y, -h/2);
    Eigen::Vector3f v4(w/2, y, -h/2);
    for (int i = 0; i < z_direction_points / 2; ++i) {
      z_points[i*6] = v3(0);
      z_points[i*6+1] = v3(1);
      z_points[i*6+2] = v3(2);
      z_points[i*6+3] = v4(0);
      z_points[i*6+4] = v4(1);
      z_points[i*6+5] = v4(2);
      v3(2) += unit;
      v4(2) += unit;
    }
  }

  glColor3ub(0, 0, 255);
  for (int i = 0; i < x_direction_points / 2; ++i) {
    Eigen::Vector3f v1(x_points[i*6], x_points[i*6+1], x_points[i*6+2]);
    Eigen::Vector3f v2(x_points[i*6+3], x_points[i*6+4], x_points[i*6+5]);
    Eigen::Vector3f v3(z_points[i*6], z_points[i*6+1], z_points[i*6+2]);
    Eigen::Vector3f v4(z_points[i*6+3], z_points[i*6+4], z_points[i*6+5]);
    glBegin(GL_LINES);
      glVertex3f(v1(0), v1(1), v1(2));
      glVertex3f(v2(0), v2(1), v2(2));
      glVertex3f(v3(0), v3(1), v3(2));
      glVertex3f(v4(0), v4(1), v4(2));
    glEnd();
  }
}

void DrawCameraVolume(void) {
  float w = 640;
  float h = 480;
  float fx = 519.0f;
  float fy = 519.0f;
  float cx = 335.0f;
  float cy = 267.0f;
  float z = 1.0f;
  float x1 = z*(0.0f-cx)/fx;
  float y1 = z*(0.0f-cy)/fy;
  float x2 = z*(w-cx)/fx;
  float y2 = z*(0.0f-cy)/fy;
  float x3 = z*(w-cx)/fx;
  float y3 = z*(h-cy)/fy;
  float x4 = z*(0.0f-cx)/fx;
  float y4 = z*(h-cy)/fy;

  glBegin(GL_LINES);
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(x1, y1, z);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(x2, y2, z);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(x3, y3, z);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(x4, y4, z);

        glVertex3f(x1, y1, z);
        glVertex3f(x2, y2, z);
        glVertex3f(x2, y2, z);
        glVertex3f(x3, y3, z);
        glVertex3f(x3, y3, z);
        glVertex3f(x4, y4, z);
        glVertex3f(x4, y4, z);
        glVertex3f(x1, y1, z);
    glEnd();
}

ViewControler* ViewControler::instance_= NULL;

ViewControler::
ViewControler(Eigen::Vector3d position,
                Eigen::Vector3d view_dir,
                Eigen::Vector3d up_dir,
                double nearZ,
                double farZ,
                Callback callback)
                : init_position_(position),
                  init_view_dir_(view_dir),
                  init_up_dir_(up_dir),
                  nearZ_(nearZ),
                  farZ_(farZ),
                  callback_(callback) {
  LoadInitSetting();
}

void ViewControler::
Create(Eigen::Vector3d position,
       Eigen::Vector3d view_dir,
       Eigen::Vector3d up_dir,
       double nearZ,
       double farZ,
       Callback callback) {
  instance_ =
    new ViewControler(position, view_dir, up_dir, nearZ, farZ, callback);
}


void ViewControler::
LoadInitSetting() {
  position_ = init_position_;
  view_dir_ = init_view_dir_;
  up_dir_ = init_up_dir_;
}

void ViewControler::Reset() {
  LoadInitSetting();
}

void ViewControler::LookAt() {
  gluLookAt(
    (GLdouble) position_.x(),
    (GLdouble) position_.y(),
    (GLdouble) position_.z(),
    (GLdouble) position_.x() + view_dir_.x(),
    (GLdouble) position_.y() + view_dir_.y(),
    (GLdouble) position_.z() + view_dir_.z(),
    (GLdouble) up_dir_.x(),
    (GLdouble) up_dir_.y(),
    (GLdouble) up_dir_.z()
  );
}

void ViewControler::resize(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (float)width/(float)height, nearZ_, farZ_);
}

void ViewControler::exit_cleanup(void) {
  if (!callback_) return;
  callback_();
}

void ViewControler::
RegistCallback(const string& name, Callback callback) {
  callback_map_[name] = callback;
}

void ViewControler::
Call(const string& name) const {
  if (callback_map_.count(name)) {
    callback_map_.at(name)();
  }
}

} // namespace GL

} // namespace rgbdmapping