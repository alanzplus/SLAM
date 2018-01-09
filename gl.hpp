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

#ifndef __COMMON_H__
#define __COMMON_H__
// C
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
// C++
#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>
#include <utility>
#include <map>
#include <cstdint>
#include <cstring>
#include <string>
#include <exception>
#include <thread>
#include <mutex>
#include <atomic>

#include "Eigen/Eigen"

namespace rgbdvo {

class NonCopyable {
 protected:
  NonCopyable()  {}
  ~NonCopyable() {}
 private:
  NonCopyable(const NonCopyable&);
  const NonCopyable& operator=(const NonCopyable&);
};

const int32_t NETWORK_CONTORL_PORT = 9999;
const int32_t NETWORK_VIDEO_PORT = 9998;

class GlobalFlags {
  public:
    static bool start_loop_clourse_detection;
};

class GlobalConfig {
  public:
    static bool use_depth_filter;
    static float depth_lower_bound;
    static float depth_upper_bound;
    static int32_t frame_sequence_size;
    static bool save_frame_sequence;
    static bool save_transformation;
    static std::string frame_sequence_fn;
    static std::string transformation_fn;

    static bool save_optimized_frame_sequence;
    static std::string optimized_frame_sequence_fn;
};

inline Eigen::Vector3d toEuler(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  const double& q0 = q.w();
  const double& q1 = q.x();
  const double& q2 = q.y();
  const double& q3 = q.z();
  double roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  double pitch = asin(2*(q0*q2-q3*q1));
  double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
  return Eigen::Vector3d(roll, pitch, yaw);
}


namespace GL {

// void InitGL(int32_t argc, char* argv[]);
void MouseCallback(int32_t button, int32_t state, int32_t x, int32_t y);
void MouseMotionCallback(int32_t x, int32_t y);
void KeyboardCallback(uint8_t key, int32_t x, int32_t y);
void Reshape(int32_t w, int32_t h);
void CreateWorldMask(void);
void DrawCameraOri(void);
void DrawCameraVolume(void);

class ViewControler {
 public:
   typedef void(*Callback)(void);
   typedef std::map<std::string, Callback> CallbackMap;

   static ViewControler* instance() {
     if (!instance_) {
       std::cerr << "GL::ViewControler::instance is empty, exit" << std::endl;
       exit(-1);
     }
     return instance_;
   }

   static void Create(Eigen::Vector3d position,
                      Eigen::Vector3d view_dir,
                      Eigen::Vector3d up_dir,
                      double nearZ,
                      double farZ,
                      Callback callback = NULL);
   void Reset(void);
   void LookAt();
   void resize(int32_t width, int32_t height);
   void exit_cleanup();
   void RegistCallback(const std::string& name, Callback callback);
   void Call(const std::string&) const;

   inline Eigen::Vector3d GetPosition();
   inline Eigen::Vector3d GetViewDir();
   inline Eigen::Vector3d GetUpDir();
   inline Eigen::Vector3d GetRightDir();
   inline void SetPosition(const Eigen::Vector3d& pos);
   inline void SetViewDir(const Eigen::Vector3d& vdir);
   inline void SetUpDir(const Eigen::Vector3d& udir);

  private:
    ViewControler(Eigen::Vector3d position,
                  Eigen::Vector3d view_dir,
                  Eigen::Vector3d up_dir,
                  double nearZ,
                  double farZ,
                  Callback callback);
    ~ViewControler() {}
    void LoadInitSetting();

  private:
   static ViewControler* instance_;
   Callback callback_;
   CallbackMap callback_map_;

   Eigen::Vector3d position_;
   Eigen::Vector3d view_dir_;
   Eigen::Vector3d up_dir_;

   Eigen::Vector3d init_position_;
   Eigen::Vector3d init_view_dir_;
   Eigen::Vector3d init_up_dir_;

   double nearZ_;
   double farZ_;
};

inline Eigen::Vector3d ViewControler::GetPosition() {
  return position_;
}

inline Eigen::Vector3d ViewControler::GetViewDir() {
  return view_dir_;
}

inline Eigen::Vector3d ViewControler::GetUpDir() {
  return up_dir_;
}

inline Eigen::Vector3d ViewControler::GetRightDir() {
  Eigen::Vector3d right = view_dir_.cross(up_dir_);
  right = right.normalized();
  return right;
}

inline void ViewControler::SetPosition(const Eigen::Vector3d& pos) {
  position_ = pos;
}

inline void ViewControler::SetViewDir(const Eigen::Vector3d& vdir) {
  view_dir_ = vdir;
  view_dir_ = view_dir_.normalized();
}

inline void ViewControler::SetUpDir(const Eigen::Vector3d& udir) {
  up_dir_ = udir;
}

} // namespace GL

} // namespace
#endif