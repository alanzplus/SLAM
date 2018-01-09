/*
The MIT License (MIT)

Copyright (c) 2014.4 JZ Xuan <jzxuanuni@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __RGBD_SLAM_IROBOT__
#define __RGBD_SLAM_IROBOT__
#define IROBOT_LINE_UP        (0)
#define IROBOT_LINE_DOWN      (1)
#define IROBOT_TURN_LEFT      (2)
#define IROBOT_TURN_RIGHT     (3)
#include <cstdint>
#include <string>
#include <memory>
#include <map>
#include <utility>

namespace rgbdslam {
class iRobot {
 public:

    typedef enum {
      FORWARD = 0,
      BACKWARD = 1,
      TURN_LEFT = 2,
      TURN_RIGHT = 3
    }Direction;

    typedef enum {
      SERIAL_DATABIT_8N1 = 0,
      SERIAL_DATABIT_7N1 = 1,
      SERIAL_DATABIT_7O1 = 2
    } SerialFromat;

    struct Options {
      std::string dev;
      int32_t send_buffer_size;
      int32_t recv_buffer_size;
      int32_t baud;
      SerialFromat format;
      int32_t init_line_speed;
      int32_t init_turn_speed;
      Direction init_direction;
      int32_t init_inc_step;
      int32_t init_dec_step;
    };

    struct State {
      int32_t line_speed;
      int32_t turn_speed;
      Direction direction;
      int64_t distance;
      int32_t angle;
      int32_t inc_step;
      int32_t dec_step;
    };

    struct SensorData {
      int16_t distance;
      int32_t angle;
    };

    static const std::map<Direction, std::string>
    DirectionToString;

    static const std::map<std::string, Direction>
    StringToDirection;

 public:
   static bool ConnectToiRobot(const Options& options = GetDefaultOptions());
   static iRobot* GetiRobot(void);
   static Options GetDefaultOptions(void);
   // iRobot related
   void SafeMode(void);
   void FullMode(void);
   void Move(const Direction direction, const int32_t target_speed, bool smooth = false);
   // if success, it will update the distance and angle in state_
   // and sensor_data_
   bool ReadOdometry(void);

   const State& GetState(void) const;
   const SensorData& GetSensorData(void) const;
 private:
   iRobot(const Options& options);
   ~iRobot();

   int32_t GetSpeed(int32_t current_speed, int32_t target_speed, bool smooth);
   // Serial Port relatd
   int32_t OpenSerialPort(const char* dev);
   void CloseSerialPort(int32_t fd);
   bool SetSerialPort(int32_t fd,
                      int32_t baud = 57600,
                      SerialFromat format = SERIAL_DATABIT_8N1);
   inline int32_t SerialSend(int32_t fd, uint8_t* buffer, int32_t len);


 private:
    static iRobot* irobot_;
    Options options_;
    int32_t serial_fd_;
    std::unique_ptr<uint8_t[]> send_buffer_;
    std::unique_ptr<uint8_t[]> recv_buffer_;
    std::unique_ptr<State> state_;
    std::unique_ptr<SensorData> sensor_data_;
};

}
#endif