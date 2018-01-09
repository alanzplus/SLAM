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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <assert.h>

#include <iostream>
#include "irobot.hpp"
using namespace std;
namespace rgbdslam {
iRobot* iRobot::irobot_ = NULL;

const std::map<iRobot::Direction, std::string>
iRobot::DirectionToString = {
  std::make_pair(iRobot::Direction::FORWARD, "FORWARD"),
  std::make_pair(iRobot::Direction::BACKWARD, "BACKWARD"),
  std::make_pair(iRobot::Direction::TURN_LEFT, "TURN_LEFT"),
  std::make_pair(iRobot::Direction::TURN_RIGHT, "TURN_RIGHT")
};

const std::map<std::string, iRobot::Direction>
iRobot::StringToDirection = {
  std::make_pair("FORWARD", iRobot::Direction::FORWARD),
  std::make_pair("BACKWARD", iRobot::Direction::BACKWARD),
  std::make_pair("TURN_LEFT", iRobot::Direction::TURN_LEFT),
  std::make_pair("TURN_RIGHT", iRobot::Direction::TURN_RIGHT)
};

iRobot::iRobot(const Options& options) {
  options_ = options;

  send_buffer_.reset(new uint8_t[options.send_buffer_size]);
  recv_buffer_.reset(new uint8_t[options.recv_buffer_size]);

  state_.reset(new State());
  state_->line_speed = options.init_line_speed;
  state_->turn_speed = options.init_turn_speed;
  state_->direction = options.init_direction;
  state_->distance = 0;
  state_->angle = 0;
  state_->inc_step = options.init_inc_step;
  state_->dec_step = options.init_dec_step;

  sensor_data_.reset(new SensorData);
  sensor_data_->distance = 0;
  sensor_data_->angle = 0;

  serial_fd_ = OpenSerialPort(options.dev.c_str());
  SetSerialPort(serial_fd_, options.baud, options.format);
  SafeMode();
}

iRobot::~iRobot() {
  CloseSerialPort(serial_fd_);
}

bool iRobot::
ConnectToiRobot(const Options& options) {
  if (!irobot_) {
    irobot_ = new iRobot(options);
  }
  return true;
}

iRobot* iRobot::
GetiRobot(void) {
  if (irobot_ == NULL) {
    cerr << "irobot_ is NULL." << endl;
    exit(-1);
  }
  return irobot_;
}

iRobot::Options
iRobot::GetDefaultOptions(void) {
  static Options ret;
#if __APPLE__
  ret.dev = "/dev/tty.usbserial-00001014";
#elif __linux
  ret.dev = "/dev/ttyUSB0";
#endif
  ret.send_buffer_size = 128;
  ret.recv_buffer_size = 128;
  ret.baud = 57600;
  ret.format = SERIAL_DATABIT_8N1;
  ret.init_line_speed = 0;
  ret.init_turn_speed = 0;
  ret.init_inc_step = 10;
  ret.init_dec_step = 100;
  ret.init_direction = FORWARD;
  return ret;
}

int32_t iRobot::OpenSerialPort(const char* dev) {
  int32_t ret;
  ret = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (ret == -1) {
    perror("openDEV: cannot open");
    exit(-1);
  }
  return ret;
}

void iRobot::CloseSerialPort(int32_t fd) {
  close(fd);
}

bool iRobot::
SetSerialPort(int32_t fd, int32_t baud, SerialFromat format) {
  // input  <=> receive data
  // output <=> send data
  struct termios newsetting;
  struct termios oldsetting;

  if ( tcgetattr(fd, &oldsetting) != 0) {
    perror("serial abnormal");
    return false;
  }

  // init newsetting
  bzero(&newsetting, sizeof(newsetting));  // set to 0

  // set the Baud rate
  switch (baud) {
    case 4800 :
      cfsetispeed(&newsetting, B4800);
      cfsetospeed(&newsetting, B4800);
      break;

    case 9600 :
      cfsetispeed(&newsetting, B9600);
      cfsetospeed(&newsetting, B9600);
      break;

    case 19200 :
      cfsetispeed(&newsetting, B19200);
      cfsetospeed(&newsetting, B19200);
      break;


    case 57600 :
      cfsetispeed(&newsetting, B57600);
      cfsetospeed(&newsetting, B57600);
      break;

    case 115200 :
      cfsetispeed(&newsetting, B115200);
      cfsetospeed(&newsetting, B115200);
      break;

    default :
      cfsetispeed(&newsetting, B57600);
      cfsetospeed(&newsetting, B57600);
  }

  // set the bit type
  switch(format) {
    case SERIAL_DATABIT_8N1 :   // 8 data bit and 1 stop bit, no parity
      newsetting.c_cflag &= ~PARENB; // disable parity generation and detection
      newsetting.c_cflag &= ~CSTOPB; // use one stop bit
      newsetting.c_cflag |= CS8;     // use 8 data bit
      break;

    case SERIAL_DATABIT_7N1 :   // 7 data bit and 1 stop bit, even parity
      newsetting.c_cflag |= PARENB;  // enable parity generation and detection
      newsetting.c_cflag &= ~PARODD; // use even parity
      newsetting.c_cflag &= ~CSTOPB; // use one stop bit
      newsetting.c_cflag |= CS7;     // use 7 data bit
      newsetting.c_iflag |= INPCK;   // perform parity checking on received characters
      newsetting.c_cflag |= ISTRIP;  // Set to seven bits all incoming characters
      break;

    case SERIAL_DATABIT_7O1 :   // 7 data bit and 1 stop bit, odd parity
      newsetting.c_cflag |= PARENB;  // enable parity generation and detection
      newsetting.c_cflag |= PARODD;  // use odd parity
      newsetting.c_cflag &= ~CSTOPB; // use one stop bit
      newsetting.c_cflag |= CS7;     // use 7 data bit
      newsetting.c_iflag |= INPCK;   // perform parity checking on received characters
      newsetting.c_cflag |= ISTRIP;  // Set to seven bits all incoming characters
      break;

    default :                   // 8 data bit and 1 stop bit, no parity
      newsetting.c_cflag &= ~PARENB; // disable parity generation and detection
      newsetting.c_cflag &= ~CSTOPB; // use one stop bit
      newsetting.c_cflag |= CS8;     // use 8 data bit
    }

    // disable bit mask for data bit
    newsetting.c_cflag &= ~CSIZE;

    // enable receive
    newsetting.c_cflag |= CREAD;

    // Ignore any modem status lines
    newsetting.c_cflag |= CLOCAL;

    // set input to raw data mode
    newsetting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // set output to raw data mode
    newsetting.c_oflag = 0;

    // set the read method
    newsetting.c_cc[VTIME] = 0;
    newsetting.c_cc[VMIN]  = 0;

    // it flushes both data received but not read and data written but not transmitted.
    tcflush(fd, TCIOFLUSH);
    if ( tcsetattr(fd, TCSANOW, &newsetting) != 0) {
       perror("setDEV:tcsetattr");
       return false;
    }

    return true;
}

inline int32_t iRobot::
SerialSend(int32_t fd, uint8_t* buffer, int32_t len) {
  return write(fd, buffer, len);
}

void iRobot::
SafeMode(void) {
  send_buffer_[0] = 128;
  send_buffer_[1] = 131;
  SerialSend(serial_fd_, send_buffer_.get(), 2);
}

void iRobot::
FullMode(void) {
  send_buffer_[0] = 128;
  send_buffer_[1] = 132;
  SerialSend(serial_fd_, send_buffer_.get(), 2);
}

int32_t
iRobot::GetSpeed(int32_t current_speed, int32_t target_speed, bool smooth) {
  if (current_speed < target_speed) {
    current_speed += state_->inc_step;
    current_speed = current_speed > target_speed ? target_speed : current_speed;
  } else {
    current_speed -= state_->dec_step;
    current_speed = current_speed < target_speed ? target_speed : current_speed;
  }
  return current_speed;
}

void iRobot::
iRobot::Move(Direction direction, const int32_t target_speed, bool smooth) {
  if (direction != state_->direction) {
    state_->line_speed = 0;
    state_->turn_speed = 0;
  }
  int32_t speed = target_speed;
  switch (direction) {
    case FORWARD:
      state_->direction = direction;
      state_->line_speed = GetSpeed(state_->line_speed, speed, smooth);
      speed = state_->line_speed;
      send_buffer_[0] = 145;
      send_buffer_[1] = speed >> 8;
      send_buffer_[2] = speed;
      send_buffer_[3] = speed >> 8;
      send_buffer_[4] = speed;
      break;

    case BACKWARD:
      state_->direction = direction;
      state_->line_speed = GetSpeed(state_->line_speed, speed, smooth);
      speed = state_->line_speed;
      send_buffer_[0] = 145;
      send_buffer_[1] = (-speed) >> 8;
      send_buffer_[2] = (-speed);
      send_buffer_[3] = (-speed) >> 8;
      send_buffer_[4] = (-speed);
      break;

    case TURN_LEFT:
      state_->direction = direction;
      state_->turn_speed = GetSpeed(state_->turn_speed, speed, smooth);
      speed = state_->turn_speed;
      send_buffer_[0] = 145;
      send_buffer_[1] = speed >> 8;
      send_buffer_[2] = speed;
      send_buffer_[3] = (-speed) >> 8;
      send_buffer_[4] = (-speed);
      break;

    case TURN_RIGHT:
      state_->direction = direction;
      state_->turn_speed = GetSpeed(state_->turn_speed, speed, smooth);
      speed = state_->turn_speed;
      send_buffer_[0] = 145;
      send_buffer_[1] = (-speed) >> 8;
      send_buffer_[2] = (-speed);
      send_buffer_[3] = speed >> 8;
      send_buffer_[4] = speed;
      break;

    default:
      send_buffer_[0] = 145;
      send_buffer_[1] = 0;
      send_buffer_[2] = 0;
      send_buffer_[3] = 0;
      send_buffer_[4] = 0;
      break;
  }
  SerialSend(serial_fd_, send_buffer_.get(), 5);
}

bool iRobot::
ReadOdometry(void) {
  send_buffer_[0] = 149;
  send_buffer_[1] = 3;
  send_buffer_[2] = 19;
  send_buffer_[3] = 20;
  send_buffer_[4] = 39;
  SerialSend(serial_fd_, send_buffer_.get(), 5);
  // sleep 50ms
  usleep(50);

  // FIONREAD
  // Returns the number of bytes in the input buffer.
  int nread = 0;
  ioctl(serial_fd_, FIONREAD, &nread);
  nread = read(serial_fd_, recv_buffer_.get(), nread);
  if (nread != 6) {
    return false;
  }
  sensor_data_->distance = (recv_buffer_[0] << 8) + recv_buffer_[1];
  state_->distance += abs(sensor_data_->distance);
  sensor_data_->angle = ((recv_buffer_[2] << 8) + recv_buffer_[3]) % 360;
  state_->angle += sensor_data_->angle;
  state_->angle %= 360;
  return true;
}

const iRobot::State&
iRobot::GetState(void) const {
  return *state_;
}

const iRobot::SensorData&
iRobot::GetSensorData(void) const {
  return *sensor_data_;
}

} // namespace rgbdslam