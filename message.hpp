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

#ifndef __MESSAGE__
#define __MESSAGE__

#include <string>
#include <iostream>
#include <map>
namespace Com {

class Message {
 public:
  typedef std::map<std::string, std::string> SSMap;
  typedef SSMap Header;
  typedef SSMap Dict;
  typedef const void* RawContent;
  typedef const void* const ConstRawContent;
  Message() : raw_content_(nullptr) {}
  ~Message() {
    raw_content_ = nullptr;
  }

  static std::string SSMapToString(const SSMap& ssmap);
  static SSMap StringToSSMap(const std::string& str);

  Header& header() {
    return header_;
  }

  const Header& header() const {
    return header_;
  }

  Dict& dict_content() {
    return dict_content_;
  }

  const Dict& dict_content() const {
    return dict_content_;
  }

  RawContent& raw_content() {
    return raw_content_;
  }

  ConstRawContent& raw_content() const {
    return raw_content_;
  }

  bool SetRawContent(RawContent raw_content, size_t size);
  size_t GetDictContentSize() const;

  std::string ToString() const {
    return HeaderToString() + ContentToString();
  }

  bool FromString(const std::string& str);

  std::string HeaderToString() const {
    return SSMapToString(header_);
  }

  std::string ContentToString() const;

 private:
  Header header_;
  Dict dict_content_;
  RawContent raw_content_; // borrow pointer
};

class Messenger {
 private:
  typedef enum {
    WAIT_FOR_HEADER = 1,
    PROCESS_HEADER = 2,
    WAIT_FOR_CONTENT = 3,
    PROCESS_CONTENT = 4
  } MessageRecvState;

 public:
   struct Options {
     int32_t buffer_size;
     int32_t timeout;      // in ms
   };

 public:
   static Options GetDefaultOptions(void);
   Messenger(const Options& options = GetDefaultOptions());
   ~Messenger();
   int32_t Send(const int32_t sock_fd, const Message& message);
   // timeout in micro second -> us
   int32_t Recv(const int32_t sock_fd, Message& message, int32_t delay = -1);

 private:
   std::unique_ptr<uint8_t[]> buffer_;
   Options options_;
};

} // namespace Com
#endif