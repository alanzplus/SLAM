#include "message.hpp"
#include "xnix_utility.h"
#include <sstream>
using namespace std;
namespace Com {

string
Message::SSMapToString(const SSMap& ssmap) {
  ostringstream oss;
  for (const auto& amap : ssmap) {
    oss << amap.first << ":" << amap.second << "\r\n";
  }
  return oss.str();
}

Message::SSMap
Message::StringToSSMap(const string& str) {
  SSMap ssmap;
  istringstream iss(str);
  string line;
  while (iss >> line) {
    size_t idx = line.find(':');
    if (idx == string::npos) {
      return SSMap();
    }
    ssmap[line.substr(0,idx)] = line.substr(idx+1);
  }
  return ssmap;
}

bool Message::
SetRawContent(RawContent raw_content, size_t size) {
  raw_content_ = raw_content;
  header_["content-type"] = "raw";
  header_["content-length"] = to_string(size);
  return true;
}

std::string
Message::ContentToString() const {
  try {
    if (header_.at("content-type") == "dict") {
      return SSMapToString(dict_content_);
    } else {
      return std::string((const char*)raw_content_,
                          std::stoll(header_.at("content-length")));
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return std::string();
  }
}

size_t Message::
GetDictContentSize() const {
  size_t ret = 0;
  for (auto it = dict_content_.begin(); it != dict_content_.end(); ++it) {
    ret += (it->first).size();
    ret += (it->second).size();
    ret += 3;
  }
  return ret;
}

Messenger::Options
Messenger::GetDefaultOptions(void) {
  Options ret;
  ret.buffer_size = 640 * 480 * 4;
  ret.timeout = 300;
  return ret;
}

Messenger::Messenger(const Options& options)
  : options_(options),
    buffer_(new uint8_t[options.buffer_size]) {
  ;
}

Messenger::~Messenger() {
  ;
}

int32_t Messenger::
Send(const int32_t sock_fd, const Message& message) {
  auto& header = message.header();
  string header_str(message.HeaderToString());
  header_str += "\r\n";

  auto buffer = buffer_.get();
  memcpy(buffer, header_str.data(), header_str.size());
  size_t total_size = header_str.size();
  if (header.at("content-type") == "dict") {
    string content_str(message.ContentToString());
    memcpy(buffer + total_size, content_str.data(), content_str.size());
    total_size += content_str.size();
  } else if (header.at("content-type") == "raw") {
    size_t raw_content_size = stoll(header.at("content-length"));
    memcpy(buffer + total_size, message.raw_content(), raw_content_size);
    total_size += raw_content_size;
  } else {
    return -1;
  }
  int32_t ret = SocketSend(sock_fd, (const char*)buffer, &total_size, 1, 0);
  return ret;
}

// return -1 means socket error need to close socket and try reconnect
int32_t Messenger::
Recv(const int32_t sock_fd, Message& message, int32_t delay) {
  Message::Header& header = message.header();
  header.clear();
  size_t header_size = 0;
  size_t content_size = 0;
  size_t size = 0;
  size_t total_recv_size = 0;
  int32_t recv_ret = 0;
  string str;
  // points to the first free space in the internal buffer
  uint8_t* buffer = buffer_.get();
  // points to the split line between header and content
  const char* pSplitLine = NULL;
  const char* p = NULL;
  MessageRecvState state = WAIT_FOR_HEADER;
  while (1) {
    switch (state) {
      case WAIT_FOR_HEADER:
        size = 1000;
        recv_ret = SocketRecv(sock_fd, (char*)buffer, &size, DONT_WAIT_ALL_DATA,
                                            delay, 0);
        if (recv_ret != 1) {
          // cerr << "WAIT_FOR_HEADER ERROR:" << recv_ret << endl << flush;
          return -1;
        }
        total_recv_size += size;
        buffer[size] = '\0';
        // try to find the end of the header
        p = strstr((const char*)buffer_.get(), "\r\n\r\n");
        // if we cannot find the header, start to wait again
        state = p == NULL ? WAIT_FOR_HEADER : PROCESS_HEADER;
        // move the buffer
        buffer += size;
        break;

      case PROCESS_HEADER:
        // \r   \n  \r  \n
        //  ^       ^
        //  |       |
        //  p     pSplitLine
        pSplitLine = p + 2;
        header_size = pSplitLine - (const char*)buffer_.get();
        header = Message::StringToSSMap(
          string((const char*)buffer_.get(), header_size));
        content_size = stoll(header["content-length"]);
        // get the left size
        size = header_size + 2 + content_size - total_recv_size;
        state = WAIT_FOR_CONTENT;
        break;

      case WAIT_FOR_CONTENT:
        if (SocketRecv(sock_fd, (char*)buffer, &size,
                                     WAIT_ALL_OR_TIMEOUT,
                                     300000, 0) != 1) {
          return -1;
        }
        state = PROCESS_CONTENT;
        break;

      case PROCESS_CONTENT:
        Message::RawContent& raw_content = message.raw_content();
        // points to the start of the content
        raw_content = buffer_.get() + header_size + 2;
        if (header["content-type"] == "dict") {
          Message::Dict& dict = message.dict_content();
          dict.clear();
          dict = Message::StringToSSMap(
            string((const char*)raw_content, content_size));
        }
        return 1;
        break;
    }
  }
}

} // namespace Com