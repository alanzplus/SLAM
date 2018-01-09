#include "message.hpp"
#include "xnix_utility.h"
#include <iostream>
#include <string>
#include <vector>
#include <signal.h>
#include <iostream>
#include <utility>
#include <memory>
using namespace std;
using namespace Com;
// Display what client type in
void PrintServer(char * argv[]) {
  int32_t sock_fd = CreateServerSocket(argv[0], AF_INET, 10);
  unique_ptr<char[]> str(new char[100]);
  int32_t client_fd = Accept(sock_fd, -1, 0, str.get());
  Messenger messenger;
  while (1) {
    Message message;
    switch (messenger.Recv(client_fd, message)) {
      case -1:
        cout << "Socket Error" << endl;
        exit(-1);
    }
    cout << message.ToString() << endl;
  }
}

void sig_handler(int signo) {
  if (signo == SIGPIPE) {
    cout << "Server closed the socket." << endl << flush;
  }
  exit(-1);
}

// Send what user type in to the remote server
void PrintClient(char * argv[]) {
  int32_t sock_fd = ConnectTo(argv[0], argv[1], 2000, 0);
  if (signal(SIGPIPE, sig_handler) == SIG_ERR) {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        exit(-1);
  }
  Messenger messenger;
  while (1) {
    Message message;
    string str;
    cout << "Please enter Header:" << endl;
    while (1) {
      string ss;
      getline(cin, ss);
      if (ss == "") break;
      ss += "\r\n";
      str += ss;
    }
    message.header() = Message::StringToSSMap(str);
    cout << "Please enter content-type: raw / dict" << endl;
    string type;
    getline(cin, type);
    if (type == "raw") {
      string content;
      cout << "Enter Raw Content:";
      getline(cin, content);
      message.SetRawContent(content.data(), content.size());
    } else if (type == "dict") {
      cout << "Enter Dict Content:";
      string content;
      while (1) {
        string line;
        getline(cin, line);
        if (line == "") break;
        line += "\r\n";
        content += line;
      }
      Message::Header& header = message.header();
      Message::Dict& dict = message.dict_content();
      dict.clear();
      dict = Message::StringToSSMap(content);
      header["content-type"] = "dict";
      header["content-length"] = to_string(message.GetDictContentSize());
    }
    switch (messenger.Send(sock_fd, message)) {
      case -1:
        cout << "Error" << endl;
        exit(-1);
    }
  }
}

int main(int argc, char *argv[])
{
  if (argc == 1) {
    return 0;
  } else {
    if (string(argv[1]) == string("server")) {
      PrintServer(argv+2);
    } else {
      PrintClient(argv+2);
    }
  }
}
