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

#include "xnix_utility.h"

#ifdef DEGBUG
  #define DebugStr(args...) fprintf(stderr, args)
#else
  #define DebugStr(args...)
#endif

// namespace xnix_utility {
static void* get_in_addr(struct sockaddr *sa);
static ssize_t rio_read(rio_t *rp, char *usrbuf, size_t n);
// Error Handling
void unix_error(char *msg) {
  fprintf(stderr, "%s: %s\n", msg, strerror(errno));
  exit(0);
}

void posix_error(int code, char *msg) {
  fprintf(stderr, "%s: %s\n", msg, strerror(code));
  exit(0);
}

void dns_error(char *msg) {
  fprintf(stderr, "%s: DNS error %d\n", msg, h_errno);
  exit(0);
}

void ai_error(int code) {
  fprintf(stderr, "%s\n", gai_strerror(code));
}

void Close(int fd) {
  if (close(fd) < 0) {
    unix_error("Close error");
  }
}

void *Malloc(size_t size) {
  void *ret;
  if (!(ret = malloc(size))) {
    unix_error("Malloc error");
  }
  return ret;
}

void *Realloc(void *ptr, size_t size) {
  void *p;
  if ((p = realloc(ptr, size)) == NULL) {
    unix_error("Realloc error");
  }
  return p;
}

void Free(void *ptr) {
  free(ptr);
}

// RIO (Robust I/O)
// Unbuffered input and output functions for reading and writting
// binary data to and from network

// return value:
// on success return the actually bytes read, otherwise return -1
ssize_t rio_readn(int fd, void *usrbuf, size_t n) {
  size_t nleft = n;
  ssize_t nread;
  char *bufp = (char*)usrbuf;

  while (nleft > 0) {
    if ((nread = read(fd, bufp, nleft)) < 0) {
      if (errno == EINTR) { // Interrupted by signal handler return
        nread = 0;
      } else {
        return -1; // error
      }
    } else if (nread == 0) { // EOF
      break;
    }

    nleft -= nread;
    bufp += nread;
  }
  return (n - nleft);
}

// return value:
// on success return the actually bytes write, otherwise return -1
ssize_t rio_writen(int fd, void *usrbuf, size_t n) {
  size_t nleft = n;
  ssize_t nwritten;
  char *bufp = (char*)usrbuf;

  while (nleft > 0) {
    if ((nwritten = write(fd, bufp, nleft)) <= 0) {
      if (errno == EINTR) { // Interrupted by signal handler return
        nwritten = 0;
      } else {
        return -1; // error
      }
    }
    nleft -= nwritten;
    bufp += nwritten;
  }
  return n;
}

// return value:
// on success return the actaully bytes read, otherwise return -1
static ssize_t rio_read(rio_t *rp, char *usrbuf, size_t n) {
  int cnt;
  while (rp->rio_cnt <= 0) { // Refill if the buf is empty
    rp->rio_cnt = read(rp->rio_fd, rp->rio_buf, RIO_BUFFER);

    if (rp->rio_cnt < 0) { // Interup
      if (errno != EINTR) return -1;
    } else if (rp->rio_cnt == 0) { // EOF
      return 0;
    } else {
      rp->rio_bufptr = rp->rio_buf;
    }
  }

  // copy data from internal buffer to user buffer
  cnt = n < rp->rio_cnt ? n : rp->rio_cnt;
  memcpy(usrbuf, rp->rio_bufptr, cnt);
  rp->rio_bufptr += cnt;
  rp->rio_cnt -= cnt;
  return cnt;
}

ssize_t rio_readlineb(rio_t *rp, void *usrbuf, size_t maxlen) {
  int n, rc;
  char c, *bufp = (char*)usrbuf;
  // copy at most maxlen - 1 bytes, the left 1 byte for null terminate ch
  for (n = 1; n < maxlen; ++n) {
    if ((rc = rio_read(rp, &c, 1)) == 1) {
      *bufp++ = c;
      if (c == '\n') {
        break;
      }
    } else if (rc == 0) {
      if (n == 1) return 0; // EOF, no data read
      else break;
    } else {
      return -1;
    }
  }
  *bufp = '\0';
  return n;
}

ssize_t rio_readnb(rio_t *rp, void *usrbuf, size_t n) {
  size_t nleft = n;
  ssize_t nread;
  char *bufp = (char*)usrbuf;

  while (nleft > 0) {
    if ((nread = rio_read(rp, bufp, nleft)) < 0) {
      if (errno == EINTR) nread = 0;
      else return -1;
    } else if (nread == 0) {
      break;
    }
    nleft -= nread;
    bufp += nread;
  }
  return (n - nleft);
}

// Create a server socket and bind it to a specific port
// 1. Input:
//  <0> port
//  <1> type
//    - AF_INET  : IPv4
//    - AF_INET6 : IPv6
//  <2> backlog : specify the max length of connection pending queue.
// 2. Output:
//  <1> ret : Server Socket FD if success else -1
int CreateServerSocket(const char *port, int type, int backlog) {
  if (!port) {
    app_error("Server port should be filled.\n");
    return -1;
  }

  // prepare address struct
  struct addrinfo hints, *server_info = NULL;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = type;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  int ret;
  if ((ret = getaddrinfo(NULL, port, &hints, &server_info)) != 0) {
    ai_error(ret);
    return -1;
  }

  // Loop through all the results and bind the first we can
  int sock_fd;
  struct addrinfo *p;
  for (p = server_info; p != NULL; p = p->ai_next) {
    if ((sock_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("server: socket");
      continue;
    }
    // SO_REUSERADDR: Allows other sockets to bind() to this port, unless
    // there is an active listening socket boud to the port already
    int yes = 1;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
      unix_error("setsockopt");
    }
    if (bind(sock_fd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sock_fd);
      perror("server: bind");
      continue;
    }
    break;
  }

  if (!p) {
    app_error("server: failed to create socket on port %s\n", port);
    if (server_info) {
      freeaddrinfo(server_info);
    }
    return -1;
  }

  freeaddrinfo(server_info);

  // backlog sepcifies how many pending connections you can have before the
  // kernel starts rejecting new ones.
  if (listen(sock_fd, backlog)) {
    unix_error("server failed to bind.");
  }

  return sock_fd;
}

// Try to accept a remote connection from client.
// Implement based on accept() and select()
// 1. Input:
//  <1> socket
//  <2> timeout : in ms
//    - if > 0 block for timeout us
//    - if = 0 non-blocking
//    - if < 0 block forever if no remote connection
//  <3> retry : currently does not support
//  <4> client_addr: a pointer to a str buffer, length at least 100
// 2. Output:
//  <1> client_addr : client ip address string
//  <2> ret : client socket if success else -1
int Accept(int sock_fd, int timeout, int retry, char *client_addr) {
  struct timeval tv, *tv_ptr;
  if (timeout < 0) {
    tv_ptr = NULL;
  } else {
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = timeout % 1000;
    tv_ptr = &tv;
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(sock_fd, &read_fds);
  switch(select(sock_fd + 1, &read_fds, NULL, NULL, tv_ptr)) {
    case 0: return 0; // timeout
    case -1: return -1; // error
  }

  struct sockaddr_storage addr;
  socklen_t addr_size = sizeof(addr);
  int client_fd = accept(sock_fd, (struct sockaddr *)&addr, &addr_size);

  if (client_fd == -1) {
    unix_error("Accpet:");
  }

  const int MAXSIZE = 100;
  if (client_addr) {
    inet_ntop(addr.ss_family, get_in_addr((struct sockaddr *)&addr),
              client_addr, MAXSIZE);
    DebugStr("Server: got connections from %s\n", client_addr);
  }

  return client_fd;
}
// Try to connect to a remote sever using SOCK_STREAM
// 1. Input:
//  <1> host : server host name or ip address
//  <2> port : server port
//  <3> timeout : in ms
//    - if < 0 block forever
//    - if = 0 non-block
//    - if > 0 timeout
//  <4> retry : currently doesn't support
// 2. Output
//  <2> if success return sock_fd, else return -1
int ConnectTo(const char* host, const char* port, int timeout, int retry) {
  if (!host || !port) {
    app_error("server address or port cannot be empty!.\n");
    return -1;
  }

  // given server address and port number, return the basic info about the
  // server
  struct addrinfo hints, *server_info = NULL;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  int ret;
  if ((ret = getaddrinfo(host, port, &hints, &server_info)) != 0) {
    ai_error(ret);
    return -1;
  }

  // Timeout Settings
  struct timeval tv, *tv_ptr;
  if (timeout < 0) {
    tv_ptr = NULL;
  } else {
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = timeout % 1000;
    tv_ptr = &tv;
  }

  // Loop through all the results and connect to the first we can
  fd_set write_fds;
  int sock_fd;
  struct addrinfo* p;
  for (p = server_info; p != NULL; p = p->ai_next) {
    if ((sock_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("client: socket");
      continue;
    }

    if (SetSockNonBlocking(sock_fd) < 0) {
      close(sock_fd);
      unix_error("ConnectTo: SetSockNonBlocking:");
    }

    if (connect(sock_fd, p->ai_addr, p->ai_addrlen) < 0) {
      if (errno == EINPROGRESS) {
        FD_ZERO(&write_fds);
        FD_SET(sock_fd, &write_fds);
        switch (select(sock_fd + 1, NULL, &write_fds, NULL, tv_ptr)) {
          case -1:
            perror("ConnectTo: select:");
            close(sock_fd);
            continue;

          case 0:
          DebugStr("ConnectTo: select: timeout\n");
            close(sock_fd);
            continue;
        }

        int error;
        socklen_t len = sizeof(error);

        if (getsockopt(sock_fd, SOL_SOCKET, SO_ERROR, &error, &len) < 0)  {
          perror("ConnectTo: getsockopt");
          close(sock_fd);
          continue;
        }

        if (error) {
          DebugStr("ConnectTo: %s\n", strerror(error));
          close(sock_fd);
          continue;
        }

      } else {
        DebugStr("ConnectTo: connect:");
        close(sock_fd);
        continue;
      }
    }
    break;
  }

  if (!p) {
    DebugStr("failed to connect to (%s, %s)\n", host, port);
    if (server_info) {
      freeaddrinfo(server_info);
    }
    return -1;
  }

  if (SetSockBlocking(sock_fd) < 0) {
    close(sock_fd);
    unix_error("ConnectTo: SetSockBlocking");
  }

  {
    const int MAXSIZE = 100;
    char *str = (char*)Malloc(MAXSIZE);
    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
              str, MAXSIZE);
    DebugStr("connected to %s\n", str);
    freeaddrinfo(server_info);
    Free(str);
  }
  return sock_fd;
}

// Implement based on write() and select(). It supports block and non-block
// 1. Input:
//  <1> sock_fd
//  <2> buffer
//  <3> size : point to size, specify the number of bytest want to send
//  <4> timeout : in ms
//    - if < 0, block forever if socket buffer is not enough to hold the user's buffer data
//    - if = 0, will not block and will return immediately
//    - if > 0, will block for a timeout ms if the socket buffer is not enough to hold user's buffer data
// timeout > 0, Send will block for a timeout us if the socket buffer is not
// enough to hold user's buffer data
//  <5> retry : currently doesn't support
// 2. Output:
//  <1> size : return the actual bytes copy into the socket buffer
//  <2> ret
//    - 0 timeout
//    - -1 peer close the socket
//    - 1 all data are copied into socket buffer
// 3. Note
//  <1> Writting to connection that has benn closed by the peer FIRST TIME elicits
//  an error with errno set to EPIPE. Writting to such a connection a second time
//  elicits a SIGPIPE signal whose default action is to terminate the process.
//  <2> When some unix internal errors occur, SocketSend will terminate the program
int SocketSend(int sock_fd, const char *buffer, size_t *size,
               int timeout, int retry) {
  struct timeval tv, *tv_ptr;
  if (timeout < 0) {
    tv_ptr = NULL;
  } else {
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = timeout % 1000;
    tv_ptr = &tv;
  }
  fd_set write_fds;
  int cnt = 0;
  while (cnt < *size) {
    FD_ZERO(&write_fds);
    FD_SET(sock_fd, &write_fds);
    switch(select(sock_fd + 1, NULL, &write_fds, NULL, tv_ptr)) {
      case 0:
        *size = cnt;
        return 0;
      case -1:
        *size = cnt;
        unix_error("SocketSend: select");
        return -1;
    }

    int n = write(sock_fd, buffer + cnt, *size - cnt);
    if (errno == EPIPE) {
      return -1 ;
    } else if (n < 0) {
      unix_error("SocketSend: write");
    }
    cnt += n;
  }
  *size = cnt;
  return 1;
}

// Read data from socket stream.
// Implement based on read() and select(). It supports block and non-block
// 1. Input:
//  <1> sock_fd
//  <2> buffer
//  <3> size : a pointer to size_t, specify the number of bytes want to receive
//  <4> flag
//    - WAIT_ALL_OR_TIMEOUT : Wait all the data arrive or timeout
//    - DONT_WAIT_ALL_DATA  : Return immediately first time get the data
//  <5> timeout : timeout in ms
//    - if timeout < 0, Recv will block forever if socket buffer does not contain any data.
//    - if timeout = 0, Recv will not block and will return immediately.
//    - if timeout > 0, Recv will block for a timeout ms if the socket buffer does not contain any data.
//  <6> retry : currently doesn't support
// 2. Output
//  <1> size : *size the actual bytes copy from socket buffer
//  <2> ret
//    - -1 sender close the socket
//    - 0 timeout
//    - 1 all data are copied into socket buffer if flag = WAIT_ALL_OR_TIMEOUT
//      or some data are copied into socket buffer if flag = DONT_WAIT_ALL_DATA
// 3. Note
//  <1> When some unix internal errors occur, SocketRecv will terminate the program
int SocketRecv(int sock_fd, char *buffer, size_t *size, RecvFlag flag,
               int timeout, int retry) {
  struct timeval tv, *tv_ptr;
  if (timeout < 0) {
    tv_ptr = NULL;
  } else {
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = timeout % 1000;
    tv_ptr = &tv;
  }

  fd_set read_fds;
  int cnt = 0;
  while (cnt < *size) {
    FD_ZERO(&read_fds);
    FD_SET(sock_fd, &read_fds);
    switch(select(sock_fd + 1, &read_fds, NULL, NULL, tv_ptr)) {
      case 0:   // timeout
        *size = cnt;
        return 0;

      case -1:  // internal error
        *size = cnt;
        unix_error("SocketRecv: select");

      default:
        break;
    }
    // read will not block since buffer has data, though not enough
    int n = read(sock_fd, buffer + cnt, *size - cnt);

    if (n < 0) {
      unix_error("SocketRecv: read");
    }

    cnt += n;

    // when the sender close the socket, read will return 0
    if (n == 0) {
      *size = cnt;
      return -1;
    }

    if (flag == DONT_WAIT_ALL_DATA) {
      *size = cnt;
      return 1;
    }
  }
  *size = cnt;
  return 1;
}

// Set socket as block
int SetSockBlocking(int sock_fd) {
  int flags;
  if (-1 == (flags = fcntl(sock_fd, F_GETFL, 0))) {
    flags = 0;
  }
  return fcntl(sock_fd, F_SETFL, flags & ~O_NONBLOCK);
}

// Set socket as non-block
int SetSockNonBlocking(int sock_fd) {
  int flags;
  if (-1 == (flags = fcntl(sock_fd, F_GETFL, 0))) {
    flags = 0;
  }
  return fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK);
}

static void* get_in_addr(struct sockaddr *sa) {
  if (sa->sa_family == AF_INET) {
    return &(((struct sockaddr_in *)sa)->sin_addr);
  } else {
    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }
}

// } // namespace xnix_utility