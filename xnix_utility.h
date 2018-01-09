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

#ifndef __XNIX_UTILITY__
#define __XNIX_UTILITY__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <setjmp.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

extern int h_errno;     // defined by BIND for DNS erros
// these functions will terminate the current process by calling exit(0)
void unix_error(char *msg);
void posix_error(int code, char *msg);
void dns_error(char *msg);
void ai_error(int code);
void app_error(char *msg);
#define app_error(args...) fprintf(stderr, args)

// Process control wrappers
extern char **environ;  // defined by libc
pid_t Fork(void);
void Execve(const char* filename, char *const argv[], char *const envp[]);
pid_t Wait(int *status);
pid_t Waitpid(pid_t pid, int *iptr, int options);
void Kill(pid_t pid, int signum);
unsigned int Sleep(unsigned int secs);
void Pause(void);
unsigned int Alarm(unsigned int seconds);
void Setpgid(pid_t pid, pid_t pgid);
pid_t Getpgrp();

// Signal wrappers
typedef void handler_t(int);
handler_t *Signal(int signum, handler_t *handler);
void Sigprocmask(int how, const sigset_t *set, sigset_t *oldset);
void Sigemptyset(sigset_t *set);
void Sigfillset(sigset_t *set);
void Sigfillset(sigset_t *set);
void Sigaddset(sigset_t *set, int signum);
void Sigdelset(sigset_t *set, int signum);
int Sigismember(const sigset_t *set, int signum);

// Unix I/O Wrappers
int Open(const char *pathname, int flags, mode_t mode);
ssize_t Read(int fd, void *buf, size_t count);
ssize_t Write(int fd, const void *buf, size_t count);
off_t Lseek(int fildes, off_t offset, int whence);
void Close(int fd);
int Select(int n, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
           struct timeval *timeout);
int Dup2(int fd1, int fd2);
void Stat(const char *filename, struct stat *buf);
void Fstat(int fd, struct stat *buf);

// Memory mapping wrappers
void *Mmap(void *addr, size_t len, int prot, int flags, int fd, off_t offset);
void Munmap(void *start, size_t length);

// Dynamic storage allocation wrappers
void *Malloc(size_t size);
void *Realloc(void *ptr, size_t size);
void *Calloc(size_t nmemb, size_t size);
void Free(void *ptr);


// RIO (Robust I/O)
// Unbuffered input and output functions for reading and writting
// binary data to and from network
// return value:
// on success return the actually bytes read, otherwise return -1
ssize_t rio_readn(int fd, void *usrbuf, size_t n);
ssize_t rio_writen(int fd, void *usrbuf, size_t n);

// Buffered input and output functions
#define RIO_BUFFER 8192
typedef struct {
  int rio_fd;
  int rio_cnt;
  char *rio_bufptr;
  char rio_buf[RIO_BUFFER];
} rio_t;

// init the buffer
void rio_readinitb(rio_t *rp, int fd);
// read a line per call
ssize_t rio_readlineb(rio_t *rp, void *usrbuf, size_t maxlen);
// read n bytes per call
ssize_t rio_readnb(rio_t *rp, void *usrbuf, size_t n);

// Network Wrapper
// Create a server socket and bind it to a specific port
// 1. Input:
//  <0> port
//  <1> type
//    - AF_INET  : IPv4
//    - AF_INET6 : IPv6
//  <2> backlog : specify the max length of connection pending queue. 10 is recommened by default.
// 2. Output:
//  <1> ret : Server Socket FD if success else -1
int CreateServerSocket(const char *port, int type, int backlog);

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
int Accept(int sock_fd, int timeout, int retry, char *client_addr);

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
int ConnectTo(const char* host, const char* port, int timeout, int retry);

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
int SocketSend(int sock_fd, const char *buffer, size_t *size, int timeout, int retry);

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
typedef enum {
  WAIT_ALL_OR_TIMEOUT = 0,  // Wait all the data arrive or timeout
  DONT_WAIT_ALL_DATA = 1,   // Return immediately first time get the data
}RecvFlag;
int SocketRecv(int sock_fd, char *buffer, size_t *size, RecvFlag flag,
               int timeout, int retry);
int SetSockBlocking(int sock_fd);
int SetSockNonBlocking(int sock_fd);

#ifdef __cplusplus
}
#endif

#endif