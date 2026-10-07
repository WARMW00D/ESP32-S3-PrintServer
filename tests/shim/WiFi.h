#pragma once
#include <stdint.h>
#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cctype>

inline uint32_t testMillis = 0;
inline uint32_t millis() { return testMillis; }
class String {
  std::string s_;
 public:
  String() = default;
  String(const char *s) : s_(s) {}
  String(std::string s) : s_(std::move(s)) {}
  String &operator=(const char *s) { s_ = s; return *this; }
  const char *c_str() const { return s_.c_str(); }
  size_t length() const { return s_.size(); }
  char operator[](size_t i) const { return s_[i]; }
  int indexOf(char c, int pos=0) const { auto n=s_.find(c,pos); return n==s_.npos ? -1 : int(n); }
  bool startsWith(const char *p) const { return s_.rfind(p,0)==0; }
  bool endsWith(const char *p) const { size_t n=std::char_traits<char>::length(p); return s_.size()>=n && s_.compare(s_.size()-n,n,p)==0; }
  String substring(int a, int b) const { return s_.substr(a,b-a); }
  String substring(int a) const { return s_.substr(a); }
  void trim() { auto a=s_.find_first_not_of(" \t\r\n"); if(a==s_.npos){s_.clear();return;} auto b=s_.find_last_not_of(" \t\r\n"); s_=s_.substr(a,b-a+1); }
  void toLowerCase() { for(char &c:s_) c=char(std::tolower(static_cast<unsigned char>(c))); }
  bool operator==(const char *s) const { return s_==s; }
  bool operator!=(const char *s) const { return s_!=s; }
  String &operator+=(char c) { s_+=c; return *this; }
  String &operator+=(const String &s) { s_+=s.s_; return *this; }
};
struct Socket {
  std::vector<uint8_t> input, output;
  size_t pos=0;
  bool open=true;
};
class WiFiClient {
  std::shared_ptr<Socket> socket_;
 public:
  WiFiClient() = default;
  explicit WiFiClient(std::shared_ptr<Socket> p) : socket_(std::move(p)) {}
  explicit operator bool() const { return bool(socket_); }
  int available() const { return socket_ ? int(socket_->input.size()-socket_->pos) : 0; }
  int read() { return available() ? socket_->input[socket_->pos++] : -1; }
  bool connected() const { return socket_ && socket_->open; }
  void stop() { if(socket_) socket_->open=false; }
  size_t write(const uint8_t *p,size_t n) { socket_->output.insert(socket_->output.end(),p,p+n); return n; }
  void print(const char *s) { write(reinterpret_cast<const uint8_t *>(s),strlen(s)); }
  void printf(const char *s,...) { char b[1024]; va_list a;va_start(a,s);int n=vsnprintf(b,sizeof(b),s,a);va_end(a);write(reinterpret_cast<const uint8_t *>(b),n); }
};
class WiFiServer {
 public:
  inline static std::deque<std::shared_ptr<Socket>> incoming;
  explicit WiFiServer(uint16_t) {}
  void begin() {}
  void setNoDelay(bool) {}
  WiFiClient available() { if(incoming.empty())return {};auto p=incoming.front();incoming.pop_front();return WiFiClient(p); }
};
struct IPAddress { String toString() const { return "192.168.4.1"; } };
constexpr int WL_CONNECTED=3;
struct WiFiClass { int status() const{return WL_CONNECTED;} IPAddress localIP()const{return {};} IPAddress softAPIP()const{return {};} };
inline WiFiClass WiFi;
struct SerialClass { void printf(const char *,...) {} };
inline SerialClass Serial;
