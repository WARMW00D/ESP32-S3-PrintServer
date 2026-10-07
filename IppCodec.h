#pragma once
// Bounded IPP wire codec (RFC 8010). No Arduino dependencies: host-testable.
#include "PwgRaster.h"
#include <vector>
#include <string>

namespace mobileprint {
inline uint16_t be16(const uint8_t *p) { return uint16_t(p[0]) * 256 + p[1]; }
enum Operation : uint16_t {
  PrintJob = 2, ValidateJob = 4, CreateJob = 5, SendDocument = 6,
  CancelJob = 8, GetJobAttributes = 9, GetJobs = 10, GetPrinterAttributes = 11
};
enum Status : uint16_t {
  Ok = 0, BadRequest = 0x0400, NotFound = 0x0406, NotPossible = 0x0404,
  FormatUnsupported = 0x040a, AttributesUnsupported = 0x040b,
  InternalError = 0x0500, OperationUnsupported = 0x0501,
  PrinterBusy = 0x0507, ServiceUnavailable = 0x0502
};
struct Request {
  uint8_t major = 2, minor = 0;
  uint16_t operation = 0, status = Ok;
  uint32_t id = 0, jobId = 0;
  std::string format = "image/pwg-raster", name = "Mobile print", user = "anonymous";
  std::string sides = "one-sided";
  bool lastDocument = true;
  std::string whichJobs = "not-completed";
  bool myJobs = false;
};

class RequestParser {
 public:
  void reset() { bytes_.clear(); cursor_ = 0; group_ = 0; collection_ = 0; done_ = false; req_ = Request(); previous_.clear(); member_.clear(); }
  const Request &request() const { return req_; }
  bool done() const { return done_; }
  // 0: more metadata; 1: end-of-attributes; -1: malformed/oversized metadata.
  int feed(uint8_t byte) {
    if (done_ || bytes_.size() >= 8192) return -1;
    bytes_.push_back(byte);
    if (bytes_.size() < 8) return 0;
    if (!cursor_) {
      req_.major = bytes_[0]; req_.minor = bytes_[1];
      req_.operation = be16(bytes_.data() + 2); req_.id = be32(bytes_.data() + 4);
      if (!((req_.major == 1 && req_.minor == 1) || (req_.major == 2 && req_.minor == 0)))
        req_.status = 0x0503; // server-error-version-not-supported
      cursor_ = 8;
    }
    while (cursor_ < bytes_.size()) {
      uint8_t tag = bytes_[cursor_];
      if (tag < 0x10) {
        if (collection_) return -1;
        ++cursor_;
        if (tag == 3) { if (group_ == 0) return -1; done_ = true; return 1; }
        if (tag != 1 && tag != 2) return -1;
        group_ = tag; previous_.clear(); continue;
      }
      if (!group_) return -1;
      size_t left = bytes_.size() - cursor_;
      if (left < 3) return 0;
      uint16_t nameLen = be16(bytes_.data() + cursor_ + 1);
      if (nameLen > 255) return -1;
      if (left < size_t(5 + nameLen)) return 0;
      uint16_t valueLen = be16(bytes_.data() + cursor_ + 3 + nameLen);
      if (valueLen > 4096) return -1;
      if (left < size_t(5 + nameLen + valueLen)) return 0;
      const uint8_t *value = bytes_.data() + cursor_ + 5 + nameLen;
      std::string name = nameLen ? std::string(reinterpret_cast<const char *>(bytes_.data() + cursor_ + 3), nameLen) : previous_;
      if (name.empty()) return -1;
      if (tag == 0x34) ++collection_;
      else if (tag == 0x37) { if (!collection_) return -1; --collection_; }
      else if (collection_) {
        if (tag == 0x4a) member_ = std::string(reinterpret_cast<const char *>(value), valueLen);
        else if (member_ == "x-dimension" || member_ == "y-dimension") {
          uint32_t expected = member_ == "x-dimension" ? 21000 : 29700;
          if (tag != 0x21 || valueLen != 4 || be32(value) != expected) req_.status = AttributesUnsupported;
        } else if (member_.find("media-") == 0 && member_.find("-margin") != std::string::npos) {
          if (tag != 0x21 || valueLen != 4 || be32(value) != 635) req_.status = AttributesUnsupported;
        }
      } else parse(name, tag, value, valueLen);
      previous_ = name;
      cursor_ += 5 + nameLen + valueLen;
    }
    return 0;
  }

 private:
  std::vector<uint8_t> bytes_;
  size_t cursor_ = 0;
  uint8_t group_ = 0;
  unsigned collection_ = 0;
  bool done_ = false;
  Request req_;
  std::string previous_, member_;
  void parse(const std::string &name, uint8_t tag, const uint8_t *p, size_t n) {
    const std::string value(reinterpret_cast<const char *>(p), n);
    auto unsupported = [&]() { if (!req_.status) req_.status = AttributesUnsupported; };
    if (name == "document-format") {
      if (tag != 0x49) { req_.status = BadRequest; return; }
      req_.format = value;
      if (value != "image/pwg-raster") req_.status = FormatUnsupported;
    } else if (name == "job-id") {
      if (tag != 0x21 || n != 4) { req_.status = BadRequest; return; }
      req_.jobId = be32(p);
    } else if (name == "job-uri") {
      size_t slash = value.rfind('/');
      if (slash == std::string::npos || slash + 1 == value.size()) { req_.status = BadRequest; return; }
      uint64_t id = 0;
      for (size_t i = slash + 1; i < value.size(); ++i) {
        if (value[i] < '0' || value[i] > '9') { req_.status = BadRequest; return; }
        id = id * 10 + value[i] - '0';
        if (id > UINT32_MAX) { req_.status = BadRequest; return; }
      }
      req_.jobId = uint32_t(id);
    } else if (name == "last-document") {
      if (tag != 0x22 || n != 1) { req_.status = BadRequest; return; }
      req_.lastDocument = p[0] != 0;
    } else if (name == "job-name") req_.name = value.substr(0, 128);
    else if (name == "requesting-user-name") req_.user = value.substr(0, 128);
    else if (name == "attributes-charset" && value != "utf-8") req_.status = BadRequest;
    else if (name == "which-jobs") {
      req_.whichJobs = value;
      if (value != "all" && value != "completed" && value != "not-completed") unsupported();
    } else if (name == "my-jobs") {
      if (tag != 0x22 || n != 1) req_.status = BadRequest;
      else req_.myJobs = p[0] != 0;
    }
    else if (name == "copies" && (tag != 0x21 || n != 4 || be32(p) != 1)) unsupported();
    else if (name == "media" && value != "iso_a4_210x297mm") unsupported();
    else if (name == "sides") {
      if (value != "one-sided" && value != "two-sided-long-edge" && value != "two-sided-short-edge") unsupported();
      else req_.sides = value;
    }
    else if (name == "print-color-mode" && value != "monochrome" && value != "auto") unsupported();
    else if (name == "printer-resolution" &&
             (tag != 0x32 || n != 9 || (be32(p) != 300 && be32(p) != 600) || be32(p+4) != be32(p) || p[8] != 3)) unsupported();
    else if (name == "number-up" && (tag != 0x21 || n != 4 || be32(p) != 1)) unsupported();
    else if (name == "finishings" && (tag != 0x23 || n != 4 || be32(p) != 3)) unsupported();
    else if (name == "print-quality" && (tag != 0x23 || n != 4 || (be32(p) != 4 && be32(p) != 5))) unsupported();
    else if (name == "orientation-requested" && (tag != 0x23 || n != 4 || be32(p) != 3)) unsupported();
    else if (name == "compression" && value != "none") unsupported();
    else if (name == "job-hold-until" && value != "no-hold") unsupported();
    // Client-side page selection/scaling/orientation is already present in raster.
    // media-col is a collection: PWG page geometry is independently validated.
  }
};

class Response {
 public:
  std::vector<uint8_t> bytes;
  explicit Response(const Request &r, uint16_t status = Ok) {
    bytes = {r.major, r.minor}; u16(status); u32(r.id);
    group(1); text(0x47, "attributes-charset", "utf-8");
    text(0x48, "attributes-natural-language", "en");
  }
  void group(uint8_t tag) { bytes.push_back(tag); }
  void u16(uint16_t n) { bytes.push_back(n >> 8); bytes.push_back(n); }
  void u32(uint32_t n) { bytes.push_back(n >> 24); bytes.push_back(n >> 16); bytes.push_back(n >> 8); bytes.push_back(n); }
  void attr(uint8_t tag, const char *name, const uint8_t *p, size_t n) {
    bytes.push_back(tag); size_t len = strlen(name); u16(uint16_t(len));
    bytes.insert(bytes.end(), name, name + len); u16(uint16_t(n)); bytes.insert(bytes.end(), p, p+n);
  }
  void text(uint8_t tag, const char *name, const std::string &s) { attr(tag, name, reinterpret_cast<const uint8_t *>(s.data()), s.size()); }
  void integer(const char *name, uint32_t n, uint8_t tag = 0x21) {
    uint8_t p[4] = {uint8_t(n >> 24), uint8_t(n >> 16), uint8_t(n >> 8), uint8_t(n)}; attr(tag, name, p, 4);
  }
  void boolean(const char *name, bool v) { uint8_t b = v; attr(0x22, name, &b, 1); }
  void range(const char *name, uint32_t low, uint32_t high) {
    uint8_t p[8] = {uint8_t(low>>24),uint8_t(low>>16),uint8_t(low>>8),uint8_t(low),uint8_t(high>>24),uint8_t(high>>16),uint8_t(high>>8),uint8_t(high)};
    attr(0x33, name, p, 8);
  }
  void resolution(const char *name, uint32_t dpi = 600) {
    const uint8_t p[] = {uint8_t(dpi>>24),uint8_t(dpi>>16),uint8_t(dpi>>8),uint8_t(dpi),
                        uint8_t(dpi>>24),uint8_t(dpi>>16),uint8_t(dpi>>8),uint8_t(dpi),3};
    attr(0x32, name, p, 9);
  }
  void beginCollection(const char *name) { attr(0x34, name, reinterpret_cast<const uint8_t *>(""), 0); }
  void member(const char *name) { text(0x4a, "", name); }
  void endCollection() { attr(0x37, "", reinterpret_cast<const uint8_t *>(""), 0); }
  void end() { group(3); }
};
} // namespace mobileprint
