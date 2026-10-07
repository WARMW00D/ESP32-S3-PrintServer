#pragma once
// Experimental Android IPP endpoint. This is not an AirPrint/Mopria certification.
#include <WiFi.h>
#include <ESPmDNS.h>
#include "IppCodec.h"

namespace mobileprint {
class Server {
 public:
  typedef bool (*Ready)();
  typedef bool (*Flush)();
  typedef void (*Activity)(bool);
  Server(PwgRaster::Writer write, Ready ready, Flush flush, Activity activity)
    : listener_(631), raster_(write), ready_(ready), flush_(flush), activity_(activity) {}
  bool busy() const { return activeId_ != 0; }
  void begin() {
    if (!begun_) { listener_.begin(); listener_.setNoDelay(true); begun_ = true; }
  }
  void advertise(const char *hostname) {
    // Caller has already started mDNS on the current STA or AP interface.
    host_ = hostname;
    MDNS.addService("ipp", "tcp", 631);
    MDNS.addServiceTxt("ipp", "tcp", "txtvers", "1");
    MDNS.addServiceTxt("ipp", "tcp", "qtotal", "1");
    MDNS.addServiceTxt("ipp", "tcp", "rp", "ipp/print");
    MDNS.addServiceTxt("ipp", "tcp", "ty", "HP LaserJet P2015 (ESP32)");
    MDNS.addServiceTxt("ipp", "tcp", "product", "(HP LaserJet P2015)");
    MDNS.addServiceTxt("ipp", "tcp", "pdl", "image/pwg-raster");
    MDNS.addServiceTxt("ipp", "tcp", "Color", "F");
    MDNS.addServiceTxt("ipp", "tcp", "Duplex", "F");
    // Do not advertise _universal / URF / ipp-everywhere before implementing them.
  }
  void abort() {
    if (busy()) failActive("network-reset");
    for (auto &s : slots_) close(s);
  }
  void poll() {
    if (!begun_) return;
    WiFiClient client = listener_.available();
    if (client) {
      Slot *free = nullptr;
      for (auto &s : slots_) if (!s.used) { free = &s; break; }
      if (!free) { client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"); client.stop(); }
      else { free->reset(); free->client = client; free->used = true; free->last = millis(); }
    }
    // Serve query/cancel sockets before progressing the document socket.
    for (auto &s : slots_) if (s.used && s.jobId == 0) service(s);
    for (auto &s : slots_) if (s.used && s.jobId != 0) service(s);
    if (activeId_ && uint32_t(millis() - activeSince_) > 90000 && !documentOpen_) failActive("job-data-timeout");
  }

 private:
  struct Slot {
    WiFiClient client;
    bool used = false, headersDone = false, chunked = false, chunkCR = false;
    bool chunkLF = false, trailers = false, bodyDone = false, accepted = false;
    uint32_t last = 0, remaining = 0, chunkLeft = 0, bodyBytes = 0, jobId = 0;
    String line, headers;
    RequestParser parser;
    Request request;
    void reset() {
      used = headersDone = chunked = chunkCR = chunkLF = trailers = bodyDone = accepted = false;
      last = remaining = chunkLeft = bodyBytes = jobId = 0;
      line = headers = ""; parser.reset(); request = Request();
    }
  } slots_[3];
  struct Job {
    uint32_t id = 0, state = 3, created = 0;
    std::string name, user, reason = "job-incoming";
  } jobs_[4];
  WiFiServer listener_;
  PwgRaster raster_;
  Ready ready_;
  Flush flush_;
  Activity activity_;
  bool begun_ = false, documentOpen_ = false;
  uint32_t activeId_ = 0, activeSince_ = 0, nextId_ = 1;
  String host_ = "esp32-printserver";
  Job *job(uint32_t id) { for (auto &j : jobs_) if (j.id == id && id) return &j; return nullptr; }
  String address() const { return WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString(); }
  std::string uri() const { return std::string("ipp://") + address().c_str() + ":631/ipp/print"; }
  void close(Slot &s) { s.client.stop(); s.reset(); }
  void httpError(Slot &s, const char *status) {
    Serial.printf("[IPP] HTTP error: %s\n", status);
    if (s.jobId && activeId_ == s.jobId) failActive("document-format-error");
    s.client.printf("HTTP/1.1 %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n", status);
    close(s);
  }
  void respond(Slot &s, Response &response) {
    response.end();
    s.client.printf("HTTP/1.1 200 OK\r\nContent-Type: application/ipp\r\nContent-Length: %u\r\nConnection: close\r\n\r\n", unsigned(response.bytes.size()));
    s.client.write(response.bytes.data(), response.bytes.size()); close(s);
  }
  void status(Slot &s, uint16_t code, const char *message = nullptr) {
    Response r(s.request, code);
    if (message) r.text(0x41, "status-message", message);
    respond(s, r);
  }
  void releaseActive() { activeId_ = 0; documentOpen_ = false; activity_(false); }
  void failActive(const char *reason) {
    Job *j = job(activeId_);
    if (j) { j->state = 8; j->reason = reason; }
    raster_.abort(); flush_(); releaseActive();
  }
  void appendJob(Response &r, const Job &j) {
    r.group(2); r.integer("job-id", j.id);
    r.text(0x45, "job-uri", uri() + "/jobs/" + std::to_string(j.id));
    r.text(0x45, "job-printer-uri", uri());
    r.text(0x42, "job-name", j.name); r.text(0x42, "job-originating-user-name", j.user);
    r.integer("job-state", j.state, 0x23); r.text(0x44, "job-state-reasons", j.reason);
    r.integer("time-at-creation", j.created); r.integer("job-printer-up-time", millis()/1000);
  }
  void mediaCollection(Response &r, const char *name) {
    r.beginCollection(name);
    r.member("media-size"); r.beginCollection("");
    r.member("x-dimension"); r.integer("", 21000);
    r.member("y-dimension"); r.integer("", 29700); r.endCollection();
    for (const char *m : {"media-bottom-margin", "media-left-margin", "media-right-margin", "media-top-margin"}) {
      r.member(m); r.integer("", 635);
    }
    r.endCollection();
  }
  void printerAttributes(Response &r) {
    r.group(4);
    r.text(0x45, "printer-uri-supported", uri());
    r.text(0x44, "uri-authentication-supported", "none");
    r.text(0x44, "uri-security-supported", "none");
    r.text(0x42, "printer-name", "HP P2015 ESP32");
    r.text(0x41, "printer-info", "HP LaserJet P2015 via ESP32, experimental IPP");
    r.text(0x41, "printer-make-and-model", "HP LaserJet P2015");
    r.text(0x41, "printer-location", "Local USB");
    r.text(0x45, "printer-more-info", std::string("http://") + address().c_str() + "/");
    r.integer("printer-up-time", millis()/1000);
    r.integer("printer-state", busy() ? 4 : ready_() ? 3 : 5, 0x23);
    r.text(0x44, "printer-state-reasons", busy() ? "none" : ready_() ? "none" : "offline");
    r.boolean("printer-is-accepting-jobs", !busy() && ready_());
    r.integer("queued-job-count", busy() ? 1 : 0);
    r.text(0x44, "ipp-versions-supported", "1.1"); r.text(0x44, "", "2.0");
    bool first = true;
    for (uint16_t op : {PrintJob, ValidateJob, CreateJob, SendDocument, CancelJob, GetJobAttributes, GetJobs, GetPrinterAttributes}) {
      r.integer(first ? "operations-supported" : "", op, 0x23); first = false;
    }
    r.text(0x47, "charset-configured", "utf-8"); r.text(0x47, "charset-supported", "utf-8");
    r.text(0x48, "natural-language-configured", "en"); r.text(0x48, "generated-natural-language-supported", "en");
    r.text(0x49, "document-format-default", "image/pwg-raster");
    r.text(0x49, "document-format-supported", "image/pwg-raster");
    r.text(0x44, "pwg-raster-document-type-supported", "black_1");
    r.text(0x44, "", "sgray_1"); r.text(0x44, "", "sgray_8");
    r.resolution("pwg-raster-document-resolution-supported", 600); r.resolution("", 300);
    r.text(0x44, "pwg-raster-document-sheet-back", "normal");
    r.resolution("printer-resolution-default", 600);
    r.resolution("printer-resolution-supported", 600); r.resolution("", 300);
    r.integer("copies-default", 1); r.range("copies-supported", 1, 1);
    r.text(0x44, "media-default", "iso_a4_210x297mm");
    r.text(0x44, "media-supported", "iso_a4_210x297mm");
    r.text(0x44, "media-ready", "iso_a4_210x297mm");
    mediaCollection(r, "media-col-default"); mediaCollection(r, "media-col-database"); mediaCollection(r, "media-col-ready");
    r.beginCollection("media-size-supported");
    r.member("x-dimension"); r.integer("", 21000); r.member("y-dimension"); r.integer("", 29700); r.endCollection();
    bool mf = true;
    for (const char *m : {"media-size", "media-bottom-margin", "media-left-margin", "media-right-margin", "media-top-margin"}) {
      r.text(0x44, mf ? "media-col-supported" : "", m); mf = false;
    }
    for (const char *m : {"media-bottom-margin-supported", "media-left-margin-supported", "media-right-margin-supported", "media-top-margin-supported"}) r.integer(m, 635);
    r.text(0x44, "sides-default", "one-sided"); r.text(0x44, "sides-supported", "one-sided");
    r.text(0x44, "", "two-sided-long-edge"); r.text(0x44, "", "two-sided-short-edge");
    r.text(0x44, "print-color-mode-default", "monochrome"); r.text(0x44, "print-color-mode-supported", "monochrome");
    r.boolean("color-supported", false); r.boolean("page-ranges-supported", false);
    r.integer("print-quality-default", 5, 0x23); r.integer("print-quality-supported", 5, 0x23); r.integer("", 4, 0x23);
    r.integer("orientation-requested-default", 3, 0x23); r.integer("orientation-requested-supported", 3, 0x23);
    r.integer("number-up-default", 1); r.integer("number-up-supported", 1);
    r.integer("finishings-default", 3, 0x23); r.integer("finishings-supported", 3, 0x23);
    r.text(0x44, "multiple-document-handling-default", "single-document");
    r.text(0x44, "multiple-document-handling-supported", "single-document");
    r.boolean("multiple-document-jobs-supported", false);
    r.integer("multiple-operation-time-out", 90);
    r.text(0x44, "multiple-operation-time-out-action", "abort-job");
    r.range("job-k-octets-supported", 0, 131072);
    bool jf = true;
    for (const char *a : {"copies", "media", "media-col", "sides", "printer-resolution", "print-color-mode", "job-name"}) {
      r.text(0x44, jf ? "job-creation-attributes-supported" : "", a); jf = false;
    }
  }
  void dispatch(Slot &s) {
    const Request &q = s.request;
    Serial.printf("[IPP] operation=0x%04x request=%u format=%s status=0x%04x\n",
                  unsigned(q.operation), unsigned(q.id), q.format.c_str(), unsigned(q.status));
    if (q.status) { status(s, q.status, "Unsupported request/settings; use A4, 300/600 dpi, monochrome, one/two-sided, one copy"); return; }
    switch (q.operation) {
      case GetPrinterAttributes: { Response r(q); printerAttributes(r); respond(s, r); break; }
      case ValidateJob: status(s, Ok); break;
      case GetJobs: {
        Response r(q);
        for (const auto &j : jobs_) {
          bool include = q.whichJobs == "all" || (q.whichJobs == "completed" ? j.state >= 7 : j.state < 7);
          if (j.id && include && (!q.myJobs || j.user == q.user)) appendJob(r, j);
        }
        respond(s, r); break;
      }
      case GetJobAttributes: {
        Job *j = job(q.jobId); if (!j) { status(s, NotFound); break; }
        Response r(q); appendJob(r, *j); respond(s, r); break;
      }
      case CancelJob: {
        Job *j = job(q.jobId); if (!j) { status(s, NotFound); break; }
        if (j->id != activeId_) { status(s, NotPossible); break; }
        raster_.abort(); flush_(); j->state = 7; j->reason = "job-canceled-by-user";
        releaseActive(); status(s, Ok); break;
      }
      case CreateJob:
      case PrintJob: {
        if (busy()) { status(s, PrinterBusy); break; }
        if (!ready_()) { status(s, ServiceUnavailable, "USB printer not ready"); break; }
        uint32_t id = nextId_++;
        if (!id) id = nextId_++;
        Job &j = jobs_[id % 4]; j = Job(); j.id = id; j.created = millis()/1000;
        j.name = q.name; j.user = q.user; activeId_ = id; activeSince_ = millis();
        activity_(true); raster_.reset(); raster_.setDuplex(q.sides.c_str());
        if (q.operation == CreateJob) {
          j.state = 4; j.reason = "job-data-insufficient";
          Response r(q); appendJob(r, j); respond(s, r);
        } else { j.state = 5; s.jobId = id; s.accepted = true; documentOpen_ = true; }
        break;
      }
      case SendDocument: {
        Job *j = job(q.jobId);
        if (!j) { status(s, NotFound); break; }
        if (!q.lastDocument) { status(s, AttributesUnsupported, "Only one document per job"); break; }
        if (documentOpen_ || activeId_ != q.jobId || j->state != 4) { status(s, NotPossible); break; }
        s.jobId = q.jobId; s.accepted = true; documentOpen_ = true; j->state = 5; j->reason = "job-incoming";
        break;
      }
      default: status(s, OperationUnsupported); break;
    }
  }
  bool parseHeaders(Slot &s) {
    int nl = s.headers.indexOf('\n');
    String first = s.headers.substring(0, nl); first.trim();
    if (!(first == "POST /ipp/print HTTP/1.1" || first == "POST /ipp/print HTTP/1.0" ||
          first == "POST /ipp/print/ HTTP/1.1")) { httpError(s, "404 Not Found"); return false; }
    bool hasLength = false, hasType = false, expect = false, transfer = false;
    for (int pos = nl + 1; pos < int(s.headers.length());) {
      int end = s.headers.indexOf('\n', pos); if (end < 0) break;
      String line = s.headers.substring(pos, end); line.trim(); pos = end + 1;
      if (!line.length()) break;
      int colon = line.indexOf(':'); if (colon < 1) { httpError(s, "400 Bad Request"); return false; }
      String name = line.substring(0, colon); name.toLowerCase();
      String value = line.substring(colon+1); value.trim();
      if (name == "content-length") {
        if (hasLength || !value.length()) { httpError(s, "400 Bad Request"); return false; }
        uint64_t length = 0;
        for (unsigned i = 0; i < value.length(); ++i) {
          if (value[i] < '0' || value[i] > '9') { httpError(s, "400 Bad Request"); return false; }
          length = length*10 + value[i]-'0';
          if (length > 134217728) { httpError(s, "413 Payload Too Large"); return false; }
        }
        s.remaining = uint32_t(length); hasLength = true;
      } else if (name == "transfer-encoding") {
        if (transfer) { httpError(s, "400 Bad Request"); return false; }
        value.toLowerCase(); if (value != "chunked") { httpError(s, "501 Not Implemented"); return false; }
        s.chunked = transfer = true;
      } else if (name == "content-type") { value.toLowerCase(); hasType = value.startsWith("application/ipp"); }
      else if (name == "expect") { value.toLowerCase(); if (value != "100-continue") { httpError(s, "417 Expectation Failed"); return false; } expect = true; }
      else if (name == "content-encoding" && value != "identity") { httpError(s, "415 Unsupported Media Type"); return false; }
    }
    if (!hasType) { httpError(s, "415 Unsupported Media Type"); return false; }
    if ((!hasLength && !s.chunked) || (hasLength && s.chunked) || (hasLength && s.remaining < 9)) {
      httpError(s, "400 Bad Request"); return false;
    }
    s.headersDone = true; s.headers = "";
    if (expect) s.client.print("HTTP/1.1 100 Continue\r\n\r\n");
    return true;
  }
  // Returns a body octet, -1 when waiting, -2 when framed body is complete,
  // -3 on malformed HTTP framing. Also bounds chunk lines and trailers.
  int bodyByte(Slot &s) {
    if (s.bodyDone) return -2;
    if (!s.chunked) {
      if (!s.remaining) { s.bodyDone = true; return -2; }
      if (!s.client.available()) return -1;
      --s.remaining; return s.client.read();
    }
    // Limit framing work per call even if a peer sends many empty trailers.
    for (unsigned budget = 0; budget < 128 && s.client.available(); ++budget) {
      if (s.chunkLeft) { --s.chunkLeft; int b = s.client.read(); if (!s.chunkLeft) s.chunkCR = true; return b; }
      if (s.chunkCR) { if (s.client.read() != '\r') return -3; s.chunkCR = false; s.chunkLF = true; continue; }
      if (s.chunkLF) { if (s.client.read() != '\n') return -3; s.chunkLF = false; continue; }
      int b = s.client.read(); s.line += char(b);
      if (s.line.length() > 256) return -3;
      if (b != '\n') continue;
      if (!s.line.endsWith("\r\n")) return -3;
      String line = s.line.substring(0, s.line.length()-2); s.line = "";
      if (s.trailers) {
        if (line.length()) { s.headers += line; if (s.headers.length() > 2048) return -3; continue; }
        s.bodyDone = true; return -2;
      }
      int semi = line.indexOf(';'); if (semi >= 0) line = line.substring(0, semi);
      if (!line.length()) return -3;
      uint64_t size = 0;
      for (unsigned i = 0; i < line.length(); ++i) {
        char c = line[i]; int d = c >= '0' && c <= '9' ? c-'0' : c >= 'a' && c <= 'f' ? c-'a'+10 : c >= 'A' && c <= 'F' ? c-'A'+10 : -1;
        if (d < 0) return -3;
        size = size*16 + d; if (size > 134217728) return -3;
      }
      if (!size) s.trailers = true; else s.chunkLeft = uint32_t(size);
    }
    return -1;
  }
  void complete(Slot &s) {
    if (!s.accepted) { httpError(s, "400 Bad Request"); return; }
    if (activeId_ != s.jobId) { status(s, NotPossible, "Job canceled or aborted"); return; }
    bool valid = raster_.finish();
    if (!valid) {
      Serial.printf("[IPP] Raster error: %s\n", raster_.error());
      failActive("document-format-error"); status(s, BadRequest, "Invalid or truncated PWG raster"); return;
    }
    if (!flush_()) { failActive("aborted-by-system"); status(s, InternalError, "USB transfer failed"); return; }
    Job *j = job(s.jobId); j->state = 9; j->reason = "none";
    Serial.printf("[IPP] Job %u: %u pages transferred to USB (physical completion not tracked)\n", unsigned(j->id), unsigned(raster_.pages()));
    Response r(s.request); appendJob(r, *j); releaseActive(); respond(s, r);
  }
  void service(Slot &s) {
    if (!s.used) return;
    if (s.jobId && s.jobId != activeId_) { status(s, NotPossible, "Job canceled or aborted"); return; }
    // Pending raster rows are output even when the sender is waiting for a response.
    if (s.accepted && raster_.pending()) {
      if (!raster_.drain()) { failActive("aborted-by-system"); status(s, InternalError, "USB transfer failed"); }
      else s.last = millis();
      return;
    }
    if (uint32_t(millis()-s.last) > 30000) {
      if (s.jobId == activeId_ && s.jobId) failActive("job-data-timeout");
      httpError(s, "408 Request Timeout"); return;
    }
    for (unsigned budget = 0; budget < 512 && s.used; ++budget) {
      if (!s.headersDone) {
        if (!s.client.available()) break;
        char c = char(s.client.read()); s.last = millis(); s.headers += c;
        if (s.headers.length() > 2048) { httpError(s, "431 Request Header Fields Too Large"); return; }
        if (s.headers.endsWith("\r\n\r\n")) { if (!parseHeaders(s)) return; }
        continue;
      }
      int b = bodyByte(s);
      if (b == -1) break;
      if (b == -3) { httpError(s, "400 Bad Request"); return; }
      if (b == -2) { complete(s); return; }
      s.last = millis();
      if (++s.bodyBytes > 134217728) { httpError(s, "413 Payload Too Large"); return; }
      if (!s.parser.done()) {
        int result = s.parser.feed(uint8_t(b));
        if (result < 0) { httpError(s, "400 Bad Request"); return; }
        if (result == 1) {
          s.request = s.parser.request(); dispatch(s);
          // Avoid consuming document bytes in the same pass as request dispatch.
          return;
        }
      } else {
        if (!raster_.feed(uint8_t(b))) {
          Serial.printf("[IPP] Raster error: %s\n", raster_.error());
          failActive("document-format-error"); status(s, BadRequest, "Unsupported or malformed PWG raster"); return;
        }
        if (raster_.pending()) return;
      }
    }
    if (s.used && !s.client.connected() && !s.client.available()) {
      // Content-Length may end on the last octet even after the peer sent FIN.
      if (s.headersDone && !s.chunked && !s.remaining) { complete(s); return; }
      if (s.jobId && s.jobId == activeId_) failActive("job-data-timeout");
      close(s);
    }
  }
};
} // namespace mobileprint
