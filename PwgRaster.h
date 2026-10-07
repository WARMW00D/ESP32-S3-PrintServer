#pragma once
// Streaming PWG Raster -> PCL 5: A4, 300/600 dpi, simplex or duplex.
// Format reference: PWG 5102.4-2012, sections 4.2-4.4.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

namespace mobileprint {
inline uint32_t be32(const uint8_t *p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
         (uint32_t(p[2]) << 8) | p[3];
}

class PwgRaster {
 public:
  typedef bool (*Writer)(const uint8_t *, size_t);
  explicit PwgRaster(Writer writer) : writer_(writer) { reset(); }
  void reset() {
    state_ = Sync; pos_ = rowPos_ = row_ = pages_ = repeat_ = run_ = 0;
    started_ = pageOpen_ = false; error_ = nullptr;
  }
  const char *error() const { return error_; }
  bool pending() const { return state_ == Emit; }
  uint32_t pages() const { return pages_; }
  bool started() const { return started_; }
  void setDuplex(const char *sides) {
    duplex_ = sides && strcmp(sides, "two-sided-long-edge") == 0 ? 1 :
              sides && strcmp(sides, "two-sided-short-edge") == 0 ? 2 : 0;
  }

  // Caller must drain pending rows before feeding more input. Never retain a page.
  bool feed(uint8_t byte) {
    if (error_) return false;
    switch (state_) {
      case Sync:
        if (byte != uint8_t("RaS2"[pos_++])) return fail("Not PWG Raster (RaS2)");
        if (pos_ == 4) { pos_ = 0; state_ = Header; }
        break;
      case Header:
        header_[pos_++] = byte;
        if (pos_ == sizeof(header_)) {
          pos_ = 0;
          if (!openPage()) return false;
          row_ = 0; state_ = Repeat;
        }
        break;
      case Repeat:
        repeat_ = uint16_t(byte) + 1;
        if (repeat_ > height_ - row_) return fail("Row repeat exceeds page");
        rowPos_ = 0; state_ = Control;
        break;
      case Control:
        // PWG: 0..127 repeat count+1, 129..255 literal count 257-control.
        // 128 is not a valid PWG run (unlike extensions in some CUPS readers).
        if (byte == 128) return fail("Invalid PWG run");
        literal_ = byte > 128;
        run_ = literal_ ? 257 - byte : byte + 1;
        if (run_ > bytesPerLine_ - rowPos_) return fail("Run exceeds row");
        state_ = Value;
        break;
      case Value:
        if (literal_) { rowData_[rowPos_++] = byte; --run_; }
        else { memset(rowData_ + rowPos_, byte, run_); rowPos_ += run_; run_ = 0; }
        if (!run_) state_ = rowPos_ == bytesPerLine_ ? Emit : Control;
        break;
      case Emit: return fail("Decoder must drain before input");
    }
    return true;
  }

  // One physical row per call, so HTTP queries can run between USB writes.
  bool drain() {
    if (error_) return false;
    if (state_ != Emit) return true;
    if (row_ >= margin_ && row_ < height_ - margin_) {
      const uint32_t outWidth = width_ - 2 * margin_;
      const size_t count = (outWidth + 7) / 8;
      memset(bits_, 0, count);
      static const uint8_t bayer[4][4] = {
        {0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}
      };
      for (uint32_t x = 0; x < outWidth; ++x) {
        const uint32_t src = x + margin_;
        bool black;
        if (depth_ == 1) {
          bool bit = (rowData_[src / 8] & (0x80 >> (src & 7))) != 0;
          black = colorSpace_ == 3 ? bit : !bit; // DeviceK vs sGray
        } else {
          uint8_t gray = rowData_[src];
          black = gray < uint8_t(bayer[row_ & 3][src & 3] * 16 + 8);
        }
        if (black) bits_[x / 8] |= 0x80 >> (x & 7);
      }
      char command[32];
      int n = snprintf(command, sizeof(command), "\033*b%uW", unsigned(count));
      if (!write(reinterpret_cast<const uint8_t *>(command), size_t(n)) ||
          !write(bits_, count)) return false;
    }
    ++row_;
    if (--repeat_) return true;
    if (row_ == height_) {
      if (!text("\033*rB\f")) return false;
      pageOpen_ = false; ++pages_; state_ = Header;
    } else state_ = Repeat;
    return true;
  }

  bool finish() {
    if (error_) return false;
    if (!pages_ || state_ != Header || pos_) return fail("Truncated PWG document");
    // Close the PCL language and the PJL job explicitly.  Some LaserJet
    // firmware keeps the USB status LED blinking until it sees EOJ/UEL.
    return text("\033E\033%-12345X@PJL EOJ NAME=\"IPP\"\r\n\033%-12345X");
  }
  void abort() {
    // A partial page may already have printed. Never resend failed USB bytes.
    const char *end = "\033*rB\033E\033%-12345X@PJL EOJ NAME=\"IPP\"\r\n\033%-12345X";
    if (started_) writer_(reinterpret_cast<const uint8_t *>(end), strlen(end));
    reset();
  }

 private:
  enum State { Sync, Header, Repeat, Control, Value, Emit } state_;
  Writer writer_;
  uint8_t header_[1796], rowData_[4961], bits_[621];
  uint32_t dpi_ = 300, margin_ = 75; // 6.35 mm: dpi / 4
  uint32_t width_ = 0, height_ = 0, depth_ = 0, colorSpace_ = 0;
  uint32_t bytesPerLine_ = 0, row_, pages_;
  size_t pos_, rowPos_;
  uint16_t repeat_, run_;
  bool literal_ = false, started_, pageOpen_;
  uint8_t duplex_ = 0;
  const char *error_;
  bool fail(const char *s) { error_ = s; return false; }
  bool write(const uint8_t *p, size_t n) { return writer_(p, n) || fail("USB write failed"); }
  bool text(const char *s) { return write(reinterpret_cast<const uint8_t *>(s), strlen(s)); }
  bool openPage() {
    width_ = be32(header_ + 372); height_ = be32(header_ + 376);
    depth_ = be32(header_ + 384); colorSpace_ = be32(header_ + 400);
    bytesPerLine_ = be32(header_ + 392);
    dpi_ = be32(header_ + 276); margin_ = dpi_ / 4;
    const bool geometry = dpi_ == 300
      ? width_ >= 2480 && width_ <= 2481 && height_ >= 3507 && height_ <= 3508
      : dpi_ == 600 && width_ >= 4960 && width_ <= 4961 && height_ >= 7015 && height_ <= 7016;
    if (!geometry || be32(header_ + 280) != dpi_)
      return fail("Only full-page A4 at 300 or 600 dpi is supported");
    // CUPS/PWG sets Duplex at +272 and Tumble at +368 for two-sided jobs.
    // They are boolean fields; accept either value and let the IPP sides
    // attribute select the PCL binding direction.
    if (be32(header_ + 272) > 1 || be32(header_ + 368) > 1 || be32(header_ + 340) > 1 ||
        be32(header_ + 344) || be32(header_ + 456) > 1 || be32(header_ + 460) > 1 ||
        be32(header_ + 396) || be32(header_ + 420) != 1 ||
        be32(header_ + 388) != depth_ ||
        !((depth_ == 1 && (colorSpace_ == 3 || colorSpace_ == 18)) ||
          (depth_ == 8 && colorSpace_ == 18)) ||
        bytesPerLine_ != (width_ * depth_ + 7) / 8)
      return fail("Unsupported PWG page settings");
    if (pages_ >= 1000) return fail("Too many pages");
    if (!started_) {
      // HP P2015 supports PCL 5. No PDF/PostScript interpreter on the ESP32.
      const char *pjlDuplex = duplex_ == 1
        ? "@PJL SET DUPLEX=ON\n@PJL SET BINDING=LONGEDGE\n"
        : duplex_ == 2
          ? "@PJL SET DUPLEX=ON\n@PJL SET BINDING=SHORTEDGE\n"
          : "@PJL SET DUPLEX=OFF\n";
      char start[220];
      int startLen = snprintf(start, sizeof(start),
        "\033%%-12345X@PJL JOB NAME=\"IPP\"\n%s@PJL ENTER LANGUAGE=PCL\n\033E", pjlDuplex);
      if (startLen <= 0 || !write(reinterpret_cast<const uint8_t *>(start), size_t(startLen))) return false;
      started_ = true;
    }
    char command[180];
    // Duplex is a job-level setting. Repeating it before page 2 can make
    // older LaserJet firmware start a new physical sheet instead of using
    // the back side of the current one.
    const char *duplexCommand = pages_ ? "" :
      (duplex_ == 1 ? "\033&l1S" : duplex_ == 2 ? "\033&l2S" : "\033&l0S");
    const char *jobPageSetup = pages_ ? "" : "\033&l26A\033&l0O\033&l0E\033&l0L\033&l1X";
    // PCL logical left edge is 1/4 inch from physical paper; crop that margin.
    // Top margin zero, then move down 1/4 inch in the selected device units.
    int n = snprintf(command, sizeof(command),
      "%s%s"
      "\033&a0L\033&u%uD\033*p0X\033*p%uY\033*t%uR\033*r%uS\033*r%uT"
      "\033*r1A\033*b0M", jobPageSetup, duplexCommand,
      unsigned(dpi_), unsigned(margin_), unsigned(dpi_),
      unsigned(width_ - 2 * margin_), unsigned(height_ - 2 * margin_));
    pageOpen_ = true;
    return write(reinterpret_cast<const uint8_t *>(command), size_t(n));
  }
};
} // namespace mobileprint
