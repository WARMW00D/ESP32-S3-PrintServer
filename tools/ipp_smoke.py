#!/usr/bin/env python3
"""Query IPP or print one A4 calibration page; Python standard library only.

Examples:
  python tools/ipp_smoke.py 192.168.4.1
  python tools/ipp_smoke.py 192.168.4.1 --print --chunked
"""
import argparse
import http.client
import struct

def attribute(tag, name, value):
    name = name.encode()
    if isinstance(value, str):
        value = value.encode()
    return bytes([tag]) + struct.pack(">H", len(name)) + name + struct.pack(">H", len(value)) + value

def request(host, operation):
    return (struct.pack(">BBHI", 2, 0, operation, 1) + b"\x01"
            + attribute(0x47, "attributes-charset", "utf-8")
            + attribute(0x48, "attributes-natural-language", "en")
            + attribute(0x45, "printer-uri", f"ipp://{host}:631/ipp/print"))

def calibration(dpi=600):
    width, height = (4960,7016) if dpi == 600 else (2480,3508)
    scale = dpi // 300
    header = bytearray(1796)
    for offset, value in {276: dpi, 280: dpi, 340: 1, 372: width, 376: height,
                          384: 8, 388: 8, 392: width, 400: 18, 420: 1,
                          456: 1, 460: 1}.items():
        struct.pack_into(">I", header, offset, value)
    yield b"RaS2" + header
    for y in range(height):
        row = bytearray([255] * width)
        for x in range(width):
            # 10 mm grid inside a 10 mm frame, plus a grayscale ramp.
            if 118*scale <= x < width - 118*scale and 118*scale <= y < height - 118*scale:
                if y in (118*scale,119*scale,height-119*scale,height-120*scale) or x in (118*scale,119*scale,width-119*scale,width-120*scale):
                    row[x] = 0
                elif (x-118*scale) % (118*scale) == 0 or (y-118*scale) % (118*scale) == 0:
                    row[x] = 100
                if 350*scale <= y < 700*scale and 250*scale <= x < width-250*scale:
                    row[x] = (x-250*scale) * 255 // (width-500*scale)
        encoded = bytearray([0])  # one copy of this scanline
        pos = 0
        while pos < width:
            repeat = 1
            while pos+repeat < width and repeat < 128 and row[pos+repeat] == row[pos]:
                repeat += 1
            if repeat >= 2:
                encoded.extend((repeat-1, row[pos])); pos += repeat
                continue
            end = min(width, pos+128)
            cursor = pos+1
            while cursor < end:
                if cursor+1 < width and row[cursor] == row[cursor+1]:
                    break
                cursor += 1
            count = cursor-pos
            if count == 1:
                encoded.extend((0, row[pos]))
            else:
                encoded.append(257-count); encoded.extend(row[pos:cursor])
            pos = cursor
        yield bytes(encoded)

def show(data):
    if len(data) < 8:
        raise SystemExit("Truncated IPP response")
    major, minor, status, rid = struct.unpack_from(">BBHI", data)
    print(f"IPP {major}.{minor}, status 0x{status:04x}, request {rid}")
    pos, previous = 8, ""
    while pos < len(data):
        tag = data[pos]; pos += 1
        if tag == 3:
            break
        if tag < 0x10:
            previous = ""; continue
        n = struct.unpack_from(">H", data, pos)[0]; pos += 2
        name = data[pos:pos+n].decode(errors="replace") if n else previous
        pos += n
        n = struct.unpack_from(">H", data, pos)[0]; pos += 2
        value = data[pos:pos+n]; pos += n; previous = name
        if tag in (0x21, 0x23) and len(value) == 4:
            text = str(struct.unpack(">I", value)[0])
        elif tag == 0x22:
            text = str(bool(value[0]))
        elif tag == 0x32 and len(value) == 9:
            x, y, unit = struct.unpack(">IIB", value); text = f"{x}x{y}, unit {unit}"
        elif tag in (0x34, 0x37):
            continue
        else:
            text = value.decode(errors="replace")
        print(f"  {name}: {text}")
    return status

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="ESP32 IP address, without a URL scheme")
    parser.add_argument("--print", action="store_true", help="send one A4 calibration page")
    parser.add_argument("--chunked", action="store_true", help="exercise HTTP chunked framing")
    parser.add_argument("--dpi",type=int,choices=[300,600],default=600)
    args = parser.parse_args()
    body = request(args.host, 2 if args.print else 11)
    if args.print:
        body += (attribute(0x49, "document-format", "image/pwg-raster")
                 + attribute(0x42, "job-name", "IPP calibration") + b"\x02"
                 + attribute(0x44, "media", "iso_a4_210x297mm")
                 + attribute(0x44, "sides", "one-sided")
                 + attribute(0x21, "copies", struct.pack(">I", 1)))
    body += b"\x03"
    connection = http.client.HTTPConnection(args.host, 631, timeout=180)
    headers = {"Content-Type": "application/ipp", "Connection": "close"}
    if args.chunked:
        def parts():
            yield body
            if args.print:
                yield from calibration(args.dpi)
        connection.request("POST", "/ipp/print", parts(), headers, encode_chunked=True)
    else:
        if args.print:
            body += b"".join(calibration(args.dpi))
        connection.request("POST", "/ipp/print", body, headers)
    response = connection.getresponse()
    data = response.read()
    print("HTTP", response.status, response.reason)
    if response.status != 200:
        raise SystemExit(1)
    code = show(data)
    connection.close()
    raise SystemExit(0 if code < 0x0400 else 1)

if __name__ == "__main__":
    main()
