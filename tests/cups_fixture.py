#!/usr/bin/env python3
"""Independent PWG encoder: generate a fixture with the system's libcups."""
import ctypes as c
import ctypes.util
import struct
import sys

lib = c.CDLL(ctypes.util.find_library("cups"))
callback_type = c.CFUNCTYPE(c.c_ssize_t, c.c_void_p, c.c_void_p, c.c_size_t)
output = bytearray()

@callback_type
def write(_context, buffer, count):
    output.extend(c.string_at(buffer, count))
    return count

lib.cupsRasterOpenIO.argtypes = [callback_type, c.c_void_p, c.c_int]
lib.cupsRasterOpenIO.restype = c.c_void_p
lib.cupsRasterWriteHeader2.argtypes = [c.c_void_p, c.c_void_p]
lib.cupsRasterWritePixels.argtypes = [c.c_void_p, c.c_void_p, c.c_uint]
lib.cupsRasterClose.argtypes = [c.c_void_p]
raster = lib.cupsRasterOpenIO(write, None, 3)  # CUPS_RASTER_WRITE_PWG
assert raster
for depth, color, dpi in [(1,3,300),(1,18,300),(8,18,300),(1,3,600),(1,18,600),(8,18,600)]:
    width, height = (4960,7016) if dpi == 600 else (2480,3508)
    header = bytearray(1796)
    count = (width * depth + 7) // 8
    for offset, value in {
        276: dpi, 280: dpi, 340: 1, 352: 595, 356: 842,
        372: width, 376: height, 384: depth, 388: depth, 392: count,
        400: color, 420: 1, 452: 3, 456: 1, 460: 1,
    }.items():
        struct.pack_into("=I", header, offset, value)
    header[1732:1732+len(b"iso_a4_210x297mm")] = b"iso_a4_210x297mm"
    raw_header = c.create_string_buffer(bytes(header))
    assert lib.cupsRasterWriteHeader2(raster, raw_header)
    for y in range(height):
        # Alternate literal runs, long repeats and repeated scanlines.
        if y % 256 < 128:
            row = bytes([255 if color != 3 else 0]) * count
        else:
            row = bytes((x * 31 + (y // 8) * 7) % 256 for x in range(count))
        data = c.create_string_buffer(row)
        assert lib.cupsRasterWritePixels(raster, data, len(row)) == len(row)
lib.cupsRasterClose(raster)
with open(sys.argv[1], "wb") as f:
    f.write(output)
print(f"CUPS produced {len(output)} bytes, 6 PWG pages")
