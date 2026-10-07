# Local IPP testing: HP P2015 + Poco X7 Pro

Experimental build, not yet tested on physical printer/phone hardware.
No changes have been published to GitHub.

1. Open `ESP32-S3-PrintServer.ino` with all three new headers beside it.
2. Use Arduino-ESP32 3.3.10, the hardware settings in README and partition
   **16M Flash (3MB APP/9.9MB FATFS)**. Verified library copies are in `dependencies/libraries`;
   copy them to your Arduino Sketchbook libraries folder if needed.
3. S3 binary: N16R8 / OPI PSRAM / OLED enabled. C6 binary: 4 MB / OLED off.
4. Connect phone and ESP32 to the same Wi-Fi network, or use the ESP32 AP.
5. Enable Android Default Print Service or try Mopria, select HP P2015 ESP32,
   or add the board IP manually: `ipp://BOARD_IP:631/ipp/print`.
6. Print `tests/test-page.pdf`: A4 portrait, mono, simplex, one copy, normal
   quality. Check text, grayscale, page order, 10 mm frame and 100 mm line.

Firmware files and checksums are in `firmware`. Full image flashing:

```text
python -m pip install esptool
python -m esptool --chip esp32s3 --port COM5 write-flash 0x0 firmware/full-flash.bin
```

Replace COM5 with the actual port. The full image overwrites settings/NVS,
so configure Wi-Fi again. Application-only `application.bin` at `0x10000`
requires an already-matching partition table. Arduino IDE upload is recommended
for the first test.

Diagnostics (Python standard library; each `--print` sends one physical page):

```text
python tools/ipp_smoke.py BOARD_IP
python tools/ipp_smoke.py BOARD_IP --print
python tools/ipp_smoke.py BOARD_IP --print --chunked
```

Save 115200-baud Serial logs including `[IPP]` and `[USB]` and screenshots if
Android cannot discover or print. USB transfer success does not confirm physical
paper output. Cancellation cannot retract printed pages. IPP is not protected
by the portal password. RAW/LPR remain available but their legacy handlers still
block the loop while printing.

Scope: full-page A4 portrait, 600 dpi by default (300 dpi also supported), mono, one copy, one active job;
PWG black_1/sgray_1/sgray_8. HP P2015d supports one-sided and automatic duplex
(two-sided-long-edge/two-sided-short-edge). No native PDF/JPEG, AirPrint, Bluetooth,
or Mopria/IPP Everywhere certification; Android rasterizes documents.

Validation: full firmware compilation, ASan/UBSan host tests and independent
3-page libcups-generated PWG input. To rerun on Linux: `bash tests/run.sh`,
then optionally `python tests/cups_fixture.py /tmp/cups.pwg` and
`bash tests/run.sh /tmp/cups.pwg`. Dependency versions/FQBN: `BUILD_INFO.json`.
