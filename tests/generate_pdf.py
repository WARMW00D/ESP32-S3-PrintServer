#!/usr/bin/env python3
"""Rebuild the included Android test document (requires reportlab)."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm

path = Path(__file__).with_name("test-page.pdf")
c = canvas.Canvas(str(path), pagesize=A4)
c.setTitle("HP P2015 ESP32 IPP test / A4")
w, h = A4
for page in (1, 2):
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(0.3 * mm)
    c.rect(10 * mm, 10 * mm, w - 20 * mm, h - 20 * mm)
    c.setFont("Helvetica-Bold", 18)
    c.drawString(18 * mm, h - 26 * mm, "HP P2015 / ESP32 / IPP")
    c.setFont("Helvetica", 11)
    c.drawString(18 * mm, h - 35 * mm, f"A4 portrait | 300 dpi | monochrome | page {page} of 2")
    c.drawString(18 * mm, h - 43 * mm, "Print at actual size. Frame is 10 mm from paper edges.")
    c.drawString(18 * mm, h - 56 * mm, "Text: ABCDEFG abcdefg 0123456789")
    for size, y in [(8, 67), (10, 76), (14, 87), (20, 101)]:
        c.setFont("Helvetica", size)
        c.drawString(18 * mm, h - y * mm, f"{size} pt: The quick brown fox 0123456789")
    c.setFont("Helvetica", 11)
    c.drawString(18 * mm, h - 116 * mm, "Grayscale steps (white to black)")
    for x in range(16):
        gray = 1 - x / 15
        c.setFillColorRGB(gray, gray, gray)
        c.rect((18 + 10 * x) * mm, h - 137 * mm, 10 * mm, 15 * mm, fill=1, stroke=0)
    c.setFillColorRGB(0, 0, 0)
    c.drawString(18 * mm, h - 150 * mm, "Measure this line: 100 mm")
    c.line(18 * mm, h - 158 * mm, 118 * mm, h - 158 * mm)
    for x in range(0, 101, 10):
        c.line((18 + x) * mm, h - 156 * mm, (18 + x) * mm, h - 160 * mm)
    for i in range(11):
        c.line((18 + i * 10) * mm, 30 * mm, (18 + i * 10) * mm, 100 * mm)
    for i in range(8):
        c.line(18 * mm, (30 + i * 10) * mm, 118 * mm, (30 + i * 10) * mm)
    c.drawString(130 * mm, 85 * mm, "10 mm grid")
    c.drawString(130 * mm, 75 * mm, "No clipping?")
    c.drawString(130 * mm, 65 * mm, "No extra blank pages?")
    c.drawString(130 * mm, 55 * mm, "Correct page order?")
    c.showPage()
c.save()
print(path)
