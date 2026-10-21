"""Validate generated glyph orientation, Unicode IDs and boot paragraph contents."""
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = ROOT / 'src/user_source/hdl_source/SD/subtitle_font.v'


class SubtitleFontTests(unittest.TestCase):
    def test_checked_in_font_and_default_paragraph(self):
        source = FONT.read_text(encoding='utf-8')
        pixels = {int(a): int(v, 16) for a, v in re.findall(r"pixels\[(\d+)\]=16'h([0-9a-f]+)", source)}
        self.assertEqual(set(pixels), set(range(4096)))
        code_to_id = {i: i for i in range(32, 127)}
        code_to_id.update({int(c, 16): int(i, 16) for c, i in re.findall(r"16'h([0-9a-f]+): glyph=8'h([0-9a-f]+)", source)})
        text = (ROOT / 'assets/subtitle.txt').read_text(encoding='utf-8-sig').strip()
        length = int(re.search(r"assign length=9'd(\d+)", source).group(1))
        ids = [int(i, 16) for i in re.findall(r"8'd\d+: glyph<=8'h([0-9a-f]+)", source)]
        self.assertEqual(length, len(text))
        self.assertEqual(ids, [code_to_id[ord(c)] for c in text])
        font = ImageFont.truetype('C:/Windows/Fonts/simsun.ttc', 16)
        # Independently render every included glyph, verify row order/MSB-left.
        for codepoint, glyph in code_to_id.items():
            bitmap = Image.new('L', (16, 16))
            draw = ImageDraw.Draw(bitmap)
            left, top, right, bottom = draw.textbbox((0, 0), chr(codepoint), font=font)
            draw.text(((16 - right + left) // 2 - left, (16 - bottom + top) // 2 - top),
                      chr(codepoint), font=font, fill=255)
            for y in range(16):
                row = pixels[glyph * 16 + y]
                for x in range(16):
                    self.assertEqual(bool(row & (0x8000 >> x)), bitmap.getpixel((x, y)) >= 96)
            if codepoint != 32:
                self.assertTrue(any(pixels[glyph * 16 + y] for y in range(16)))

    def test_generator_and_additional_characters(self):
        with tempfile.TemporaryDirectory(prefix='subtitle_font_') as tmp:
            temp = Path(tmp)
            extra = temp / 'extra.txt'
            extra.write_text('中文测试', encoding='utf-8')
            output = temp / 'generated.v'
            subprocess.run([sys.executable, str(ROOT / 'scripts/generate_subtitle_font.py'),
                            '--extra', str(extra), '--output', str(output)], check=True, capture_output=True)
            data = output.read_text(encoding='utf-8')
            for c in '中文测试':
                self.assertIn(f"16'h{ord(c):04x}: glyph=", data)
            extra.write_text('😀', encoding='utf-8')
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/generate_subtitle_font.py'),
                                     '--extra', str(extra), '--output', str(output)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    unittest.main()
