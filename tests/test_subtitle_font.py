"""Validate generated glyph orientation, Unicode IDs and boot paragraph contents."""
import re
import datetime
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = ROOT / 'src/user_source/hdl_source/SD/subtitle_font.v'


class SubtitleFontTests(unittest.TestCase):
    def test_serial_lookup_table_matches_original_mapping(self):
        source = FONT.read_text(encoding='utf-8')
        mapping = {int(c,16):int(i,16) for c,i in
                   re.findall(r"16'h([0-9a-f]+): glyph=9'h([0-9a-f]+)",source)}
        rows = [(int(n),int(c,16),int(i,16)) for n,c,i in
                re.findall(r"9'd(\d+): entry=\{16'h([0-9a-f]+),9'h([0-9a-f]+)\};",source)]
        self.assertEqual([n for n,c,i in rows],list(range(len(mapping))))
        self.assertEqual([c for n,c,i in rows],sorted(mapping))
        self.assertEqual({c:i for n,c,i in rows},mapping)

    def test_compiled_srt_uses_current_font_and_timestamps(self):
        font = FONT.read_text(encoding='utf-8')
        code_to_id = {i: i for i in range(32, 127)}
        code_to_id.update({int(c, 16): int(i, 16) for c, i in
                          re.findall(r"16'h([0-9a-f]+): glyph=9'h([0-9a-f]+)", font)})
        rtl = (FONT.parent / 'audio4_subtitle_data.v').read_text(encoding='utf-8')
        rom = {int(a): int(v, 16) for a, v in re.findall(r"text_rom\[(\d+)\]=9'h([0-9a-f]+)", rtl)}
        self.assertEqual(set(rom), set(range(1024)))
        base = 0
        for track, number in enumerate(range(2,11)):
            cues = re.split(r'\n\s*\n', (ROOT / f'assets/product_20261027/AUDIO{number}.srt').read_text(encoding='utf-8-sig').strip())
            self.assertEqual(len(cues), 3)
            offset = 0
            for index, block in enumerate(cues):
                lines = block.splitlines()
                text = ''.join(lines[2:])
                self.assertEqual([rom[base+offset+i] for i in range(len(text))], [code_to_id[ord(c)] for c in text])
                self.assertIn(f"7'd{track*8+index}: base=10'd{base+offset};", rtl)
                ticks = []
                for timestamp in lines[1].split(' --> '):
                    t = datetime.datetime.strptime(timestamp, '%H:%M:%S,%f')
                    ticks.append(((t.hour*3600+t.minute*60+t.second)*1000000+t.microsecond)*48000//1000000)
                self.assertIn(f"7'd{track*8+index}: begin start_sample=22'd{ticks[0]}; end_sample=22'd{ticks[1]}; length=6'd{len(text)};", rtl)
                offset += len(text)
            path = f'assets/product_20261027/AUDIO{number}.txt'
            self.assertEqual(offset, len((ROOT / path).read_text(encoding='utf-8-sig').strip()))
            base += offset
        opening = (ROOT / 'assets/opening.txt').read_text(encoding='utf-8-sig').splitlines()
        for i in range(192):
            char = opening[i//32][i%32] if i//32<6 and i%32<len(opening[i//32]) else ' '
            self.assertEqual(rom[base+i], code_to_id[ord(char)])

    def test_checked_in_font_and_default_paragraph(self):
        source = FONT.read_text(encoding='utf-8')
        bases = [int(x) for x in re.findall(r'// Font row base (\d+),', source)]
        depths = [int(x)+1 for x in re.findall(r'pixels\d+ \[0:(\d+)\]', source)]
        self.assertEqual(depths, [4096, 2048, 512])
        self.assertTrue(all(d & (d-1) == 0 for d in depths))
        pixels = {bases[int(b)]+int(a): int(v,16) for b,a,v in
                  re.findall(r"pixels(\d+)\[(\d+)\]=16'h([0-9a-f]+)", source)}
        depth = sum(depths)
        self.assertEqual(set(pixels), set(range(depth)))
        code_to_id = {i: i for i in range(32, 127)}
        code_to_id.update({int(c, 16): int(i, 16) for c, i in re.findall(r"16'h([0-9a-f]+): glyph=9'h([0-9a-f]+)", source)})
        self.assertEqual(len(set(code_to_id.values())), len(code_to_id))
        self.assertGreater(max(code_to_id.values()), 255)
        text = (ROOT / 'assets/product_20261027/AUDIO2.txt').read_text(encoding='utf-8-sig').strip()
        length = int(re.search(r"assign length=9'd(\d+)", source).group(1))
        ids = [int(i, 16) for i in re.findall(r"8'd\d+: glyph<=9'h([0-9a-f]+)", source)]
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
