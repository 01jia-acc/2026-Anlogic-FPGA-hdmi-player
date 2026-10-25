"""Prepare a center-cropped RGB24 PIC1 and preview using the real FPGA glyph ROM."""
import argparse
import datetime
import re
import shutil
import struct
from pathlib import Path
from PIL import Image, ImageOps, ImageDraw

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--card', type=Path, help='Replace only PIC1.BMP after backing it up')
    args = parser.parse_args()
    output = ROOT / 'assets/opening'
    output.mkdir(parents=True, exist_ok=True)
    with Image.open(args.image) as source:
        background = ImageOps.fit(source.convert('RGB'), (640, 480), Image.Resampling.LANCZOS)
    bmp = output / 'PIC1.BMP'
    background.save(bmp, format='BMP')
    data = bmp.read_bytes()
    assert data[:2] == b'BM' and len(data) == 921654
    assert struct.unpack_from('<iiHHI', data, 18) == (640, 480, 1, 24, 0)
    with Image.open(bmp) as check:
        check.load()
        assert check.tobytes() == background.tobytes()
    font = (ROOT / 'src/user_source/hdl_source/SD/subtitle_font.v').read_text(encoding='utf-8')
    ids = {chr(i): i for i in range(32, 127)}
    ids.update({chr(int(c,16)): int(i,16) for c,i in re.findall(r"16'h([0-9a-f]+): glyph=8'h([0-9a-f]+)", font)})
    pixels = {int(a): int(v,16) for a,v in re.findall(r"pixels\[(\d+)\]=16'h([0-9a-f]+)", font)}
    texts = (ROOT / 'assets/opening.txt').read_text(encoding='utf-8').splitlines()
    sheet = Image.new('RGB', (1280, 960))
    for panel, tick in enumerate((90, 210, 330, 450)):
        frame = background.copy()
        draw = ImageDraw.Draw(frame)
        draw.rectangle((40,190,279,191), fill='#b18539')
        draw.rectangle((40,452,39+tick,454), fill='#b18539')
        for i,(top,start) in enumerate(zip((96,152,304,336,368,416),(60,120,180,240,300,390))):
            if tick < start:
                continue
            scale = 2 if i == 0 else 1
            left = 8+((tick-60)>>1) if i == 0 and tick<124 else 40
            for col,char in enumerate(texts[i]):
                for y in range(16):
                    bits = pixels[ids[char]*16+y]
                    for x in range(16):
                        if bits & (0x8000>>x):
                            px,py = left+(col*16+x)*scale,top+y*scale
                            draw.rectangle((px,py,px+scale-1,py+scale-1),fill='#39281c')
        sheet.paste(frame, ((panel%2)*640,(panel//2)*480))
    sheet.save(output / 'preview.png')
    if args.card:
        target = args.card.resolve() / 'PIC1.BMP'
        if not target.is_file():
            raise FileNotFoundError(target)
        backups = ROOT.parent / 'opening_backups'
        backups.mkdir(exist_ok=True)
        backup = backups / ('PIC1_before_opening_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.BMP')
        shutil.copy2(target, backup)
        shutil.copyfile(bmp, target)
        assert target.read_bytes() == data
        print('Previous PIC1 backed up:', backup)
        print('Replaced:', target)
    print('Verified RGB24 BMP:', bmp)
    print('Preview:', output / 'preview.png')


if __name__ == '__main__':
    main()
