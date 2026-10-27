"""Import and compile the demonstration card without modifying any card file.
Usage: python scripts/generate_product_subtitles.py --card F:/
Raw PCM must already be signed little-endian 16-bit mono at 48000 Hz.
"""
import argparse
import hashlib
import json
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'assets/product_20261027'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--card',type=Path,required=True)
    args=parser.parse_args()
    manifest={'sample_rate':48000,'format':'s16le mono','tracks':{}}
    staged={}
    for n in range(1,11):
        bmp=(args.card/f'PIC{n}.BMP').read_bytes()
        if len(bmp)<54:parser.error(f'PIC{n}: truncated BMP')
        w,h,planes,bits,compression=struct.unpack_from('<iiHHI',bmp,18)
        offset=struct.unpack_from('<I',bmp,10)[0]
        if not (bmp[:2]==b'BM' and 0<w<=1920 and 0<abs(h)<=1080 and planes==1 and bits==24 and compression==0 and offset>=54 and len(bmp)>=offset+((w*3+3)//4*4)*abs(h)):
            parser.error(f'PIC{n}: unsupported or truncated RGB24 BMP')
        pcm=(args.card/f'AUDIO{n}.PCM').read_bytes()
        if not pcm or len(pcm)%2:parser.error(f'AUDIO{n}: empty/odd PCM length')
        samples=len(pcm)//2
        if n>1 and samples>=2**22:parser.error(f'AUDIO{n}: exceeds 22-bit sample counter')
        manifest['tracks'][str(n)]={'samples':samples,'pcm_sha256':hashlib.sha256(pcm).hexdigest(),'bmp_sha256':hashlib.sha256(bmp).hexdigest(),'width':w,'height':h}
        if n==1:continue
        for extension in ('srt','txt'):
            name=f'AUDIO{n}.{extension}'
            location=args.card/'字幕'/name
            if not location.is_file():location=args.card/name
            staged[name]=location.read_text(encoding='utf-8-sig').strip()
        cues=[]
        for block in re.split(r'\n\s*\n',staged[f'AUDIO{n}.srt']):
            lines=block.splitlines()
            def tick(t):
                hh,mm,ss,ms=map(int,re.split('[:,]',t))
                return ((hh*3600+mm*60+ss)*1000+ms)*48
            start,end=map(tick,lines[1].split(' --> '))
            text=''.join(lines[2:]);cues.append((start,end,text))
            if not (0<=start<end<=samples and 1<=len(text)<=40):parser.error(f'AUDIO{n}: invalid cue')
        if not 1<=len(cues)<=8 or any(a[1]>b[0] for a,b in zip(cues,cues[1:])) or ''.join(c[2] for c in cues)!=staged[f'AUDIO{n}.txt']:
            parser.error(f'AUDIO{n}: overlapping cues or TXT mismatch')
    # Validate font/text capacity before changing local imported assets.
    opening=(ROOT/'assets/opening.txt').read_text(encoding='utf-8-sig')
    chars={c for name,text in staged.items() if name.endswith('.txt') for c in text if ord(c)>=127}
    chars.update(c for c in opening if ord(c)>=127)
    if len(chars)>417 or any(ord(c)>65535 for c in chars):parser.error('Font exceeds 9-bit glyph capacity')
    if sum(len(t) for name,t in staged.items() if name.endswith('.txt'))+192>1024:parser.error('Packed text exceeds 1024 words')
    ASSETS.mkdir(parents=True,exist_ok=True)
    for name,text in staged.items():(ASSETS/name).write_text(text+'\n',encoding='utf-8')
    (ASSETS/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    command=[sys.executable,str(ROOT/'scripts/generate_subtitle_font.py'),'--text',str(ASSETS/'AUDIO2.txt'),'--opening',str(ROOT/'assets/opening.txt')]
    for n in range(3,11):command+=['--extra',str(ASSETS/f'AUDIO{n}.txt')]
    subprocess.run(command,check=True)
    subprocess.run([sys.executable,str(ROOT/'scripts/generate_file_subtitles.py'),'--card',str(args.card)],check=True)
    print('Imported/compiled PIC2..PIC10 subtitles. Card files were not modified. Rebuild TD before downloading.')

if __name__=='__main__':main()
