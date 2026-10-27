"""Compile product PIC2..PIC10 SRT plus opening into one packed 1024x9 ROM.
Run generate_product_subtitles.py to import/check a new demonstration card.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RTL = ROOT / 'src/user_source/hdl_source/SD'
ASSETS = ROOT / 'assets/product_20261027'
MANIFEST = json.loads((ASSETS / 'manifest.json').read_text(encoding='utf-8'))
TOTALS = {n: MANIFEST['tracks'][str(n)]['samples'] for n in range(2,11)}


def load_cues(number):
    def samples(timestamp):
        h, m, s, ms = map(int, re.split('[:,]', timestamp))
        return ((h*3600+m*60+s)*1000+ms)*48
    cues = []
    for block in re.split(r'\n\s*\n', (ASSETS / f'AUDIO{number}.srt').read_text(encoding='utf-8-sig').strip()):
        lines = block.splitlines()
        start, end = map(samples, lines[1].split(' --> '))
        text = ''.join(lines[2:])
        if not 0 <= start < end <= TOTALS[number] or not 1 <= len(text) <= 40:
            raise ValueError(f'AUDIO{number}: invalid cue time/length')
        cues.append((start,end,text))
    paragraph = (ASSETS / f'AUDIO{number}.txt').read_text(encoding='utf-8-sig').strip()
    if not 1 <= len(cues) <= 8 or ''.join(c[2] for c in cues) != paragraph:
        raise ValueError(f'AUDIO{number}: 1..8 cues must match TXT exactly')
    if any(a[1] > b[0] for a,b in zip(cues,cues[1:])):
        raise ValueError(f'AUDIO{number}: overlapping cues')
    return cues


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--card', type=Path)
    args=parser.parse_args()
    if args.card:
        for n in range(1,11):
            for kind,extension in (('pcm','PCM'),('bmp','BMP')):
                name=f'AUDIO{n}' if kind=='pcm' else f'PIC{n}'
                data=(args.card / f'{name}.{extension}').read_bytes()
                if hashlib.sha256(data).hexdigest()!=MANIFEST['tracks'][str(n)][kind+'_sha256']:
                    parser.error(f'{name}: card changed; import the product again')
    font=(RTL/'subtitle_font.v').read_text(encoding='utf-8')
    ids={chr(i):i for i in range(32,127)}
    ids.update({chr(int(c,16)):int(i,16) for c,i in re.findall(r"16'h([0-9a-f]+): glyph=9'h([0-9a-f]+)",font)})
    words=[]; metadata=[]; bases=[]; counts=[]
    for track,n in enumerate(TOTALS):
        cues=load_cues(n); counts.append(len(cues))
        for cue,(start,end,text) in enumerate(cues):
            bases.append((track*8+cue,len(words)))
            metadata.append((track*8+cue,start,end,len(text)))
            words.extend(ids[c] for c in text)
        print(f'AUDIO{n}: {len(cues)} cues, {sum(len(c[2]) for c in cues)} characters, {TOTALS[n]/48000:.6f}s')
    opening_base=len(words)
    opening=(ROOT/'assets/opening.txt').read_text(encoding='utf-8-sig').splitlines()
    if len(opening)!=6 or any(len(s)>32 for s in opening):parser.error('Opening requires six lines, <=32 characters')
    words.extend(ids[opening[i//32][i%32]] if i%32<len(opening[i//32]) else 32 for i in range(192))
    if len(words)>1024:parser.error('Packed text exceeds 1024 glyphs')
    used=len(words); words.extend([32]*(1024-len(words)))
    out=['// Generated product PIC2..PIC10 SRT and opening; compact shared ROM.',
         'module audio4_subtitle_data(input wire clk, input wire [3:0] track, text_track,',
         '    input wire [2:0] cue, text_cue, input wire [5:0] position,',
         '    input wire opening, input wire [7:0] opening_address,',
         '    output reg [8:0] glyph, output reg [21:0] start_sample, end_sample,',
         '    output reg [2:0] last_cue,',
         '    output reg [5:0] length, output reg [21:0] total_samples);',
         '    reg [9:0] base;',
         '    reg [8:0] text_rom [0:1023]; /* fehdl force_ram=1, ram_style="bram" */',
         '    initial begin']
    out.extend(f"        text_rom[{i}]=9'h{v:03x};" for i,v in enumerate(words))
    out+=['    end',f"    wire [9:0] address=opening ? 10'd{opening_base}+opening_address : base+position;",
          '    always @(posedge clk) glyph<=text_rom[address];','    always @* begin','        case(track)']
    out.extend(f"            4'd{t}: begin total_samples=22'd{TOTALS[n]}; last_cue=3'd{counts[t]-1}; end" for t,n in enumerate(TOTALS))
    out+=["            default: begin total_samples=22'd1; last_cue=0; end",'        endcase','        case({track,cue})']
    out.extend(f"            7'd{k}: begin start_sample=22'd{s}; end_sample=22'd{e}; length=6'd{l}; end" for k,s,e,l in metadata)
    out+=['            default: begin start_sample=0; end_sample=0; length=0; end','        endcase','        case({text_track,text_cue})']
    out.extend(f"            7'd{k}: base=10'd{b};" for k,b in bases)
    out+=['            default: base=0;','        endcase','    end','endmodule','']
    (RTL/'audio4_subtitle_data.v').write_text('\n'.join(out),encoding='utf-8')
    print(f'Packed ROM: {used}/1024 words, opening base {opening_base}')

if __name__=='__main__':main()
