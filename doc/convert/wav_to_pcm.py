"""将符合讲义格式的 WAV 去头，输出 FPGA 使用的原始 PCM；不隐式重采样。"""
import argparse
from pathlib import Path
import wave


def convert(source: Path, target: Path) -> int:
    if source.resolve() == target.resolve():
        raise ValueError("输入和输出不能是同一个文件")
    if target.exists():
        raise ValueError("输出已存在，请换一个输出路径；不会覆盖已有文件")
    # Validate before creating any output. A complete frame is exactly 2 bytes.
    with wave.open(str(source), "rb") as wav:
        if (wav.getframerate(), wav.getnchannels(), wav.getsampwidth(), wav.getcomptype()) != (48000, 1, 2, "NONE"):
            raise ValueError("必须是 48000 Hz、单声道、16 位整数 PCM WAV；其他格式请先用 FFmpeg 转换")
        frames = wav.getnframes()
        if frames == 0:
            raise ValueError("音频为空")
        created = False
        try:
            with target.open("xb") as output:
                created = True
                remaining = frames * 2
                while remaining:
                    data = wav.readframes(min(32768, remaining // 2))
                    if not data or len(data) % 2:
                        raise ValueError("WAV 数据不完整")
                    output.write(data)
                    remaining -= len(data)
        except Exception:
            if created:
                target.unlink(missing_ok=True)
            raise
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="48 kHz / mono / s16 WAV")
    parser.add_argument("target", type=Path, nargs="?", default=Path("AUDIO.PCM"))
    args = parser.parse_args()
    try:
        frames = convert(args.source, args.target)
    except (ValueError, OSError, EOFError, wave.Error) as exc:
        parser.exit(1, f"转换失败：{exc}\n")
    print(f"已生成 {args.target}：{frames} 个样本，{frames / 48000:.3f} 秒，无 WAV 头")


if __name__ == "__main__":
    main()
