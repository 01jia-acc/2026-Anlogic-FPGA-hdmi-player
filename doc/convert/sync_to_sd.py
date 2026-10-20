"""将合规 BMP 同步为 FAT32 根目录的 PIC1.BMP～PIC16.BMP。"""
import argparse
from pathlib import Path
import re
import shutil
import struct
import tempfile

MAX_IMAGES = 16
MANAGED_NAME = re.compile(r"PIC(?:[1-9]|1[0-6])\.BMP", re.IGNORECASE)


def validate_bmp(path):
    """校验 FPGA 支持的 640×480、bottom-up、24-bit BI_RGB 文件。"""
    with path.open("rb") as stream:
        header = stream.read(54)
    if len(header) < 54 or header[:2] != b"BM":
        raise ValueError(f"{path.name}: 不是完整的 BMP 文件")
    offset, dib_size = struct.unpack_from("<II", header, 10)
    width, height, planes, depth, compression = struct.unpack_from("<iiHHI", header, 18)
    if (dib_size < 40 or offset < 14 + dib_size or
            (width, height, planes, depth, compression) != (640, 480, 1, 24, 0)):
        raise ValueError(f"{path.name}: 需要 640×480、24 位、无压缩、正高度 BMP")
    actual_size = path.stat().st_size
    pixel_end = offset + 640 * 480 * 3
    declared_size = struct.unpack_from("<I", header, 2)[0]
    if actual_size < pixel_end:
        raise ValueError(f"{path.name}: 像素数据不完整")
    if declared_size != 0 and not pixel_end <= declared_size <= actual_size:
        raise ValueError(f"{path.name}: BMP 头中的文件大小不合法")


def sync_new_images(source_dir, target_drive, max_count=MAX_IMAGES):
    """只替换播放器的 PIC 文件；保留所有 PCM 和其他文件/目录。"""
    if not 1 <= max_count <= MAX_IMAGES:
        raise ValueError("图片数量必须在 1～16 之间")
    source = Path(source_dir).resolve()
    target = Path(target_drive).resolve()
    if not source.is_dir():
        raise ValueError(f"源目录不存在: {source}")
    if not target.is_dir():
        raise ValueError(f"目标目录不存在: {target}")
    if source == target or source in target.parents or target in source.parents:
        raise ValueError("源目录与目标目录不能相同或互相包含")
    files = sorted((p for p in source.iterdir() if p.is_file() and p.suffix.lower() == ".bmp"),
                   key=lambda p: p.name.casefold())[:max_count]
    if not files:
        raise ValueError(f"源目录中没有 BMP 图片: {source}")
    # 校验全部待复制文件后才修改目标；不破坏旧 BMP 头，也不清空 SD 卡。
    for image in files:
        validate_bmp(image)
    old_files = [p for p in target.iterdir() if MANAGED_NAME.fullmatch(p.name)]
    desired_names = {f"PIC{i + 1}.BMP" for i in range(len(files))}
    for p in old_files:
        if p.name.upper() in desired_names and not p.is_file():
            raise ValueError(f"目标名称被目录占用: {p.name}")
    # 先完成临时副本；复制失败时原 PIC 文件不变。之后逐文件替换。
    with tempfile.TemporaryDirectory(prefix=".bmp_sync_", dir=target) as staging:
        staging = Path(staging)
        for i, image in enumerate(files, 1):
            shutil.copy2(image, staging / f"PIC{i}.BMP")
        for i, image in enumerate(files, 1):
            destination = target / f"PIC{i}.BMP"
            (staging / destination.name).replace(destination)
            print(f"[同步成功] {image.name} -> {destination.name}")
        for old in old_files:
            # Windows 文件名不区分大小写；其他平台清理旧的小写副本。
            if old.is_file() and (old.name.upper() not in desired_names or
                                  (old.name not in desired_names and
                                   not any(old.samefile(target / name) for name in desired_names))):
                old.unlink()
    print(f"已同步 {len(files)} 张图片，保留所有 PCM 和其他文件。请按 PIC 编号自行匹配音频，并安全弹出 SD 卡。")
    return len(files)


def main():
    parser = argparse.ArgumentParser(description="FAT32 多图片同步工具（最多16张，保留音频）")
    parser.add_argument("drive", help="SD 卡盘符，例如 E:；也可指定已有目标目录")
    parser.add_argument("-s", "--source", default=str(Path(__file__).resolve().parent / "output_bmp"),
                        help="BMP 源目录，默认脚本旁的 output_bmp")
    parser.add_argument("-n", "--num", type=int, default=MAX_IMAGES,
                        help="最多同步的图片数，1～16，默认16")
    args = parser.parse_args()
    destination = args.drive + "\\" if re.fullmatch(r"[A-Za-z]:", args.drive) else args.drive
    try:
        sync_new_images(args.source, destination, args.num)
    except (ValueError, OSError) as exc:
        parser.exit(1, f"[错误] {exc}\n")


if __name__ == "__main__":
    main()
