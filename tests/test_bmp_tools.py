"""只用临时目录验证图片转换和同步，不访问实际 SD 卡。"""
import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

CONVERT_DIR = Path(__file__).resolve().parents[1] / "doc/convert"


def load(name):
    spec = importlib.util.spec_from_file_location(name, CONVERT_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sync = load("sync_to_sd")
convert = load("convert_images_to_bmp")


class BmpToolsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / "source"
        self.target = self.root / "card"
        self.source.mkdir()
        self.target.mkdir()
        self.bmp = self.root / "reference.bmp"
        Image.new("RGB", (640, 480), (17, 33, 65)).save(self.bmp)
        self.bmp_bytes = self.bmp.read_bytes()

    def make_images(self, count):
        for i in range(count):
            (self.source / f"{i + 1:02d}.bmp").write_bytes(self.bmp_bytes)

    def snapshot(self):
        return {p.name: p.read_bytes() for p in self.target.iterdir() if p.is_file()}

    def test_sixteen_then_reduce_and_preserve_audio_other_files(self):
        self.make_images(18)
        for name in ("AUDIO.PCM", "OTHER.BMP", "PIC17.BMP", "notes.txt"):
            (self.target / name).write_bytes(b"keep")
        (self.target / "folder").mkdir()
        self.assertEqual(sync.sync_new_images(self.source, self.target), 16)
        for i in range(1, 17):
            self.assertEqual((self.target / f"PIC{i}.BMP").read_bytes(), self.bmp_bytes)
        self.assertEqual(sync.sync_new_images(self.source, self.target, 2), 2)
        self.assertFalse((self.target / "PIC3.BMP").exists())
        self.assertFalse((self.target / "PIC16.BMP").exists())
        for name in ("AUDIO.PCM", "OTHER.BMP", "PIC17.BMP", "notes.txt"):
            self.assertEqual((self.target / name).read_bytes(), b"keep")
        self.assertTrue((self.target / "folder").is_dir())

    def test_invalid_input_leaves_target_unchanged(self):
        (self.target / "PIC1.BMP").write_bytes(b"old")
        original = self.snapshot()
        with self.assertRaises(ValueError):
            sync.sync_new_images(self.source, self.target)
        self.make_images(2)
        bad = self.source / "02.bmp"
        mutations = [b"BM", b"not a bmp", self.bmp_bytes[:-1]]
        for offset, fmt, value in ((18, "<i", 320), (22, "<i", -480),
                                   (28, "<H", 32), (30, "<I", 1), (10, "<I", 20),
                                   (2, "<I", 54), (2, "<I", len(self.bmp_bytes) + 1)):
            payload = bytearray(self.bmp_bytes)
            struct.pack_into(fmt, payload, offset, value)
            mutations.append(payload)
        for payload in mutations:
            bad.write_bytes(payload)
            with self.assertRaises(ValueError):
                sync.sync_new_images(self.source, self.target)
            self.assertEqual(self.snapshot(), original)
        for count in (0, 17):
            with self.assertRaises(ValueError):
                sync.sync_new_images(self.source, self.target, count)
        for source, target in ((self.target, self.target), (self.root, self.target),
                               (self.source, self.root), (self.root / "missing", self.target)):
            with self.assertRaises(ValueError):
                sync.sync_new_images(source, target)
        self.assertEqual(self.snapshot(), original)

    def test_copy_failure_and_name_occupied_by_directory(self):
        self.make_images(2)
        (self.target / "PIC1.BMP").write_bytes(b"old")
        (self.target / "AUDIO.PCM").write_bytes(b"audio")
        original = self.snapshot()
        copy2 = sync.shutil.copy2
        calls = []

        def fail_second(source, target):
            calls.append(source)
            if len(calls) == 2:
                raise OSError("simulated copy failure")
            return copy2(source, target)

        with patch.object(sync.shutil, "copy2", side_effect=fail_second):
            with self.assertRaises(OSError):
                sync.sync_new_images(self.source, self.target)
        self.assertEqual(self.snapshot(), original)
        self.assertFalse(any(p.name.startswith(".bmp_sync_") for p in self.target.iterdir()))
        (self.target / "PIC2.BMP").mkdir()
        with self.assertRaises(ValueError):
            sync.sync_new_images(self.source, self.target)
        self.assertEqual(self.snapshot(), original)

    def test_conversion_alpha_resize_and_case_insensitive_bmp_extension(self):
        png = self.root / "alpha.png"
        Image.new("RGBA", (33, 21), (17, 33, 65, 100)).save(png)
        converted = self.source / "A.BMP"
        self.assertTrue(convert.convert_single_file(str(png), str(converted)))
        sync.validate_bmp(converted)
        with Image.open(converted) as image:
            self.assertEqual(image.size, (640, 480))
            self.assertEqual(image.mode, "RGB")
        (self.target / "pic1.bmp").write_bytes(b"old")
        sync.sync_new_images(self.source, self.target)
        self.assertEqual((self.target / "PIC1.BMP").read_bytes(), converted.read_bytes())
        self.assertEqual(sum(sync.MANAGED_NAME.fullmatch(p.name) is not None
                             for p in self.target.iterdir()), 1)


if __name__ == "__main__":
    with contextlib.redirect_stdout(io.StringIO()):
        unittest.main()
