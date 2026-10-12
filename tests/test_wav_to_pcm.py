import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest
import wave

spec = importlib.util.spec_from_file_location("wav_to_pcm", Path(__file__).resolve().parents[1] / "doc/convert/wav_to_pcm.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ConversionTests(unittest.TestCase):
    def test_signed_samples_and_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, target = root / "in.wav", root / "AUDIO.PCM"
            data = struct.pack("<5h", -32768, -1, 0, 1, 32767)
            with wave.open(str(source), "wb") as wav:
                wav.setparams((1, 2, 48000, 0, "NONE", "not compressed"))
                wav.writeframes(data)
            self.assertEqual(module.convert(source, target), 5)
            self.assertEqual(target.read_bytes(), data)
            with self.assertRaises(ValueError):
                module.convert(source, target)
            self.assertEqual(target.read_bytes(), data)
            with self.assertRaises(ValueError):
                module.convert(source, source)
            for channels, width, rate, payload in [(2, 2, 48000, data * 2), (1, 1, 48000, b"1234"), (1, 2, 44100, data), (1, 2, 48000, b"")]:
                with wave.open(str(source), "wb") as wav:
                    wav.setparams((channels, width, rate, 0, "NONE", "not compressed"))
                    wav.writeframes(payload)
                with self.assertRaises(ValueError):
                    module.convert(source, root / "bad.pcm")
                self.assertFalse((root / "bad.pcm").exists())
            with wave.open(str(source), "wb") as wav:
                wav.setparams((1, 2, 48000, 0, "NONE", "not compressed"))
                wav.writeframes(data)
            source.write_bytes(source.read_bytes()[:-2])
            with self.assertRaises(ValueError):
                module.convert(source, root / "truncated.pcm")
            self.assertFalse((root / "truncated.pcm").exists())


if __name__ == "__main__":
    unittest.main()
