# 图片转换与 TF 卡同步

当前工程支持 FAT32 卡根目录的 `PIC1.BMP`～`PIC16.BMP`，最多 16 张图片。此工具统一转换为 **640×480、24 位 RGB、无压缩 BI_RGB、正高度（bottom-up）的 BMP**。音频自行按转换后的图片编号复制为 `AUDIO1.PCM`～`AUDIO16.PCM`，例如 `PIC5.BMP` 对应 `AUDIO5.PCM`；旧 `AUDIO.PCM` 不再播放。

## 环境准备

安装 Python 3 和 Pillow：

```powershell
python -m pip install Pillow
```

## 图片转换

在本目录运行：

```powershell
python convert_images_to_bmp.py
```

首次运行会创建脚本旁的 `input_images` 文件夹。将图片放进去，再运行脚本，转换结果位于脚本旁的 `output_bmp`。支持 JPG、PNG、BMP、WebP、GIF 等常见格式；GIF 取首帧，透明通道会去除，图片统一转换为 RGB 并拉伸到 640×480。

也可以指定单个输入与输出文件：

```powershell
python convert_images_to_bmp.py "C:\images\photo.png" "C:\images\converted\01.bmp"
```

## 同步到 TF 卡

确认卡已格式化为 FAT32。假设实际卡盘符为 `E:`，运行：

```powershell
python sync_to_sd.py E:
```

默认从脚本旁的 `output_bmp` 读取最多 16 张 BMP，按源文件名排序（忽略大小写），依次复制为卡根目录的 `PIC1.BMP`～`PIC16.BMP`。这是字符串排序；需要固定播放顺序时，建议源文件使用 `01.bmp`、`02.bmp`……`16.bmp`，避免 `10.bmp` 排在 `2.bmp` 前面。

指定其他来源及同步数量：

```powershell
python sync_to_sd.py E: -s "C:\images\converted" -n 8
```

`-n` 范围为 1～16；源图片少于指定数量时，只同步现有图片。复制前会检查所有选中图片的格式和像素数据完整性，错误或空源目录会报错。

脚本只管理根目录的 `PIC1.BMP`～`PIC16.BMP`：覆盖这些名称，并删除本次同步数量之外的旧 PIC 文件；保留所有 PCM、其他文件、`PIC17.BMP` 和目录。它不复制或自动匹配音频，改变图片排序后需自行调整 PCM 编号。它不会清空整张卡，也不会破坏旧 BMP 的 `BM` 文件头。当前工程按 FAT32 目录和簇链读取，不需要处理已删除文件的物理残留。

脚本先完成全部临时副本，再逐个替换目标文件；临时复制失败时旧 PIC 文件不变。整组替换不是原子操作，替换阶段失败可能留下新旧混合的 PIC 文件，排除问题后重新运行即可。

同步完安全弹出卡，插入开发板，按 KEY1 复位重新扫描。手工复制时请使用上述大写 8.3 文件名，放在卡根目录，不放入子文件夹。目录中可缺号，例如只有 `PIC1.BMP`、`PIC5.BMP`，工程会按数字顺序播放。

更多播放、按键、缓存与下载说明见 [多图片播放](../多图片播放.md)，音频格式见 [PCM 音频播放](../PCM音频播放.md)。
