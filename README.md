# N16R8 RE1.0 录放音项目（ESP32-S3 + ES8388）

这块板子是 **N16R8 扩展板 RE1.0**：ESP32-S3 N16R8（16 MB Flash + 8 MB OPI PSRAM）
+ ES8388 音频 codec + 板载双咪 MIC1/MIC2 + 喇叭功放。

目标是**让模块录下声音，再用喇叭把它播出来**——一个最小可用的本地录放回环
（也是 Voice Assistant、HASS Voice 这类应用的基础）。

仓库里同时提供两套**功能等价、配置同源**的固件：

| 工程 | 路径 | 用途 |
| --- | --- | --- |
| **PlatformIO 版** | 仓库根 `src/` + `platformio.ini` | 串口命令式的深度调试固件，带 `STATUS`/`PINPROBE`/`RXSNAP`/`REGDUMP` 等诊断指令 |
| **ESPHome 版** | `esphome/` | 自定义外部组件 `re1_audio`，**烧完按板上 BTN1 就能录放**，不需要 WiFi/HASS |

两边的 ES8388 寄存器序列、I2S 配置、(L−R)/2 差分混音都是同一套——
任何一个项目调好的参数都直接搬到了对方那里。

---

## 1. 硬件接线（已经实测、netlist 校对过）

| 信号 | ESP32-S3 GPIO | ES8388 引脚 | 方向 |
| --- | --- | --- | --- |
| I2C SDA | GPIO14 | SDA | 双向 |
| I2C SCL | GPIO47 | SCL | ESP→codec |
| MCLK | GPIO8 | MCLK | ESP→codec |
| BCLK / SCLK | GPIO3 | SCLK | ESP→codec |
| LRCK / WS | GPIO9 | LRCK | ESP→codec |
| **DOUT** | **GPIO46** | **DSDIN（pin6）** | **ESP→codec** （DAC 输入）|
| **DIN** | **GPIO10** | **ASDOUT（pin8）** | **codec→ESP** （ADC 输出）|
| BTN1 | GPIO12 | — | 输入，按下接 GND |
| BTN2 | GPIO11 | — | 输入，按下接 GND |
| BTN3 | GPIO13 | — | 输入，按下接 GND |
| LED | GPIO21 | — | 输出 |
| ES8388 I2C 地址 | — | 0x10 | — |

> **不要把 DOUT/DIN 反过来**：曾经一次"交换 pin 测试"把 ESP 的推挽 TX 接到了
> 编解码器自己的 ASDOUT 推挽输出上，存在烧坏输出驱动的风险。这版项目里
> `board_config.h`、`re1_audio.yaml` 都已经按 ES8388 QFN-28 datasheet
> Rev 5.0 (Jul 2018) 的引脚定义校正回来了。

---

## 2. 选哪一套？

* **要最快验证板子是好的、麦能录、喇叭能放**
  → 用 ESPHome 版。烧完拔线即可独立运行，按 BTN1 就回放。
* **要看到详细诊断信息、要自己改 codec 寄存器或排查链路**
  → 用 PlatformIO 版。串口里可以一行行打 `STATUS` / `RXSNAP` / `REGDUMP`。
* **要做 HASS Voice 集成**
  → 在 ESPHome 版的基础上加 `wifi:` + `api:` + `ota:`，`re1_audio` 组件本身
   不依赖网络。

两个项目共享同一块板子——任何一个烧进 ESP32-S3 都会覆盖另一个，来回切换烧录即可。

---

## 3. 详细文档

* `esphome/README.md`：ESPHome 版本的完整说明（**配置 / 功能 / 实现细节 /
  编译烧录 / 测试方法 / 调试踩坑**），是项目里最详细的一份文档。
* `src/board_config.h`、`src/main.cpp`、`src/es8388_codec.{h,cpp}`：
  PlatformIO 版本的源码，每一段配置上方都有注释说明为什么这么写。

---

## 4. 仓库里有哪些东西

```
mic/
├── README.md                      # ← 你正在看的这份
├── platformio.ini                 # PlatformIO 工程配置
├── remove_map.py                  # PlatformIO extra_script：解决 Windows 中文路径下 ld map 报错
├── test_voice_assistant_style.ps1 # PlatformIO 固件的录放测试脚本（5 秒录、自动回放）
├── src/                           # PlatformIO 源码（ES8388 驱动 + 命令行）
├── include/  lib/  test/          # PlatformIO 工程默认目录
├── .pio/  .vscode/                # 构建产物 / IDE 配置
└── esphome/                       # ESPHome 版本（独立子项目）
    ├── README.md
    ├── re1_audio.yaml             # 主配置（直接 `esphome run`）
    ├── esphome_run.ps1            # 一键脚本：装 esphome → 编译 → 烧录 → 串口
    ├── components/re1_audio/      # 自定义外部组件源码
    │   ├── __init__.py
    │   ├── re1_audio.h
    │   └── re1_audio.cpp
    ├── .esphome/                  # ESPHome 编译缓存（首次编译要拉工具链）
    └── .venv/                     # 本地 Python 3.10 虚拟环境（含 ESPHome + PlatformIO）
```
