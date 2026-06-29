# RE1 Audio - ESPHome 项目（详细说明）

这是仓库根 PlatformIO 工程的 **ESPHome 等价版本**。硬件就是 N16R8 RE1.0 扩展板：
ESP32-S3（16 MB Flash + 8 MB OPI PSRAM）+ ES8388 codec + 板载双咪 + 喇叭功放。

它**不需要** WiFi、不需要 Home Assistant、不需要 ESPHome Dashboard 账号——
插上 USB 烧录后，按板上 BTN1 就能录音 5 秒，自动从喇叭回放出来。

下面这份说明是一站式的：

* 第 1 章：目录结构 + 文件作用
* 第 2 章：硬件配置（引脚、I2C 地址、按键、LED）
* 第 3 章：软件架构（外部组件、YAML、一键脚本）
* 第 4 章：功能与实现细节（ES8388 寄存器、I2S 驱动、录放流程、统计）
* 第 5 章：编译与烧录
* 第 6 章：测试方法与日志解读
* 第 7 章：常见问题与踩坑总结
* 第 8 章：扩展为带 WiFi / HASS Voice 的版本

---

## 1. 目录结构

```
esphome/
├── re1_audio.yaml                 # 主配置文件，直接 `esphome run` 它
├── esphome_run.ps1                # 一键脚本：装 esphome → 编译 → 烧录 → 串口
├── README.md                      # ← 这份文档
├── .gitignore
├── components/
│   └── re1_audio/                 # 本地 ESPHome 外部组件
│       ├── __init__.py            #   Python schema：YAML 配置 → C++ 调用
│       ├── re1_audio.h            #   C++ 头：类定义 / 状态机 / 公开 actions
│       └── re1_audio.cpp          #   C++ 实现：ES8388 init + I2S + 录放循环
├── .venv/                         # 本地 Python 3.10 虚拟环境（含 ESPHome + PlatformIO）
└── .esphome/                      # ESPHome 编译缓存（首次编译要拉 ESP32 toolchain）
```

* `re1_audio.yaml`、`esphome_run.ps1`、`components/re1_audio/` 这三个是**项目源码**。
* `.venv/`、`.esphome/` 是构建/工具产物，删掉后下次编译会重新拉一次（很慢）。

---

## 2. 硬件配置

| 项目 | 值 | 说明 |
| --- | --- | --- |
| MCU | ESP32-S3 N16R8 | 16 MB Flash + 8 MB OPI PSRAM |
| Codec | ES8388 | I2C 地址 `0x10`，I2S 从模式（codec slave） |
| I2C SDA / SCL | GPIO14 / GPIO47 | 100 kHz |
| I2S MCLK | GPIO8 | 256 × Fs，由 ESP 的 APLL 产生 |
| I2S BCLK | GPIO3 | 位时钟 |
| I2S LRCK / WS | GPIO9 | 帧时钟 |
| **I2S DOUT** | **GPIO46** | ESP TX → codec **DSDIN**（DAC 数据进入 codec） |
| **I2S DIN** | **GPIO10** | codec **ASDOUT** → ESP RX（ADC 数据离开 codec） |
| MIC 路由 | LIN1 / RIN1 差分 | 板上 MIC1+MIC2 是反相一对 |
| BTN1 | GPIO12 | 录 5 秒并自动回放 |
| BTN2 | GPIO11 | 重新回放上一次的录音 |
| BTN3 | GPIO13 | 串口里打一行诊断 |
| 状态 LED | GPIO21 | 二值灯 |
| 采样率 | 24 000 Hz | 与 PlatformIO 版本一致 |
| 录音缓存上限 | 10 秒 | PSRAM 中的 mono int16 = 480 KB |

> 引脚来自 netlist 实测 + ES8388 QFN-28 datasheet (Rev 5.0, Jul 2018) 校对。
> 之前怀疑 GPIO10/46 反了所以做过"swap test"，那是误诊，**绝对不要再交换**。

---

## 3. 软件架构

### 3.1 外部组件 `re1_audio`

ESP32 + ES8388 这块组合 ESPHome 官方还**没有一类组件**支持，所以这里把验证过的
配置封装成了一个本地 external component，YAML 通过 `external_components: type: local`
直接引入。

| 文件 | 作用 |
| --- | --- |
| `components/re1_audio/__init__.py` | 用 `cv.Schema` 声明 YAML 字段，并把它们映射到 C++ setter |
| `components/re1_audio/re1_audio.h` | `class Re1Audio : public Component, public i2c::I2CDevice`，状态机 + 缓冲区 + 公开 actions |
| `components/re1_audio/re1_audio.cpp` | ES8388 寄存器序列、I2S 安装、录音/回放每 tick 的处理 |

公开三个 action（YAML 里用 `lambda` 调）：

```cpp
id(audio).record_then_play(uint32_t seconds);  // 录 N 秒，结束后自动回放
id(audio).play_last();                          // 重新放一次最近的录音
id(audio).log_diag();                           // 串口打一条 DIAG
```

### 3.2 YAML：`re1_audio.yaml`

主要分四部分：

1. `esphome:` / `esp32:` / `psram:` / `logger:`
   开 PSRAM 的 OPI 模式，日志走 ESP32-S3 的原生 USB-CDC（也就是板上"USB"那个口，
   不是 CH343 那个）。
2. `external_components:`
   把本地 `components/` 下的 `re1_audio` 加载进来。
3. `i2c:` + `re1_audio:`
   I2C 总线 (SDA=14, SCL=47, 100kHz)，再实例化 `re1_audio: id: audio`，把所有
   引脚 / 采样率 / 录音缓存上限等参数传进去。
4. `binary_sensor:` × 3 + `output:` + `light:`
   把三个按键和一个 LED 接入 ESPHome，按下时调用对应的 lambda action。

YAML **故意去掉了** `wifi:` / `api:` / `ota:`——这是为了**完全离线运行**：你不需要
任何账号、不需要 HASS、不需要无线网就能验证板子是否好。需要联网时见第 8 章。

### 3.3 一键脚本 `esphome_run.ps1`

```powershell
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1               # 编译+烧录+日志
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1 -CompileOnly  # 只编译
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1 -LogsOnly     # 只看串口
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1 -Port COM6    # 指定端口
```

脚本会：

1. 用 `Get-Command esphome` 检查 ESPHome 是否可用，没装就 `pip install -U esphome>=2024.2.0`；
2. 默认走 "compile+upload→logs" 流程，可用 `-LogsOnly` 或 `-CompileOnly` 切换；
3. 始终在 `esphome/` 目录下执行（用 `Push-Location $PSScriptRoot`）。

---

## 4. 功能与实现细节

### 4.1 状态机

`Re1Audio` 内部只有三个状态：

```
kIdle  ──record_then_play(N)──▶  kRecording  ──采满 N×Fs 个样本──▶  kPlaying
  ▲                                                                   │
  └────────────────── stop_playback_(end_of_buffer) ───────────────────┘
```

`play_last()` 会从 `kIdle` 直接进入 `kPlaying`，不会再录一次。
所有录放的"每一帧"工作都在 ESPHome 的 `Component::loop()` 里被周期性调用，
每次最多读/写 256 个 stereo 帧（1024 字节），不会阻塞主循环。

### 4.2 ES8388 初始化（`codec_init_()`）

这是**最容易踩坑**的部分，每个寄存器写值 + 含义都列在下表，与 `re1_audio.cpp`
里的注释一一对应：

| Reg | Val | 含义 |
| --- | --- | --- |
| 0x08 | 0x00 | MASTERMODE = codec slave（ESP 当 I2S master） |
| 0x01 | 0x50 | CONTROL2：bias / reference 上电 |
| 0x02 | 0x00 | CHIPPOWER：normal |
| 0x00 | 0x16 | CONTROL1：play + record，开 EnRef |
| 0x35 | 0xA0 | DLL tuning（参考工程通用值） |
| 0x37 | 0xD0 | LOUT tuning |
| 0x39 | 0xD0 | ROUT tuning |
| 0x04 | 0xC0 | DAC 输出**关闭**（init 期间） |
| 0x17 | 0x18 | DACCONTROL1 = 16-bit + I2S Philips |
| 0x18 | 0x02 | DAC 256 Fs |
| 0x1A/0x1B | 0x00 | DAC 左右音量 0 dB |
| 0x26 | 0x00 | DAC mixer source |
| 0x27 | 0x90 | left mixer：仅 DAC |
| 0x2A | 0x90 | right mixer：仅 DAC |
| 0x2B | 0x80 | ADC/DAC 共用 LRCK |
| 0x2D | 0x00 | analog out R 默认 |
| 0x2E/2F/30/31 | 0x1E | LOUT1 / ROUT1 / LOUT2 / ROUT2 音量 |
| 0x03 | 0xFF | ADC 先全部下电 |
| 0x09 | 0x88 | **ADC PGA 增益 L=R=24 dB**（驻极体麦需要 ≥18 dB 才听得清） |
| 0x0A | 0xF0 | ADC route：differential |
| 0x0B | 0x02 | ADC control3：选 LIN1/RIN1 差分对 |
| **0x0C** | **0x0C** | **ADCCONTROL4：bit[3:2]=11(16bit) + bit[1:0]=00(I2S Philips)。<br>这里如果写成 `0x0D`（Left-Justify）就会得到非常大的电磁声！** |
| 0x0D | 0x02 | ADC 256 Fs |
| 0x0F | 0x00 | ADC unmute + HPF off |
| 0x10/0x11 | 0x00 | ADC 左右音量 0 dB |
| 0x12 | 0x00 | ALC 关 |
| 0x13/14/15/16 | 0xA0/0x12/0x06/0x00 | ALC 默认参数（关掉了所以也无所谓） |
| 0x03 | 0x09 | ADC 上电 + **MICBIAS 开**（驻极体必须的偏置电压） |
| 0x02 | 0xF0 → 0x00 | 两阶段 power up |

最后还会 `codec_set_dac_mute_(true)` + `codec_set_output_enabled_(false)`，
让喇叭在 boot 阶段是哑的，**避免开机 pop**。播放开始时再依次解 mute、开输出，
播放结束再 mute 回去。

> 这套配置和仓库根 `src/es8388_codec.cpp` 一模一样，都是反复测出来的。

### 4.3 I2S 驱动（`i2s_install_()`）

```cpp
i2s_config_t cfg = {};
cfg.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX;   // 全双工
cfg.sample_rate          = 24000;
cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;    // stereo
cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;     // ★ 必须和上面 0x0C 对齐
cfg.dma_buf_count        = 6;
cfg.dma_buf_len          = 256;
cfg.use_apll             = true;                          // 用 APLL 拉准 24 kHz
cfg.tx_desc_auto_clear   = true;
cfg.mclk_multiple        = I2S_MCLK_MULTIPLE_256;
```

`use_apll=true` 是为了让 24 kHz 的 BCLK / LRCK 是真的 24 kHz，不是 23.99x kHz。
`tx_desc_auto_clear=true` 是为了在 TX 缺数据时 DMA 自动塞 0，不会重复播上一帧
（重复播会有"嗡"声）。

### 4.4 录音流程（`step_recording_()`）

```
        ┌────────────┐    每次 256 帧             ┌──────────────────┐
ES8388 ─┤ ASDOUT(L+R)├─► I2S DMA ── i2s_read ──► │ for each frame:  │
        └────────────┘                           │   l = buf[2i]    │
                                                 │   r = buf[2i+1]  │
                                                 │   m = (l-r)/2    │
                                                 │   record_buffer_ │
                                                 │       [n++] = m  │
                                                 └──────────────────┘
```

* **`(L − R) / 2` 而不是 `(L + R) / 2`**：板上 MIC1 / MIC2 物理上是反相对的差分
  对，`L+R` 会让两路声波互相抵消，导致录到的人声 RMS 接近 0；改成 L−R 之后两路
  同相叠加，幅度大约翻倍。**这是仅次于 ADC 格式之外最容易踩的坑。**
* 录音时 DAC 输出已被关掉（mute + output disable），保证录到的不是回放残留。
* 边录边累计统计：min/max(L)、min/max(R)、min/max(M)、∑|M|、frame 数。
  录满之后一行 log 打出来：

  ```
  [I][re1_audio]: REC stop reason=target_reached samples=120000 ms=5012
                  L[-1700,1800] R[-1850,1750] M[-14000,14500] avg|M|=2900
  ```

  `avg|M|` 是判断"麦到底有没有收到声音"的最直观指标——
  * 完全安静的房间应该 **< 50**
  * 正常说话应该 **几百到几千**
  * 全 0 = 麦没接进来或 PGA/MICBIAS 没开
  * `M` 范围接近满量程（±32767）+ `avg|M|` 巨大 = 很可能在自激或 ADC 格式错

* 录满后 `start_playback_()` 会自动接力，不需要再按一次按钮。

### 4.5 回放流程（`step_playing_()`）

```
   ┌──────────────────┐  每次 256 个 mono 样本 ┌────────┐
   │ record_buffer_   ├────复制成 stereo──────►│ I2S TX │──► ES8388 DAC ──► 喇叭
   │ [pos] → s        │   buf[2i]=buf[2i+1]=s └────────┘
   └──────────────────┘
```

* 播放开始时先 `codec_set_output_enabled_(true)` 再 `codec_set_dac_mute_(false)`，
  顺序不能反——出 mute 之前必须先把输出通路打开，否则会有一段空白。
* 缓冲耗尽（`play_pos_ == record_filled_`）后立刻把 DAC mute 回去并把输出关掉，
  避免后面持续吐 0 进喇叭驱动。

### 4.6 PSRAM 缓冲区

```cpp
record_capacity_ = sample_rate_ * record_seconds_max_;     // 24000 * 10 = 240000 samples
record_buffer_   = heap_caps_malloc(capacity * 2, MALLOC_CAP_SPIRAM);
```

mono int16 占 2 字节 → 10 秒最大 = 480 KB。OPI PSRAM 8 MB 完全装得下，
即使 `record_seconds_max` 拉到 30 秒也只有 1.4 MB。如果 PSRAM 申请失败
（比如 YAML 里忘了写 `psram:`），会回退到 DRAM 申请；DRAM 装不下时
`mark_failed()`，组件直接进入 fail 状态，串口里会有 FATAL 日志。

---

## 5. 编译与烧录

### 5.1 环境要求

| 依赖 | 版本 | 说明 |
| --- | --- | --- |
| Python | **3.10 / 3.11 / 3.12 / 3.13** | **3.14 不行**，ESPHome 暂不兼容 |
| ESPHome | ≥ 2024.2.0 | `pip install -U esphome` |
| 串口驱动 | CP210x / CH343 / 自带 | Windows 下确认设备管理器有 COMx |
| 操作系统 | Windows / Linux / macOS | 本仓库的脚本是 PowerShell，命令行版三大平台通用 |

仓库里的 `.venv/` 已经是一个干净的 Python 3.10 虚拟环境，里面装好了
`esphome` + `platformio`，你可以直接：

```powershell
cd D:\esphometest\mic\esphome
.\.venv\Scripts\Activate.ps1
esphome version    # 验证装好了
```

如果 `.venv/` 被你删了，在 esphome 目录下重新建一份：

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
python -m pip install -U "esphome>=2024.2.0"
```

### 5.2 一键脚本

```powershell
cd D:\esphometest\mic\esphome
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1 -Port COM6
```

第一次会拉 ESP32 toolchain（xtensa-esp-elf-gcc、riscv32-esp-elf-gcc、esptool 等），
**视网速大约 5–15 分钟**。之后增量编译几十秒就好。

只编译不烧录：

```powershell
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1 -CompileOnly
```

只看日志（设备已经烧好了）：

```powershell
powershell -ExecutionPolicy Bypass -File .\esphome_run.ps1 -Port COM6 -LogsOnly
```

### 5.3 纯命令行

```powershell
cd D:\esphometest\mic\esphome
esphome compile re1_audio.yaml                       # 仅编译
esphome run     re1_audio.yaml --device COM6         # 编译 + 烧录 + 串口
esphome logs    re1_audio.yaml --device COM6         # 仅串口日志
esphome upload  re1_audio.yaml --device COM6         # 仅烧录已编译的固件
```

固件构建产物在 `.esphome/build/re1-audio/.pioenvs/re1-audio/firmware.bin`。

---

## 6. 测试方法与日志解读

### 6.1 验收流程

烧好后用 `esphome logs ... --device COM6` 看串口，正常的 boot 日志：

```
[I][re1_audio]: setup() begin
[I][re1_audio]: record buffer allocated 480000 bytes (240000 samples / 10s max)
[I][re1_audio]: ES8388 init start  i2c_addr=0x10
[I][re1_audio]: ES8388 init done (output muted until first PLAY)
[I][re1_audio]: I2S installed sr=24000 bits=16 mclk=8 bclk=3 lrck=9 din=10 dout=46
[I][re1_audio]: setup() done. Press BTN1 to record-and-play 5s, BTN2 to replay, BTN3 to log diag.
```

**对着麦讲话**同时**按一下 BTN1**，串口应该出现：

```
[I][re1_audio]: REC start seconds=5
[I][re1_audio]: REC progress 24576/120000 (20.5%)
[I][re1_audio]: REC progress 48128/120000 (40.1%)
[I][re1_audio]: REC progress 71680/120000 (59.7%)
[I][re1_audio]: REC progress 95232/120000 (79.4%)
[I][re1_audio]: REC stop reason=target_reached samples=120000 ms=5012
                L[-1700,1800] R[-1850,1750] M[-14000,14500] avg|M|=2900
[I][re1_audio]: PLAY start samples=120000 (5000 ms)
[I][re1_audio]: PLAY progress 24064/120000 (20.1%)
[I][re1_audio]: PLAY progress ...
[I][re1_audio]: PLAY stop reason=end_of_buffer pos=120000/120000 ms=5012
```

同时**喇叭里应该听到自己刚才说的话**——如果听不到，看下面 6.2。

### 6.2 录放健康检查表

| 看到的日志 | 意味着 | 该排查的方向 |
| --- | --- | --- |
| `L[0,0] R[0,0] M[0,0] avg\|M\|=0` | I2S 完全没收到 ADC 数据 | DIN 接错 / ADC 没上电 / MCLK 没出来 |
| `avg\|M\| < 50`，但能录满 | 麦没声音 | PGA 增益 0x09 / MICBIAS 0x03=0x09 / 板上麦坏 |
| `avg\|M\|` 几万、`M[-32768,32767]` | 信号溢出/自激 | PGA 太大 / 反馈环路 / ADC 格式错 |
| `avg\|M\|` 正常但喇叭没声 | DAC 路径有问题 | DOUT 接错 / DAC mute 没解 / 喇叭功放 EN 没开 / 接地 |
| 满屏全是嗡嗡声/电磁声 | I2S 数据帧错位 | ADCCONTROL4 (0x0C) 写成了 0x0D |
| 录到的能放，但音量很小 | L/R 互相抵消 | 没用 `(L-R)/2`，被改回 `(L+R)/2` 了 |

### 6.3 三个按键的快速验证

| 按键 | 期望 |
| --- | --- |
| BTN1 (GPIO12) | 串口 `REC start...` 5 秒后自动 `PLAY start...`，喇叭里听到自己 |
| BTN2 (GPIO11) | 串口直接 `PLAY start...`（重新放上一次的录音） |
| BTN3 (GPIO13) | 串口出现 `DIAG state=0 codec_ok=1 i2s=1 sr=24000 ...` |

按键是低电平触发（`inverted: true` + `pullup: true`），按下接 GND 即生效。
如果按 BTN1 没反应，先用 BTN3 验证按键和 ESPHome 框架是通的。

### 6.4 自动化测试（PlatformIO 版本）

仓库根 `test_voice_assistant_style.ps1` 是给 PlatformIO 固件用的"录 5 秒+回放"
自动测试。**ESPHome 版没有串口命令接口**（按键直接触发）所以那个脚本对
ESPHome 无效，但你可以仿照它写一个：

* 打开串口，等 boot 完
* 用 ESPHome 的 `MQTT` / `API` / 或者把按键暴露成 service 来远程触发
* 监控 `REC stop ...` 那一行的 `avg|M|`

最简单的"我说话它放出来吗"还是按一下 BTN1 用耳朵听最直接。

---

## 7. 常见问题与踩坑总结

按照踩坑顺序记录，方便以后换板子时不再趟一遍：

### 7.1 喇叭没声 → ADC 数据全 0

* 刚开始 I2S 没收到任何字节，`avg|M|=0` 而且 L/R 都不动。
* 原因：`PIN_I2S_DOUT` 和 `PIN_I2S_DIN` 配反了——把 ESP 的 TX 接到 codec 的输出
  上去了，逻辑上等同于让两个推挽驱动器对打。
* 修复：按 ES8388 QFN-28 datasheet（Rev 5.0, Jul 2018）：
  pin 6 DSDIN ← ESP TX (GPIO46)，pin 8 ASDOUT → ESP RX (GPIO10)。

### 7.2 喇叭里全是巨大的电磁噪声

* I2S 链路通了，能录到东西，但放出来不是人声而是嗡嗡声+尖啸。
* 原因：`ADCCONTROL4 (reg 0x0C)` 写成了 `0x0D`，那是 **Left-Justify** 格式；
  ESP32 的 `I2S_COMM_FORMAT_STAND_I2S` 是 **I2S Philips**，差一个 BCLK 的相位，
  每个 16-bit sample 都被错位采样，结果就是乱码音频。
* 修复：写 `0x0C`（bit[3:2]=11(16bit) + bit[1:0]=00(I2S Philips)）。
  这一行在 `re1_audio.cpp` 里有醒目注释。

### 7.3 录到的音很小，几乎听不见

* 寄存器都对、PGA 24 dB、MICBIAS 开了、串口里 L 和 R 都有几千的幅度，但
  混完之后 `avg|M|` 只有几十。
* 原因：板上 MIC1/MIC2 是物理反相的差分对，做 `(L+R)/2` 会让两路抵消。
* 修复：改成 `int32_t mono = ((int32_t)l - (int32_t)r) / 2;`
  如果换的是别的板子，两路同相，那要改回 `+`。判断方法：录一段稳定的语音，
  看串口 `L[..,..] R[..,..]` 是同号还是异号。

### 7.4 ESPHome 编译报 `Python version must be 3.10/3.11/3.12/3.13`

* 系统默认 Python 是 3.14。
* 修复：装 Python 3.10.x，并用它建虚拟环境：

  ```powershell
  py -3.10 -m venv .venv
  .\.venv\Scripts\Activate.ps1
  python -m pip install -U "esphome>=2024.2.0"
  ```

### 7.5 ESPHome 编译报 `Component api requires component network`

* `re1_audio.yaml` 里有 `api:` 但没有 `wifi:`/`ethernet:`，offline 用不上 api。
* 修复（本项目已经做过）：直接把 `api:` 和 `ota:` 整段删掉。需要 HASS 时再加，
  见第 8 章。

### 7.6 PlatformIO 装包时 `package-postinstall.py` 在 Windows 上失败

* 报错形如：

  ```
  subprocess.CalledProcessError: Command 'package-postinstall.py' returned non-zero exit status 1.
  ```

* 原因：在某些 Windows + 中文/括号路径环境下 PATHEXT 不会把 `.py` 当可执行文件
  直接 spawn，PlatformIO 没显式用 `python xxx.py` 调用。
* 修复：补丁 `.venv/lib/site-packages/platformio/package/manager/base.py`，
  把所有以 `.py` 结尾的命令统一改成 `[PIO_PYTHON_EXE, *cmd]` 再 `subprocess.run`。
  本仓库 `.venv/` 已经打过这个补丁，开箱可用。如果你重建了 venv，重新装一遍
  `esphome` + `platformio`，第一次出现这个报错时再打补丁即可。

### 7.7 编译报 `'delay' was not declared` / `'millis' was not declared`

* C++ 文件里直接用 Arduino 的 `delay()` / `millis()`，但没 include。
* 修复：`re1_audio.cpp` 里加上 `#include "esphome/core/hal.h"`。
  HAL 内部映射到 Arduino 框架，正常使用即可。

---

## 8. 扩展为带 WiFi / HASS Voice 的版本

`re1_audio` 组件本身和有没有网络无关。要加 HASS 集成，`re1_audio.yaml` 里
增加：

```yaml
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
  encryption:
    key: !secret api_key

ota:
  - platform: esphome
    password: !secret ota_password
```

并在同目录建 `secrets.yaml` 填好上述值。

要把"录 5 秒并回放"暴露给 HASS 当一个 service：

```yaml
button:
  - platform: template
    name: "Record and play 5s"
    on_press:
      then:
        - lambda: |-
            id(audio).record_then_play(5);
```

或者把 `record_then_play` 接到 HASS Voice 的 ESPHome `voice_assistant:` 组件
（需要按官方方案再补 `microphone:` / `speaker:` 桥接，那个会比当前组件复杂得多，
通常的做法是再加一层把 `Re1Audio` 的录音帧经 ringbuffer 转成
`microphone::Microphone` 接口）。这部分超出了本项目"先证明板子能录能放"的目标，
作为 TODO 留给后续工作。

---

## 9. 与 PlatformIO 项目的关系

| 维度 | PlatformIO 版（`src/`） | ESPHome 版（`esphome/`） |
| --- | --- | --- |
| 目标 | 深度调试、命令行手动控制 | 一键演示、可嵌入 HASS |
| 触发方式 | 串口命令 `REC_START` / `REC_STOP` / `PLAY` | 按键 BTN1/BTN2/BTN3 |
| 诊断 | `STATUS` / `PINPROBE` / `RXSNAP` / `REGDUMP` | `log_diag()`（一行） |
| 按键 | 不监听按键 | 监听 GPIO12/11/13 |
| LED | 用，作为状态指示 | 配了 `output:`，可在 lambda 里点亮 |
| 网络 | 没有 | 默认离线，可加 wifi/api/ota |
| ES8388 寄存器序列 | 同源 | 同源 |
| `(L-R)/2` 差分混音 | 是 | 是 |
| 采样率 / 缓冲 | 24 kHz / 上限 10 s | 24 kHz / 上限 10 s |

任何一个项目调好的 codec 寄存器、混音逻辑都可以**直接搬到对方**——这也是为什么
本仓库会同时保留两套实现：调试时用 PlatformIO，落地时用 ESPHome。
