# 把 re1_es8388 放到 GitHub，让树莓派 ESPHome 网页编译 Assist 固件

内置 `platform: es8388` 在你 RE1.0 板上会 `marked FAILED`。  
本仓库提供 **`re1_es8388`** 外部组件：寄存器序列与 `re1_audio` / PlatformIO 一致，供 `voice_assistant` 使用。

---

## 一、在 Windows 上把代码推到 GitHub（只做一次）

### 1. 注册 GitHub 并新建仓库

1. 打开 https://github.com 注册/登录  
2. 右上角 **+ → New repository**  
3. 仓库名填 **`mic`**（或别的名字，后面 YAML 里 URL 要一致）  
4. 选 **Public（公开）** — ESPHome 从 GitHub 拉代码需要公开仓库  
5. **不要**勾选 “Add a README”（本地已有代码）  
6. 点 **Create repository**

记下你的用户名，例如 `zhangsan`，仓库地址就是：

`https://github.com/zhangsan/mic`

### 2. 在本机初始化 Git 并推送

在 **PowerShell** 里执行（把路径和用户名换成你的）：

```powershell
cd D:\esphometest\mic

git init
git add .
git commit -m "Add re1_es8388 voice assistant component for RE1.0 board"

git branch -M main
git remote add origin https://github.com/2810113681/mic.git
git push -u origin main
```

第一次 `git push` 会要求登录 GitHub（浏览器或 Personal Access Token）。

推送成功后，在浏览器打开  
`https://github.com/2810113681/mic/tree/main/esphome/components/re1_es8388`  
应能看到 `audio_dac.py`、`re1_es8388.cpp`、`re1_es8388.h`。

---

## 二、在树莓派 ESPHome 网页里改 YAML

### 1. 改 GitHub 地址

打开 Device Builder 里 **tx** 的 YAML，在文件**最上面**（或 `external_components` 段）确保有：

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/2810113681/mic
      ref: main
      path: esphome/components
    components: [re1_es8388]
```

把 **`2810113681`** 换成你的 GitHub 用户名。  
若仓库名不是 `mic`，改 URL 里最后一段。

### 2. 把内置 es8388 换成 re1_es8388

**删除**这些块（若还在）：

```yaml
audio_dac:
  - platform: es8388
    ...

select:
  - platform: es8388
    ...
```

**改成**（或直接复制本目录 `tx_voice.yaml` 里对应段落）：

```yaml
audio_dac:
  - platform: re1_es8388
    id: re1_dac
    i2c_id: bus_a
    address: 0x10

speaker:
  ...
  audio_dac: re1_dac    # 原来是 es8388_dac
```

完整示例见 **`tx_voice.yaml`**。

### 3. Secrets

右上角 **⋮ → Secrets**，至少要有：

```yaml
wifi_ssid: "你的WiFi"
wifi_password: "你的WiFi密码"
api_encryption_key: "与HA里tx设备一致的密钥"
ota_password: "任意OTA密码"
```

### 4. 编译安装

1. **保存 → 校验**  
2. **安装**（WiFi OTA 或 USB）  
3. 打开 **日志**，正常应看到：

   ```
   [I][re1_es8388]: ES8388 init start  i2c_addr=0x10
   [I][re1_es8388]: ES8388 init done (output muted until playback)
   ```

   **不应再出现** `es8388.audio_dac is marked FAILED`。

---

## 三、Home Assistant 侧（你已配过可跳过）

1. **设置 → 语音助手**：助手用 **Home Assistant Cloud（中文）**  
2. **tx 设备页 → 配置 → 助手**：选 **Home Assistant Cloud**  
3. **按住 BTN1** 说「现在几点」→ 松开，听播报

---

## 四、常见问题

| 现象 | 处理 |
|---|---|
| 编译报找不到 `re1_es8388` | 检查 GitHub 是否 **Public**，URL/ref/path 是否正确 |
| 编译很慢 | 第一次会从 GitHub 拉组件，正常 |
| 仍报 `es8388 FAILED` | YAML 里还有 `platform: es8388`，改成 `re1_es8388` |
| 麦太小声 | `voice_assistant.microphone.channels` 改为 `1`，或 `gain_factor: 6` |
| 改了 GitHub 代码不生效 | YAML 里加 `refresh: 0s` 在 git source 下，或改 `ref` 后重新编译 |

强制每次重新拉 GitHub 组件：

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/2810113681/mic
      ref: main
      path: esphome/components
      refresh: 0s
    components: [re1_es8388]
```

---

## 五、以后只改网页 YAML 时

- **改 WiFi / voice_assistant / 按键**：直接在树莓派 ESPHome 网页改 YAML → OTA  
- **改 `re1_es8388` C++ 驱动**：在 Windows 改代码 → `git push` → 树莓派 ESPHome 重新 **安装** 固件
