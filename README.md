# mic — ESPHome external component for RE1.0 (ESP32-S3 + ES8388)

本仓库供 **Home Assistant ESPHome 网页** 通过 `external_components` 拉取使用。  
仅包含 RE1.0 板子实测可用的 ES8388 驱动 `re1_es8388`。

## 在 ESPHome YAML 里引用

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/2810113681/mic
      ref: main
      path: esphome/components
    refresh: 0s
    components: [re1_es8388]

audio_dac:
  - platform: re1_es8388
    id: re1_dac
    i2c_id: bus_a
    address: 0x10
```

`speaker` 里使用 `audio_dac: re1_dac`，不要用内置 `platform: es8388`（在 RE1.0 上会初始化失败）。

完整 Assist 配置见本地开发仓库中的 `esphome/tx_voice.yaml`（不随本 Git 仓库发布）。
