# Waveshare ESP32-C6-DEV-KIT-N16 板级支持

为微雪 **ESP32-C6-DEV-KIT-N16**（ESP32-C6-WROOM-1，16MB Flash）定制的
小智 AI 语音聊天固件，适配以下 DIY 外设：

| 外设 | 型号 | 接口 |
| --- | --- | --- |
| 功放 | MAX98357A I2S 功放模块 | I2S（共用时钟全双工） |
| 喇叭 | 40mm 内磁 4Ω 3W 全频喇叭 | 接 MAX98357 喇叭端子 |
| 麦克风 | INMP441 I2S 麦克风模块 | I2S（共用时钟全双工） |
| 红外收发 | NEC 红外收发一体模块（YS-IRTM 兼容） | UART 9600（收发双向） |
| 空调控制 | 940nm 红外发射管 + 100Ω 电阻 | GPIO10（RMT 直驱 38kHz） |

板载 WS2812 RGB LED（GPIO8）用于显示设备状态，BOOT 键（GPIO9）可
手动开始/停止对话。

## 接线表（默认：全双工共用时钟，推荐）

MAX98357 与 INMP441 共用 BCLK / WS 两条时钟线，信号线共 4 根：

### MAX98357 功放模块

| 模块引脚 | 接到 ESP32-C6 | 说明 |
| --- | --- | --- |
| VIN | 5V（USB 供电时可用板上 5V） | 2.5~5.5V，5V 时输出功率最大（4Ω 负载约 3W） |
| GND | GND | 共地 |
| DIN | GPIO6 | I2S 数据（ESP → 功放） |
| BCLK | GPIO4 | 与 INMP441 SCK 并接 |
| LRC | GPIO5 | 与 INMP441 WS 并接 |
| SD | 悬空 | 悬空 = (L+R)/2 单声道输出 |
| GAIN | 悬空 | 悬空 = 9dB 增益（接 GND 12dB / 接 3.3V 6dB） |

喇叭（4Ω 3W）接 MAX98357 的 `Speaker +` / `Speaker -` 端子。

### INMP441 麦克风模块

| 模块引脚 | 接到 ESP32-C6 | 说明 |
| --- | --- | --- |
| VDD | 3.3V | 只能接 3.3V |
| GND | GND | 共地 |
| SCK | GPIO4 | 与 MAX98357 BCLK 并接 |
| WS | GPIO5 | 与 MAX98357 LRC 并接 |
| SD | GPIO7 | I2S 数据（麦克风 → ESP） |
| L/R | GND | 接 GND = 左声道（必须接） |

### NEC 红外收发一体模块（YS-IRTM 兼容，UART）

模块既接收遥控器信号（解码后从 TXD 输出），也能发射 NEC 红外码
（ESP 从 TXD 引脚写入指令，控制电视/机顶盒等）。

| 模块引脚 | 接到 ESP32-C6 | 说明 |
| --- | --- | --- |
| VCC | 5V | 模块多为 5V 供电（3.3V 供电会缩短发射距离） |
| GND | GND | 共地 |
| TXD | → 1kΩ → GPIO21 → 2kΩ → GND | **建议分压**：模块 TXD 输出 5V TTL，直连可能损伤 ESP 引脚 |
| RXD | GPIO20 | ESP 发射指令 → 模块（不接则无法发射，仅能接收） |

- 波特率默认 9600，若不同请改 `config.h` 的 `IR_REMOTE_BAUD_RATE`
- 模块通信地址默认 0xA1（部分批次为 0xFA），不匹配时改 `IR_REMOTE_CMD_ADDR`
- 解码输出自动兼容两种格式：4 字节标准 NEC 序列、3 字节 YS-IRTM 格式

### 空调红外发射管（GPIO10，分立元件）

空调（格力/美的/海尔）使用专有红外协议（非 NEC），收发一体模块无法
发射，需要一颗独立的红外发射管：

```
GPIO10 ── 100Ω电阻 ──▶|── 发射管正极(长脚)   发射管负极 ── GND
                        （940nm 波长，正向）
```

- 直驱发射距离约 1-2 米；如需 5-8 米，加一个 S8050 NPN 三极管驱动
  （GPIO10 → 1kΩ → 基极；发射管 + 100Ω 接集电极；发射极 → GND）
- 发射管需朝向空调（红外不能穿墙，需视线可达）
- 支持品牌：**格力（gree）/ 美的（midea）/ 海尔（haier）**，
  时序协议移植自 IRremoteESP8266 开源库

## 备选接线：简单模式（各自独立时钟）

如全双工共用时钟在你的硬件上出现干扰，可改用 6 根信号线的简单模式：
编辑本目录 `config.h`，注释掉 `#define AUDIO_I2S_METHOD_DUPLEX`，按下表接线：

| 功能 | ESP32-C6 引脚 |
| --- | --- |
| MAX98357 BCLK / LRC / DIN | GPIO4 / GPIO5 / GPIO6 |
| INMP441 SCK / WS / SD | GPIO2 / GPIO3 / GPIO7 |

## 红外遥控键值说明

默认按最常见的 NEC 编码遥控器（地址 0x00FF，如 HX1838 套件、
Elegoo 遥控器）预置了按键映射：

| 遥控器按键 | NEC 命令码 | 功能 |
| --- | --- | --- |
| OK | 0x0C | 唤醒小智 / 开始-停止对话 |
| 数字 1 | 0x45 | 同 OK（备用） |
| 上箭头 | 0x18 | 音量 +10 |
| 右箭头 | 0x1B | 音量 +10（备用） |
| 下箭头 | 0x52 | 音量 −10 |
| 左箭头 | 0x5A | 音量 −10（备用） |
| * 键 | 0x19 | 静音 / 取消静音 |
| # 键 | 0x0D | 重新进入 WiFi 配网 |

**换用其他遥控器**：烧录后打开串口监视器（115200），按遥控器按键，
日志会输出 `IR-NEC: addr=0x.... cmd=0x..`。把对应的 `cmd` 值填到
`config.h` 的 `IR_NEC_CMD_*` 宏中重新编译即可。也可以在
`IR_NEC_ADDRESS_FILTER` 填入遥控器地址，只响应指定遥控器。

无法解析的帧会在日志中打印 `Unparsed IR frame (n bytes): .. ..`，
方便确认模块的实际输出协议。

## 语音控制红外设备（MCP 工具）

固件注册了两个 MCP 工具，对话中小智会自动调用：

| 语音指令示例 | 触发的工具 | 说明 |
| --- | --- | --- |
| “打开空调” / “空调调到 26 度制冷” | `self.ir_ac_control` | 经 GPIO10 发射管发空调协议 |
| “把电视关了” / “换个台” | `self.ir_send_nec` | 经 YS-IRTM 模块发 NEC 码 |

- 空调品牌默认 `IR_AC_DEFAULT_BRAND`（config.h 可改为 gree/midea/haier），
  语音中直接说“用美的协议”也能指定
- 电视等 NEC 设备需要知道设备的 NEC 码（用户码 + 命令码），可通过本模块
  学习：拿电视遥控器对着模块按一下，从串口日志 `IR-NEC: addr=0x.... cmd=0x..`
  读出键值，再让小智发射相同码即可（如“帮我发红外码 FF 00 关电视”）
- YS-IRTM 模块发射指令格式：`A1 F1 [用户码高] [用户码低] [命令码]`

## 编译

### 方式一：GitHub Actions 自动编译（推荐）

把本仓库推送到你的 GitHub 账号下，push 后 Actions 会自动编译本板子
固件，在 Actions 页面的 Artifacts 中下载 `merged-binary.bin` 即可烧录。

### 方式二：本地编译

```bash
# 安装 ESP-IDF v6.0.x 后
cd xiaozhi-esp32
python scripts/build.py waveshare/esp32-c6-dev-kit-n16 --name esp32-c6-dev-kit-n16
# 产物：build/merged-binary.bin
```

## 烧录

- **网页烧录**：使用 xiaozhi.me 提供的在线烧录工具，选择自定义固件
  （merged-binary.bin）
- **命令行**：`esptool --chip esp32c6 write_flash @build/flash_args`
  （在 build 目录执行，或用 `idf.py flash`）
- 下载模式：按住 BOOT 键再按 RESET 进入

首次上电进入 WiFi 配网模式（设备会播报提示音），也可按遥控器 `#` 键
重新配网。

## 注意事项

1. **供电**：MAX98357 建议 5V 供电以获得 3W 输出；USB 5V 直接供电时
   大音量可能出现电压跌落，建议使用质量好的 USB 电源（≥2A）。
2. **回声**：DIY 结构中麦克风靠近喇叭，播放声可能被录入（无硬件 AEC），
   适当拉开距离或在 `config.json` 中尝试 `CONFIG_USE_DEVICE_AEC=y`。
3. **GPIO8/9** 为 C6 的 strapping 引脚：GPIO8 接了 WS2812 不影响启动；
   GPIO9 为 BOOT 键，属常规用法。
4. 避免把外设接到 GPIO12/13（原生 USB）、GPIO16/17（板载 CH343 串口）、
   GPIO24~30（模组内部 Flash）。
