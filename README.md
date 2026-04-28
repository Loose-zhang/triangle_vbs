# RTKLIB Demo5 B34I VBS

本项目基于 RTKLIB 源码的 `include/` 和 `src/`，并新增 `ntrip_rtcm_obs/` 示例程序，用于接收、解析、打印 RTCM3 观测数据，并在双基站输入时生成虚拟中点 RTCM 输出。

## 目录结构

- `include/`：RTKLIB 头文件，主要数据结构和 API 声明在 `rtklib.h`。
- `src/`：RTKLIB 核心源码，包括 RTCM 解码、编码、流处理、定位和星历处理等。
- `ntrip_rtcm_obs/`：RTCM3 TCP 接收和虚拟 RTCM 生成程序。
- `build/`：CMake 构建输出目录。

## 构建

项目使用 CMake 构建：

```powershell
cmake --build build
```

如果需要重新生成构建目录：

```powershell
cmake -S . -B build
cmake --build build
```

构建成功后，可执行文件位于：

```text
build/ntrip_rtcm_obs/ntrip_rtcm_obs.exe
```

## 运行方式

### TCP 服务端模式

程序监听本地 TCP 端口，等待 STRSVR、NTRIP 转发程序或其他 RTCM 源连接进来。

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe [listen_port] [label]
```

示例：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe 50001 BASE_A
```

如果不指定端口，默认监听 `50001`。

### TCP 客户端模式

程序主动连接一个或多个 RTCM TCP 源：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe -c <host> <port> [label] [-c <host> <port> [label] ...]
```

单路示例：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe -c 127.0.0.1 51000 BASE_A
```

双路示例：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe -c 127.0.0.1 51000 BASE_A -c 127.0.0.1 51001 BASE_B
```

双路模式下，程序会：

- 分别解析两个 RTCM3 输入流。
- 从 RTCM 1005/1006 提取两个基站 ARP 坐标。
- 计算两个 ARP 的 ECEF 中点。
- 对两个同步历元的同星同码观测生成虚拟中点观测。
- 在本机 TCP `52000` 端口输出虚拟 RTCM3：`1005 + MSM7`。

## 输出日志说明

示例日志：

```text
[2435540] RTCM 1005 - station / antenna (ARP in 1005/1006)
  RTCM 1005 (  22): staid=1660 pos=41.66934392 109.23752989 1681.729
  staid=1660  sta.name=1660  ITRF year=0
  ARP ECEF  X=-1572550.5718  Y=4506243.9988  Z=4219358.4100 (m)
  ARP LLH   lat=41.66934392 deg  lon=109.23752989 deg  h_ellip=1681.7290 m
Virtual RTCM -> port 52000: TX 508 B; parse check obs_epoch=1 station=1 err=0
```

含义：

- `RTCM 1005`：收到或生成了基站坐标消息。
- `staid`：RTCM 站号。RTCM 站号只有 12 bit，不同基站出现相同站号是可能的，应结合输入流标签和 ARP 坐标区分。
- `ARP ECEF`：天线参考点的 ECEF 坐标。
- `ARP LLH`：天线参考点的大地坐标。
- `TX 508 B`：本次向 TCP `52000` 端口发送的虚拟 RTCM 总字节数。
- `parse check obs_epoch=1`：程序将刚生成的 RTCM 回读解析，得到 1 个完整观测历元。
- `station=1`：回读解析得到 1 条站点消息，通常是 RTCM 1005。
- `err=0`：回读解析没有 CRC 或帧错误。

注意：`obs_epoch=1` 不表示只发送了一条 MSM。RTCM MSM 多消息组通常只有最后一条 `sync=0` 时才返回完整观测历元，所以多条 MSM 合并后仍可能显示 `obs_epoch=1`。

## MSM 生成规则

RTCM MSM 的 cell mask 最多只能表达 64 个 cell。对单条 MSM 消息必须满足：

```text
nsat * nsig <= 64
```

其中：

- `nsat`：该消息内卫星数。
- `nsig`：该消息内信号数。

如果直接调用 `gen_rtcm3()` 生成 MSM，调用方必须自行拆分观测数据，保证每条 MSM 不超过 64 cell。

本项目已做两处修复：

- `src/rtcm3e.c`：当 `nsat * nsig > 64` 时拒绝生成该 MSM，避免静默生成错误包。
- `ntrip_rtcm_obs.c`：虚拟 RTCM 输出前按星座和 `64 / nsig` 自动拆分 MSM7。
- 虚拟历元**保留对齐后的各频点**（如双基站均为 L1+L2，则输出双频）；不再为凑满单条 MSM 颗星数而把每颗星压成单频。双频时每条 MSM 的 `nsat` 上限约为 `64 / nsig`（例如两频时约 32 颗星/包），由拆分逻辑自动处理。

## 虚拟中点观测算法

双基站模式中，程序不使用星历计算卫星位置，而是使用两个基站到同一卫星的伪距近似构造中点伪距：

```text
P_mid = sqrt((P1^2 + P2^2) / 2 - b^2 / 4)
```

其中：

- `P1`、`P2`：两个基站对同一卫星同一信号的伪距。
- `b`：两个基站 ARP 的 ECEF 距离。
- `P_mid`：虚拟中点伪距。

载波与伪距**共用同一三角形几何**，但载波观测值 `L` **不再转换成米制相路径长度**，而是直接保持周单位参与计算。为保证公式量纲一致，只把两站基线 `b` 按当前信号波长 `λ` 换成周：

```text
b_cyc = b / λ
L_mid = sqrt(max(0, (L1^2 + L2^2) / 2 - b_cyc^2 / 4))
```

任一站对该码无效载波时 **`L_mid = 0`**。若几何上导致根号内为负，该频 **`L`** 亦为 0。

该方法对每个观测码独立处理：`L1`、`L2` 是两个基站对应信号的载波相位周数，输出 `L_mid` 仍为周。适合演示与短基线近似；电离层等与频率有关的残差仍存在，高精度应用需谨慎。

## 关键源码位置

- `src/rtcm.c`：`gen_rtcm3()` 封包、CRC、长度检查。
- `src/rtcm3.c`：RTCM3 解码和 MSM 信号表。
- `src/rtcm3e.c`：RTCM3 编码，包含 MSM 编码实现。
- `src/streamsvr.c`：RTKLIB 流转换路径中的 MSM 拆包逻辑。
- `ntrip_rtcm_obs/ntrip_rtcm_obs.c`：TCP RTCM 接收、双流处理中点生成、虚拟 RTCM 输出。

## 常见问题

### 为什么日志里 `err=0` 但观测数量看起来不对？

`err=0` 只说明 RTCM 帧和 CRC 正确，不代表每个星座、每个信号都生成成功。需要检查 MSM 是否超出 `nsat * nsig <= 64`，以及输入观测码是否存在于对应星座的 MSM signal table 中。

### 为什么两个基站的 `staid` 一样？

RTCM station id 只有 12 bit，冲突是正常情况。双流模式应使用命令行 label 和 RTCM 1005/1006 的 ARP 坐标区分基站。

### 为什么 `obs_epoch=1`？

RTKLIB 解码 MSM 多消息组时，通常在最后一条 `sync=0` 的 MSM 到达后才返回一个完整观测历元。因此即使发送了多条 MSM，回读检查也可能显示 `obs_epoch=1`。

### 虚拟观测是双频还是单频？

若两路基站对同一卫星、同一观测码均有有效伪距，则该码会参与中点合成并写入输出；**各频点相互独立**（例如 L1、L2 各一条 `P_mid` / `L`）。输出不再强制每星只留一条信号；多频时 MSM 会按 `nsat * nsig <= 64` 拆成更多条消息。

注意：两站均有载波时直接对周数 `L` 使用 Apollonius 式，基线项使用 `b / λ` 换成周；任一无效或根号结果为负则为 0；码重映射时仍不会用伪距给 `L` 占位。
