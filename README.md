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

程序主动连接一个或多个 RTCM TCP 源。两类标志：

- `-c <host> <port> [label]`：**基站流**（含 obs + 1005/1006 ARP，如有 1019/1042/1044/1045/1046/1020 也会进入星历池）。
- `-e <host> <port> [label]`：**独立广播星历流**（只贡献 ephemeris，obs/ARP 一律忽略）。可指向 NTRIP 上的 `BCEP00BKG0`、`SSRA00BKG0`、`RTCM3EPH` 等纯星历挂载点。

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe (-c|-e) <host> <port> [label] [(-c|-e) <host> <port> [label] ...]
```

单路示例：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe -c 127.0.0.1 51000 BASE_A
```

双基站示例：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe -c 127.0.0.1 51000 BASE_A -c 127.0.0.1 51001 BASE_B
```

双基站 + 独立广播星历流（推荐）：

```powershell
.\build\ntrip_rtcm_obs\ntrip_rtcm_obs.exe `
    -c 127.0.0.1 51000 BASE_A `
    -c 127.0.0.1 51001 BASE_B `
    -e 127.0.0.1 51002 EPH
```

双基站模式下，程序会：

- 分别解析两个 RTCM3 输入流。
- 从 RTCM 1005/1006 提取两个基站 ARP 坐标。
- 计算两个 ARP 的 ECEF 中点。
- **若全局星历池就绪**：用广播星历做单点定位级几何运算，**估出每台基站接收机钟差**并扣除，再合成清洁的 VRS 伪距（详见后文"虚拟中点观测算法"）。
- **若星历未就绪**：自动回退到旧版 Apollonius 中线公式（仅作冷启动 backup）。
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

### 当前默认：星历驱动的"扣钟差 + 清洁合成"伪距（builder B）

只要全局星历池里有 ≥4 颗星的有效广播星历，每个历元都走以下流程，由 `build_virt_obs_with_eph()` 实现：

1. 调用 RTKLIB 的 `satposs(EPHOPT_BRDC)` 计算每颗卫星的 ECEF 位置 `r_s` 与卫星钟差 `dt_s`，已自动迭代信号传输时间。
2. 用 `geodist` 算出卫星到基站 A、B、虚拟点 V 的几何距离 `ρ_A、ρ_B、ρ_V`，已含 Sagnac 改正。
3. 用 Klobuchar (`ionmodel` + `ion_gps`/`ion_cmp`) 估电离层延迟 `I_A、I_B、I_V`，用 Saastamoinen (`tropmodel`) 估对流层延迟 `T_A、T_B、T_V`。卫星高度角 < 5° 直接剔除。
4. **逐基站接收机钟差估计**：对该历元各卫星，取**首个有效伪距**计算
   ```text
   resid_i^k = P_i^k − ρ_i^k + c·dt_s^k − I_i^k − T_i^k
             ≈ c·δt_rcv_i + 噪声 + 多径
   ```
   则 `c·δt_rcv_i ≈ median_k(resid_i^k)`。中值法对离群多径鲁棒。
5. **VRS 接收机钟差**取两基站平均：`c·δt_rcv_V = (c·δt_rcv_A + c·δt_rcv_B) / 2`。
6. **逐信号合成 VRS 伪距**：对每颗共视卫星的每个观测码（B1I/L1C/B7I…），分别计算两基站扣钟差后的"残差"（多径 + 接收机噪声 + 大气模型残差）：
   ```text
   ε_A = P_A − ρ_A + c·dt_s − I_A − T_A − c·δt_rcv_A
   ε_B = P_B − ρ_B + c·dt_s − I_B − T_B − c·δt_rcv_B
   ```
   再用 **SNR 线性功率加权**合成（`w_i = 10^(SNR_i / 10)`，是该码 ML 最优权）：
   ```text
   P_V = ρ_V − c·dt_s + I_V + T_V + c·δt_rcv_V + (w_A·ε_A + w_B·ε_B) / (w_A + w_B)
   ```
   等价表达：把"两基站含钟差伪距"先减各自钟差归零、几何用真值替换、最后再加上 VRS 钟差和 SNR-加权后的残差。`|ε_A − ε_B| > 30 m` 的信号直接剔除（明显多径/失锁）。当某一基站不报 SNR（=0）时退化为 0.5 等权。

输出的 VRS 伪距具有：
- **常数式钟差跨星一致**：所有卫星共享同一个 `δt_rcv_V`，rover 端单差 (SD) 即可消钟。
- **几何用星历计算**：与基线长度无关；千米级网络扩展无需修改公式。
- **噪声 √2 改善**：两基站独立多径平均后噪声方差减半。
- **大气模型自洽**：VRS 位置用 V 的 azel 计算自身的 I_V、T_V，短基线下 ≈ I_A ≈ I_B 自动成立，长基线则获得真实的网络插值效果。

每历元打印 QC 日志（用于排错）：
```text
VRS [eph] :52000 TX=824B nv=29 | clk_A=+1234.567m(28sv) clk_B=+1234.890m(28sv) dclk=-0.323m | rms((epsA-epsB)/2)=0.412m | obs=1 sta=0 err=0
```
- `clk_A/clk_B`：两基站接收机钟差 (m)，括号内是参与中值的卫星数。
- `dclk = clk_A − clk_B`（数学符号 Δclk）：两台接收机互差，反映两机钟同步水平。理想 < 1 m。
- `rms((epsA−epsB)/2)`（数学符号 rms((εA−εB)/2)）：两站残差差的 RMS，反映**两机多径 + 大气未建模差异**的等效噪声水平。短基线应在 0.3–0.5 m 量级。
- `obs/sta/err`：本地回读自检计数。

> 日志里所有标识符都用 ASCII 写死（`dclk` 而不是 `Δclk`，`epsA/epsB` 而不是 `εA/εB`），以兼容 Windows GBK 控制台和老旧 SSH 终端，避免出现 `蔚` 之类的乱码。源码注释和 README 正文保留数学符号，方便阅读。

### Fallback：Apollonius 中线公式（builder A，仅冷启动）

星历池尚未填充（程序刚启动头几十秒）时，自动退回到旧版几何近似：

```text
P_mid = sqrt(max(0, (P1² + P2²) / 2 − b² / 4))
b_cyc = b / λ
L_mid = sqrt(max(0, (L1² + L2²) / 2 − b_cyc²/ 4))
```

该退化算法把伪距/载波当作几何距离处理，**对短基线下伪距数值上接近算术平均**，**但承担两台接收机钟差混合**；载波因整周模糊度被破坏，不可用于 RTK 整数固定。仅作冷启动应急。

### 载波相位：A‑B 双差固定 + 加权平均

伪距估出 `clk_A`、`clk_B` 之后，对每个 `(sat, code)` 都用相同的几何缓存做载波合成。核心思路：

1. **每个星座、每个观测码各选一颗参考星 j\*** —— 选两基站均有有效 `L`、A 站高度角最高的那颗。参考星只在它失锁/落到地平线下时才换，最大化稳定性。
2. **DD 浮点模糊度**（cycles）：
   ```text
   DD_float^k = (L_A^k − L_B^k − L_A^j* + L_B^j*) − (Δρ_AB^k − Δρ_AB^j*) / λ
   ```
   其中 `Δρ_AB^k = ρ_A^k − ρ_B^k`，全部用星历精算。注意 `(clk_A − clk_B) / λ` 在 SD‑SD 中**自动消失**，所以 DD 浮点解的噪声仅剩载波本身的 mm 级（≈ 0.005 cyc），**round‑to‑nearest 极其鲁棒**——不需要 LAMBDA 搜索。
3. **取整 + 阈值校验**：`DD_n = round(DD_float)`，残差 `|DD_float − DD_n| < 0.20 cyc` 才算固定（高门槛防误固）；缓存命中则用 0.40 cyc 的"宽容门"复用，避免抖动重固定。
4. **滑周检测**：任一基站对该 `(sat, code)` 的 `LLI` 第 0 位置 1 → 缓存失效，下个历元重新固定。
5. **参考星切换**（`pick_ref_sat` 检测原参考星失锁时）：用桥接公式
   ```text
   DD_n_new^k = DD_n_old^k − DD_n_old^{j_new}
   ```
   对该星座该码下所有已固定卫星一次性重制项，无需重新搜索。
6. **VRS 载波合成**（cycles，对每颗已固定的卫星）：
   ```text
   L_B_aligned^k = L_B^k + DD_n^k                    // 把 B 的载波平移到 A 的整数参考帧
   L_A_at_V^k    = L_A^k + (ρ_V − ρ_A) / λ           // 几何平移到 V
   L_B_at_V^k    = L_B_aligned^k + (ρ_V − ρ_B) / λ
   L_V^k         = 0.5 × (L_A_at_V^k + L_B_at_V^k)   // 严格等权 → 噪声 √2 改善
   ```
   关键 1：**不减 `(clk_A − clk_B) / λ`**。原因：从伪距估出的 `clk_A − clk_B` 噪声 ≈ 0.3 m，对 B1I (`λ ≈ 0.192 m`) 折成 ≈ 1.5 cyc，远大于载波本身的 mm 级噪声，强行扣会污染结果。让两台基站钟差自然落到平均钟里反而最干净。
   
   关键 2：**载波严格等权（不像伪距那样按 SNR 加权）**。如果换成 SNR 加权 `(w_A·L_A_at_V + w_B·L_B_at_V) / (w_A + w_B)`，下一节"半整数偏移"中的 `0.5 × (N_B^j* − N_A^j*)` 会变成 `w_B/(w_A+w_B) × (N_B^j* − N_A^j*)`，这个权重比例**逐信号不同**，rover 端 DD 就不再是整数了。所以必须等权。SNR 极差的信号已被前置的 25 dBHz 门限剔除，等权合成的噪声损失（最坏 √2 倍）远小于"破坏整数 DD"的代价。

### 关于半整数模糊度偏移

经过上述合成，`L_V^k` 的有效模糊度是
```text
N_V^k = N_A^k + 0.5 × (N_B^j* − N_A^j*)
```
当 `N_B^j* − N_A^j*` 为奇数时，`N_V` 是半整数。这听上去吓人，但**对所有卫星、所有历元（参考星不变期间）都是同一常数**，所以 rover 端做 DD：
```text
∇ΔN_RV = (N_R^k − N_R^j_r) − (N_V^k − N_V^j_r)
       = (N_R^k − N_R^j_r) − (N_A^k − N_A^j_r)         // 半整数项相减消失
```
**仍是严格整数**，LAMBDA 搜索/固定一切照常。

参考星切换时 `j*` 改变，`N_V^k` 共模跳一个 `0.5 × Δ(N_B^j* − N_A^j*)`，rover 端表现为一次"伪钟跳"，被周跳/钟跳处理逻辑吸收掉。日志里 `refSwap` 计数器记录此事件频率（理想下短时间内应 = 0）。

### 未固定卫星的处理

当前实现采用"**全部固定/全部丢弃**"策略：DD 不能固定的 `(sat, code)`（含 cycle slip 后未来得及重固定、ref 不可用、或 B 该信号无 `L`）的载波直接 `L = 0`，**伪距仍正常输出**。这样所有有效 `L_V` 都遵循同一种模糊度约定，避免下游 rover 整数 DD 被"半整数 vs 整数"混合污染。代价是早期几个历元 `L` 输出可能偏少；通常 1–2 个历元后即全部固定。

### 卫星集合：A∪B（并集模式）

合成不再要求 `(sat, code)` 同时存在于 A∩B。每历元按"是否在每个基站可见"分三种处理：

| 卫星出现位置 | 伪距输出 | 载波输出 | QC 计数 |
|---|---|---|---|
| **A 和 B 均见**（branch a） | SNR 加权 ML 合成（原逻辑） | DD 固定后等权合成（原逻辑） | `fix` / `float` / `drop` |
| **只有 A 见**（branch b） | 单基几何搬移：`P_V = P_A + (ρ_V−ρ_A) + (I_V−I_A) + (T_V−T_A)` | **不输出**（避免与 DUAL 路径不同的模糊度约定混入下游） | `Aonly` |
| **只有 B 见**（branch c） | 单基几何搬移（A↔B 对称公式） | **不输出**（同上） | `Bonly` |

为什么单基支路上只输出 P 而不输出 L：DUAL 模式下 `L_V` 的有效模糊度是 `N_A^k + 0.5×(N_B^j*−N_A^j*)`，所有星共享同一半整数偏移 → rover DD 是整数；如果再混入"单 A 搬移"的 `L_V_单 = N_A^k`（无半整数项），rover 端某些 DD 就会变成半整数 → 固定失败。所以**单基支路彻底放弃载波输出**。

实际场景里这一改动主要带来"A 临时跟丢一颗星 30 秒、B 仍稳定"的 epoch 中，VRS **仍能输出该星的伪距**（之前会整颗星掉），rover 至少保持代码差分能力。

### 单基站模式（A_ONLY / B_ONLY 自动切换）

当某一台基站**整体死亡**（接收机离线、断网、硬件故障）时，状态机会自动从 DUAL 切换到 SINGLE_X 模式，把仍活着的那台基站经星历几何搬移到中点 V。rover 视角看到的"基站"位置不变，仍在中点；只是噪声从 √2 改善退化到单基水平。

#### 状态机

四种状态：
- `DUAL`：两台基站都健康（每台 P 中值估钟时 ≥ `MIN_CLOCK_SATS = 4` 颗有效卫星）。
- `A_ONLY`：B 已经连续 ≥ `MODE_FAIL_TO_SINGLE = 3` 个历元钟差估计失败 → 切换到只用 A。
- `B_ONLY`：A 已连续 ≥ 3 历元失败 → 切换到只用 B。
- `NONE`：两台都不可用 → 不输出。

恢复方向反向且更慢：连续 ≥ `MODE_RECOVER_TO_DUAL = 5` 个历元复原才回 DUAL。这种"快速降级、慢速恢复"的不对称是为了避免 RTCM 偶发丢包导致状态机来回抖动，每次抖动都会触发一次 rover 全网重固定。

#### 单基模式的合成公式

无 DD 模糊度固定，纯几何搬移：

```text
P_V = P_X + (ρ_V − ρ_X) + (I_V − I_X) + (T_V − T_X)
L_V = L_X + (ρ_V − ρ_X) / λ
```

其中 X = A 或 B，是仍活着的那台基站。`I_V − I_X` 在短基线下 < cm 量级，`λ` 项差异更小（载波的电离层符号相反，差异 < mm），故载波公式忽略大气项。

模糊度约定 `N_V^k = N_X^k`，所有星一致 → rover DD 仍是整数（虽然与 DUAL 模式的约定不同 → 切换瞬间会有一次一次性跳变）。

#### 模式切换的 LLI 注入

每次 DUAL ↔ SINGLE_X 切换的**首历元**，`build_virt_obs_with_eph` 会把所有输出星的 `LLI[j] |= 1`（cycle-slip 标志）。下游 rover 收到 LLI=1 后会把该星的整周模糊度状态重置，开始新一轮 LAMBDA 搜索。这是因为 DUAL 与 SINGLE 的模糊度约定差一个 `0.5×(N_B^j*−N_A^j*)`，如果不强制告诉 rover，它会拿旧约定的整数模糊度去解新约定的载波 → 严重错误。

5 秒短时 B 失联（< 3 历元）：状态机不切，DUAL 路径自然走 union 模式，丢失的星自动落到 branch (b)，影响只是 `Aonly` 计数和该星暂时无 L。

5 秒长时 B 失联（≥ 3 历元）：DUAL → A_ONLY，rover 重固定一次（30 秒 RTK 解锁）。B 恢复 5 秒后切回 DUAL，再重固定一次。**总代价：单次中断引发 rover 两次重固定**，约 1 分钟 RTK 中断。这比"DUAL 死掉就完全没输出"好很多。

要消除一次中断造成两次 rover 重固定，最直接的办法是把 `MODE_RECOVER_TO_DUAL` 调大（比如 60，对应 1 分钟），让 SINGLE 模式停留更长，避免 B 短暂上线就立刻切回。如果业务上 B 恢复不重要，甚至可以把 `MODE_RECOVER_TO_DUAL` 设成 `INT_MAX` 让 SINGLE 锁死直到程序重启。

### SNR 处理（统一适用于伪距与载波）

当两个基站的天线类型/朝向/遮挡不一致时，同一颗卫星在两站的 SNR 经常差 5–15 dBHz。代码处理分三层：

1. **门限过滤（gate）**：`MIN_SNR_THRES_X1000 = 25000`（即 25 dBHz）。任一基站对该 `(sat, code)` 报告的 SNR 低于此门限时，该信号同时退出**伪距和载波**输出。25 dBHz 是接收机数据手册里普遍的"低于此值多径主导，相位/伪距精度急剧劣化"边界。被丢弃的信号计入日志的 `lowSNR`。
   - 当一台基站根本不报 SNR（=0）时门限不生效（permissive），保证不报 SNR 的旧基站流仍可正常通过。
   - **参考星选择 `pick_ref_sat` 也强制走门限**：如果一颗高度角最高的星在哪一基站 SNR 不达标，宁可换次高的，也不让它做参考——因为参考星噪声会污染**所有非参考星**的 DD 残差。
2. **伪距：SNR 线性功率加权**。权重 `w_i = 10^(SNR_i / 10)`（SNR 单位 dBHz）。当两边 SNR 相同则等价 0.5/0.5；差 10 dB 时高 SNR 一边权重 ≈ 91%，差 15 dB 时 ≈ 97%。这是 σ ∝ 1/√(C/N0) 假设下的最大似然加权。
3. **载波：严格等权 0.5/0.5**。前面已解释为什么不能 SNR 加权（会破坏跨星半整数偏移的恒定性，导致 rover DD 非整数）。靠门限提前剔掉低 SNR 信号即可；剩下信号载波噪声都是 mm 级，等权与 ML 加权的噪声差远小于 1 mm。
4. **输出 SNR 走 dB 域功率加和**：`SNR_V = 10·log10(10^(SNR_A/10) + 10^(SNR_B/10))`，物理上对应"两路独立信号相干合成"。结果 cap 在 65.5 dBHz 以保证写回 `obsd_t.SNR` (uint16, 0.001 dBHz) 不溢出。**不再用算术平均**——dB 域算术平均没有物理意义，且对相差较大的两个 SNR 会偏向给"假"读数（如 45 dBHz 与 25 dBHz 平均成 35 dBHz，明显低估了实际可用信噪比）。

调阈值的话改 `ntrip_rtcm_obs.c` 顶部的 `MIN_SNR_THRES_X1000`：`25000` = 25 dBHz；放宽到 `20000` 会多保留信号但接受更多多径污染；收紧到 `30000` 适合高质量天线对（如双 NetR9）但可能在城区遮挡下输出星数变少。

### 每历元 QC 日志

```text
VRS [eph DUAL] :52000 TX=824B nv=29 | clk_A=+1234.567m(28sv) clk_B=+1234.890m(28sv) dclk=-0.323m | rms((epsA-epsB)/2)=0.412m | L: fix=42 float=3 drop=0 refSwap=0 lowSNR=2 | Aonly=0 Bonly=0 | obs=1 sta=0 err=0
```
- `[eph DUAL/A-only/B-only]`：当前合成模式。`DUAL` 是双基站 ML 合成（最佳精度），`A-only` / `B-only` 是单基站搬移模式（B/A 失联或 SNR 长期不达标后自动切入）。
- `L: fix`：本历元成功 DD 固定的 `(sat, code)` 数量（注意：每星每频独立计 1，所以双频 BeiDou 一颗星可能贡献 2–3 个）。SINGLE 模式下记的是单基载波直出数量。
- `L: float`：`DD_float` 残差超过 0.20 cyc 阈值、未固定的数量。理想下 = 0；持续 > 0 说明可能有：(a) 一台基站时间标签偏差、(b) 大气模型残差过大（长基线时）、(c) 两机硬件偏差不一致。
- `L: drop`：因参考星不可用、或某一基站缺该信号 `L` 而被主动丢的载波。
- `refSwap`：本历元发生的参考星切换次数。
- `lowSNR`：因任一基站 SNR < 25 dBHz 而**整个信号**（P 和 L 同时）被丢的次数。如果 `lowSNR` 占总信号数比例超过 10%，建议检查两台基站的天线/线缆/前端 LNA 一致性。
- `Aonly` / `Bonly`：DUAL 模式下，因卫星只在 A（或 B）侧可见而走"单基支路 P-only"的 (sat, code) 数。两者合计 / `nv` 可估算"基站间卫星集失配率"；持续 > 5% 通常表示一台基站的接收机或天线方向图与另一台不匹配。

新算法启动后 `fix` 应迅速接近 `float + fix + drop` 的总数，且 `float` 越小越好；如果 `fix` 始终为 0，请检查广播星历是否进入了池子（`g_nav_n_total()` 在 fallback 路径里有打印）。

模式切换示例（B 站从断网恢复）：
```text
VRS [eph DUAL]   ... | clk_A=+...m(28sv) clk_B=+...m(28sv) ...   ← 正常
...3 epochs of B with 0sv → switch...
VRS [eph A-only] ... | clk_A=+...m(28sv) clk_B=+0.000m(0sv) ...  ← 切到单 A，所有星打 LLI=1
VRS [eph A-only] ... | clk_A=+...m(28sv) clk_B=+0.000m(0sv) ...
...B 流恢复，但要等 5 个良好历元才回 DUAL...
VRS [eph A-only] ... | clk_A=+...m(28sv) clk_B=+...m(20sv) ...   ← B 在线但还在 hold
...5 epochs later...
VRS [eph DUAL]   ... | clk_A=+...m(28sv) clk_B=+...m(28sv) ...   ← 切回，再次 LLI=1
```
两次切换之间 rover 各重固定一次（约 30 s × 2）。

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

### 怎么知道当前用的是新算法还是 Apollonius 退化？

看每历元的 `VRS [eph DUAL/A-only/B-only]` 还是 `VRS [apollonius fallback]`。后者通常只在程序刚启动、还没收到任何 1019/1042/1044/1045/1046 时短暂出现；看到 `eph_n=0` 就意味着完全没有星历，可考虑加 `-e` 单独接一路星历挂载点。日志里 `[eph A-only]` 或 `[eph B-only]` 表示对方基站已经被状态机判定为"挂掉"并切到单基模式，需要排查那台基站的网络/接收机。

### 双基站 obs 流里本来就有星历，还需要 `-e` 吗？

不一定。基站 RTCM 里如果按一定周期发了 1019/1042/1044/1045/1046/1020，全局星历池会自动建起来，无需 `-e`。但很多基站只挂 MSM7 不发 nav，或 nav 周期非常稀疏（如 30 s 一次），冷启动慢；这时显式挂一路 `-e` 指向纯星历广播挂载点最快。也可以把同一台基站再挂一路 `-e`（同 host 同 port），让 obs 与 eph 并行入池。

### 验证 VRS 算法是否正确，应当怎么对比？

不要直接比绝对伪距值（`P_VRS − P_真实中点`），那个差里包含两边接收机的**钟差差** 和**多径差**，可达 m 级且无法消除。正确方法：

1. **跨星单差 SD**：对 VRS 各卫星 `SD_k = P_VRS^k − P_真实^k`。理想下所有 `SD_k` 应近似相等（= 钟差），随时间线性漂移。
2. **跨星双差 DD**：选一参考卫星 j*，`DD = SD_k − SD_{j*}`。伪距 DD 应在 ±0.3 m 量级（多径 + 大气模型残差）；载波 DD 应是整数 ± mm（前提是上线了 phase upgrade）。
3. **看每历元打印的 `Δclk` 与 `rms`**：算法本身的健康度直接体现在这两个数上。

### `Δclk` 总是几十米甚至上百米，正常吗？

正常。它代表两台接收机的接收机钟差差，主要由各自 GNSS 时间同步精度决定。常见在几十纳秒（≈ 10 m）到几微秒（≈ 1000 m）之间。新算法显式扣除该差，所以不会污染 VRS 输出 — `rms((εA-εB)/2)` 不大就 OK。

### 虚拟观测是双频还是单频？

若两路基站对同一卫星、同一观测码均有有效伪距，则该码会参与中点合成并写入输出；**各频点相互独立**（例如 L1、L2 各一条 `P_mid` / `L`）。输出不再强制每星只留一条信号；多频时 MSM 会按 `nsat * nsig <= 64` 拆成更多条消息。

注意：两站均有载波时直接对周数 `L` 使用 Apollonius 式，基线项使用 `b / λ` 换成周；任一无效或根号结果为负则为 0；码重映射时仍不会用伪距给 `L` 占位。
