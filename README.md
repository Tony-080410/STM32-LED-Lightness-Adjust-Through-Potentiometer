# 8点滑动平均滤波算法 - 电位器调节 LED 亮度

基于 **STM32F103C8T6** 的嵌入式示例工程：旋转电位器改变分压电压，MCU 通过 **ADC1 + DMA** 采集该电压，经滑动平均滤波后线性映射为 **TIM3_CH1 的 PWM 占空比**，从而实现 LED 亮度的无级调节。

- 芯片型号：STM32F103C8T6（LQFP48，Cortex-M3，64KB Flash / 20KB SRAM）
- 工程类型：STM32CubeMX 生成 + CMake / Ninja + `arm-none-eabi-gcc`
- HAL 库版本：STM32Cube FW_F1 V1.8.7
- 生成工具版本：STM32CubeMX 6.18.1

---

## 1. 项目功能

| 项目 | 说明                                                             |
| ---- | ---------------------------------------------------------------- |
| 输入 | 电位器（可变电阻）分压产生的模拟电压，范围 0 ~ 3.3V              |
| 采集 | ADC1_IN0（PA0）12 位采样，DMA 循环搬运，无需 CPU 干预            |
| 处理 | 8 点滑动平均滤波，抑制电位器机械抖动导致的亮度跳变               |
| 输出 | TIM3_CH1（PA6）输出 1kHz PWM，占空比 0% ~ 99.9% 线性对应旋钮位置 |
| 效果 | 顺时针旋转旋钮 → LED 由暗到亮，逆时针则变暗                      |

**数据流向：**

```
电位器分压 (0~3.3V)
        │
        ▼
   PA0 / ADC1_IN0  ──(连续转换, 12 位)──►  DMA1_Channel1 (循环模式)
        │
        ▼
   s_adc_raw (0~4095)  ──►  8 点滑动平均  ──►  adc_avg
        │
        ▼
   ccr = adc_avg × (ARR+1) / 4096   ──►  __HAL_TIM_SET_COMPARE()
        │
        ▼
   PA6 / TIM3_CH1  ──(1kHz PWM, 占空比 0~99.9%)──►  LED 亮度
```

---

## 2. 引脚配置

### 2.1 引脚分配表

| 引脚         | 复用功能       | 模式配置                                                         | 用途说明                                   |
| ------------ | -------------- | ---------------------------------------------------------------- | ------------------------------------------ |
| **PA0-WKUP** | ADC1_IN0       | `GPIO_MODE_ANALOG`（模拟输入）                                   | 电位器滑动端（中间抽头）接入，采集分压电压 |
| **PA6**      | TIM3_CH1       | `GPIO_MODE_AF_PP` + `GPIO_SPEED_FREQ_HIGH`（复用推挽输出，高速） | PWM 输出，驱动 LED                         |
| PD0-OSC_IN   | RCC_OSC_IN     | HSE 外部晶振                                                     | 8MHz 无源晶振输入端                        |
| PD1-OSC_OUT  | RCC_OSC_OUT    | HSE 外部晶振                                                     | 8MHz 无源晶振输出端                        |
| PA13         | SYS_JTMS-SWDIO | Serial Wire                                                      | SWD 调试数据线                             |
| PA14         | SYS_JTCK-SWCLK | Serial Wire                                                      | SWD 调试时钟线                             |

> GPIO 时钟使能：`GPIOA`、`GPIOD`（见 `Core/Src/gpio.c`）。
> 注：`PA0` 与 `PA6` 的引脚模式分别在 `HAL_ADC_MspInit()`（adc.c）与 `HAL_TIM_MspPostInit()`（tim.c）中完成，不在 `MX_GPIO_Init()` 内。

### 2.2 典型接线

```
        +3.3V ──┬───────────────┐
                │               │
             [电位器 两端]       │
                │               │
        GND ────┴──────► 滑动端 ─┴──► PA0 (ADC1_IN0)

        PA6 (TIM3_CH1) ──► LED 限流电阻（或三极管/MOS 驱动）──► GND
```

- 电位器两端分别接 3.3V 与 GND，滑动端接 PA0；推荐在滑动端并接 0.1µF 电容做硬件滤波。
- LED 侧：PA6 输出高电平为 3.3V，直接驱动小功率 LED 时须串接限流电阻；驱动大功率负载应加三极管/MOS。
- 注意 ADC 参考电压即 VDDA（3.3V），采集结果与供电电压相关。

---

## 3. 时钟与外设配置

### 3.1 系统时钟树（`SystemClock_Config()`，main.c）

| 节点       | 配置                               | 结果频率                                 |
| ---------- | ---------------------------------- | ---------------------------------------- |
| 时钟源     | HSE 外部晶振（8MHz），HSI 同时开启 | 8 MHz                                    |
| PLL        | 源 = HSE，`RCC_PLL_MUL9`（×9）     | 72 MHz                                   |
| SYSCLK     | PLL 输出                           | 72 MHz                                   |
| AHB 分频   | `RCC_SYSCLK_DIV1`                  | HCLK = 72 MHz                            |
| APB1 分频  | `RCC_HCLK_DIV2`                    | PCLK1 = 36 MHz（定时器时钟 ×2 = 72 MHz） |
| APB2 分频  | `RCC_HCLK_DIV1`                    | PCLK2 = 72 MHz                           |
| Flash 等待 | `FLASH_LATENCY_2`                  | 2 个等待周期                             |
| ADC 时钟   | `RCC_ADCPCLK2_DIV6`（PCLK2 / 6）   | **12 MHz**                               |

> ADC 时钟 12MHz 未超过 STM32F1 的 14MHz 上限，采样精度有保证。
> TIM3 挂载在 APB1，但因 APB1 分频系数为 2，定时器时钟自动 ×2，仍为 **72MHz**。

### 3.2 ADC1 配置（`MX_ADC1_Init()`，adc.c）

| 参数       | 取值                        | 说明                               |
| ---------- | --------------------------- | ---------------------------------- |
| Instance   | `ADC1`                      | 使用 ADC1                          |
| 扫描模式   | `ADC_SCAN_DISABLE`          | 单通道，不扫描                     |
| 连续转换   | `ENABLE`                    | 自由运行，转换完成后自动开启下一次 |
| 间断模式   | `DISABLE`                   | —                                  |
| 外部触发   | `ADC_SOFTWARE_START`        | 软件启动，启动后持续转换           |
| 数据对齐   | `ADC_DATAALIGN_RIGHT`       | 右对齐，结果 0 ~ 4095              |
| 转换通道数 | 1                           | 仅规则通道 1 个                    |
| 采集通道   | `ADC_CHANNEL_0`（Rank 1）   | 对应 PA0                           |
| 采样时间   | `ADC_SAMPLETIME_55CYCLES_5` | 55.5 个 ADC 周期                   |

**单次转换耗时估算：** (55.5 + 12.5) / 12MHz ≈ **5.7 µs**，即约 176 kSPS 的连续采样速率。

### 3.3 DMA 配置（adc.c 中的 `HAL_ADC_MspInit()` + `MX_DMA_Init()`）

| 参数         | 取值                                            | 说明                             |
| ------------ | ----------------------------------------------- | -------------------------------- |
| 通道         | `DMA1_Channel1`                                 | 与 ADC1 请求绑定                 |
| 方向         | `DMA_PERIPH_TO_MEMORY`                          | 外设 → 内存                      |
| 外设地址自增 | `DMA_PINC_DISABLE`                              | ADC 数据寄存器地址固定           |
| 内存地址自增 | `DMA_MINC_ENABLE`                               | 配合多通道扩展                   |
| 数据宽度     | 半字（16 位）                                   | ADC 数据寄存器为 16 位           |
| 模式         | `DMA_CIRCULAR`                                  | 循环模式，永不停止               |
| 优先级       | `DMA_PRIORITY_HIGH`                             | 高优先级                         |
| NVIC         | `DMA1_Channel1_IRQn`，抢占优先级 0 / 子优先级 0 | 传输完成中断，本轮不使用回调逻辑 |

### 3.4 TIM3 配置（`MX_TIM3_Init()`，tim.c）

| 参数             | 取值                             | 说明                                           |
| ---------------- | -------------------------------- | ---------------------------------------------- |
| Instance         | `TIM3`                           | 通用定时器                                     |
| Prescaler（PSC） | 71                               | 72MHz / (71+1) = **1MHz** 计数频率（1µs/tick） |
| Period（ARR）    | 999                              | 计数 0 ~ 999，共 1000 个 tick                  |
| 计数模式         | `TIM_COUNTERMODE_UP`             | 向上计数                                       |
| 时钟分频         | `TIM_CLOCKDIVISION_DIV1`         | —                                              |
| 自动重装载预装载 | `TIM_AUTORELOAD_PRELOAD_DISABLE` | —                                              |
| 通道             | `TIM_CHANNEL_1` → PA6            | PWM Generation CH1                             |
| PWM 模式         | `TIM_OCMODE_PWM1`                | 计数值 < CCR 时输出有效电平                    |
| 初始 Pulse       | 0                                | 上电初始占空比 0                               |
| 输出极性         | `TIM_OCPOLARITY_HIGH`            | 高电平有效                                     |
| 快速模式         | `TIM_OCFAST_DISABLE`             | 关闭                                           |

**PWM 频率与分辨率：**

```
f_PWM = 72MHz / ((71+1) × (999+1)) = 1 kHz
占空比分辨率 = 1 / 1000 = 0.1 %
CCR 取值范围 = 0 ~ 999，对应占空比 0 % ~ 99.9 %
```

---

## 4. 代码逻辑

### 4.1 文件结构

| 文件                           | 职责                                                                     |
| ------------------------------ | ------------------------------------------------------------------------ |
| `Core/Src/main.c`              | 系统时钟配置、外设初始化调度、**应用主循环**（滤波 + 映射 + 更新占空比） |
| `Core/Src/gpio.c`              | GPIO 端口时钟使能                                                        |
| `Core/Src/adc.c`               | ADC1 初始化、ADC 引脚（PA0）配置、**ADC-DMA 绑定**                       |
| `Core/Src/dma.c`               | DMA1 控制器时钟使能、DMA1_Channel1 中断使能                              |
| `Core/Src/tim.c`               | TIM3 PWM 初始化、PA6 复用功能配置                                        |
| `Core/Src/stm32f1xx_it.c`      | 中断服务程序（SysTick、DMA1_Channel1）                                   |
| `Core/Src/stm32f1xx_hal_msp.c` | HAL 底层初始化（MSP）                                                    |
| `Core/Inc/*.h`                 | 各模块句柄与函数声明                                                     |
| `startup_stm32f103xb.s`        | 启动文件（中断向量表、复位入口）                                         |
| `STM32F103xx_FLASH.ld`         | 链接脚本（Flash 64KB / RAM 20KB 内存布局）                               |
| `ElectronicControl_test.ioc`   | STM32CubeMX 工程配置（可据此重新生成代码）                               |

### 4.2 关键函数一览

| 函数                         | 所在文件                  | 作用                                                                                                                  |
| ---------------------------- | ------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `main()`                     | main.c                    | 初始化外设 → 启动 PWM 与 ADC-DMA → 进入无限循环执行采集-映射-输出                                                     |
| `SystemClock_Config()`       | main.c                    | 配置 HSE + PLL×9 → 72MHz 系统时钟、总线分频、Flash 等待、ADC 时钟分频                                                 |
| `MX_GPIO_Init()`             | gpio.c                    | 使能 GPIOA / GPIOD 时钟                                                                                               |
| `MX_DMA_Init()`              | dma.c                     | 使能 DMA1 时钟，配置并使能 `DMA1_Channel1_IRQn`                                                                       |
| `MX_ADC1_Init()`             | adc.c                     | 配置 ADC1 连续转换模式与 IN0 通道，内部触发 `HAL_ADC_MspInit()` 完成 PA0 模拟模式、ADC 时钟、DMA 通道初始化           |
| `MX_TIM3_Init()`             | tim.c                     | 配置 TIM3 时基与 CH1 PWM 输出，内部触发 `HAL_TIM_PWM_MspInit()` / `HAL_TIM_MspPostInit()` 完成时钟使能与 PA6 复用配置 |
| `HAL_TIM_PWM_Start()`        | HAL 库                    | 启动 TIM3_CH1 PWM 输出                                                                                                |
| `HAL_ADC_Start_DMA()`        | HAL 库                    | 启动 ADC1 连续转换并由 DMA 循环搬运到 `s_adc_raw`                                                                     |
| `__HAL_TIM_SET_COMPARE()`    | HAL 宏                    | 运行时更新 CCR 寄存器，改变占空比                                                                                     |
| `DMA1_Channel1_IRQHandler()` | stm32f1xx_it.c            | DMA 传输完成中断入口，转交 `HAL_DMA_IRQHandler()`                                                                     |
| `Error_Handler()`            | main.c                    | 致命错误处理：关中断后死循环，便于调试定位                                                                            |
| `HAL_IncTick()`              | HAL 库（SysTick_Handler） | 维护 HAL 毫秒时基                                                                                                     |

### 4.3 初始化流程（`main()` 前半段）

```c
HAL_Init();                    // HAL 库初始化 + SysTick
SystemClock_Config();          // 72MHz 系统时钟
MX_GPIO_Init();                // GPIO 时钟
MX_DMA_Init();                 // DMA 时钟 + DMA1_Channel1 中断
MX_ADC1_Init();                // ADC1 + PA0 + ADC-DMA 绑定
MX_TIM3_Init();                // TIM3 PWM + PA6

__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);   // 上电先熄灭 LED，避免满亮
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);           // 启动 PWM，失败则 Error_Handler()

s_pwm_period = htim3.Init.Period + 1U;              // 运行期读取 ARR+1 = 1000

HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&s_adc_raw, 1U);  // 启动 ADC 连续转换 + DMA
```

> 初始化顺序有隐含依赖：`MX_DMA_Init()` 必须在 `MX_ADC1_Init()` **之前**调用，否则 ADC 的 DMA 中断未使能；此顺序由 CubeMX 的 `functionlistsort` 保证。

### 4.4 主循环结构（核心逻辑）

主循环是一个**无阻塞的轮询式闭环**，每个循环完成「取样 → 滤波 → 映射 → 输出」四步：

```c
while (1) {
    /* 1. 取最新采样值 0~4095（由 DMA 在后台持续刷新） */
    uint32_t adc_value = s_adc_raw;

    /* 2. 8 点滑动平均滤波 */
    s_adc_sum -= s_adc_filter[s_adc_index];      // 减去即将被覆盖的旧样本
    s_adc_filter[s_adc_index] = (uint16_t)adc_value;
    s_adc_sum += adc_value;                      // 加上新样本（增量式维护和）
    s_adc_index++;
    if (s_adc_index >= MOVING_AVG_LEN) {         // 环形缓冲回绕
        s_adc_index = 0U;
    }
    uint32_t adc_avg = s_adc_sum / MOVING_AVG_LEN;

    /* 3. 线性映射：ADC 0~4095 → CCR 0~1000 */
    uint32_t ccr = (uint32_t)(((uint64_t)adc_avg * (uint64_t)s_pwm_period) / ADC_FULL_SCALE);

    /* 4. 更新比较寄存器，亮度随旋钮位置线性变化 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
}
```

#### 各步骤详解

**步骤 1 — 采样读取**
ADC1 处于连续转换模式，DMA1_Channel1 以循环模式在后台把每次转换结果写入 `s_adc_raw`，CPU 只需一次读取即可拿到最新值，无需等待转换、无需阻塞。采样速率约 176 kSPS，远高于主循环速率，因此每次读到的都是「最新」值。

**步骤 2 — 滑动平均滤波（8 点）**
采用**增量式求和的环形缓冲**：每次只做一次减法（剔除最旧样本）+ 一次加法（写入新样本），避免对 8 个元素重新求和，运算量为 O(1)。

- 常量定义：`MOVING_AVG_LEN = 8`；缓冲 `s_adc_filter[8]`、下标 `s_adc_index`、累加和 `s_adc_sum` 均为 `static`，跨循环保持状态。
- 上电后前 8 次循环缓冲内为零值，均值会从低位逐步收敛到真实值；因循环极快（微秒级），肉眼不可见。
- 滤波的目的：电位器碳膜磨损与机械振动会产生 ±若干 LSB 的抖动，直接映射会造成 LED 亮度闪烁；8 点平均可显著平滑。

**步骤 3 — 线性映射**
`ccr = adc_avg × (ARR+1) / 4096`，即把 12 位 ADC 满量程 4096 映射到 1000 个计数级：

- 乘法使用 `uint64_t` 中间变量，虽然 4095 × 1000 = 4,095,000 尚未超出 32 位，但保留 64 位运算可避免以后调大 ARR 时溢出，意图也更明确。
- `s_pwm_period` 在运行时从 `htim3.Init.Period + 1U` 读取，而非硬编码，便于以后调整 ARR 而无需改动映射逻辑。
- 边界情况：`adc_avg = 4095`（旋钮到底）时 `ccr = 999`，占空比 99.9%，LED 接近满亮；`adc_avg = 0` 时 `ccr = 0`，LED 完全熄灭。

**步骤 4 — 输出更新**
`__HAL_TIM_SET_COMPARE()` 直接写 TIM3 的 CCR1 寄存器，硬件立即在下个 PWM 周期按新占空比输出，无需重启 PWM、无软件延时，因此亮度变化连续无跳变。

### 4.5 中断与并发

| 中断          | 优先级（抢占/子） | 处理内容                                                                   |
| ------------- | ----------------- | -------------------------------------------------------------------------- |
| SysTick       | 15 / 0            | `HAL_IncTick()`，维护 HAL 时基                                             |
| DMA1_Channel1 | 0 / 0             | `HAL_DMA_IRQHandler(&hdma_adc1)`；本工程未注册回调，传输完成事件被静默处理 |

主循环与 DMA 之间存在「DMA 写 `s_adc_raw` / CPU 读 `s_adc_raw`」的数据竞争。由于该变量为半字（16 位）对齐访问，在 Cortex-M3 上单次 16 位读写是原子的，不会读到撕裂数据；最坏情况只是偶尔取到上一轮样本，对亮度显示无影响。

---

## 5. 编译与烧录

### 5.1 环境依赖

| 组件            | 版本 / 说明                                                                      |
| --------------- | -------------------------------------------------------------------------------- |
| CMake           | ≥ 3.22                                                                           |
| Ninja           | 构建后端（`CMakePresets.json` 中 `"generator": "Ninja"`）                        |
| GCC 工具链      | `arm-none-eabi-gcc`（供 `cmake/gcc-arm-none-eabi.cmake` 使用）                   |
| STM32Cube FW_F1 | V1.8.7，默认路径 `C:/Users/Username/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7` |
| STM32CubeMX     | 6.18.1（仅在需要重新生成代码时使用）                                             |
| 下载调试        | ST-Link + SWD（PA13/PA14）                                                       |
| 编辑器          | VS Code（已含 `.vscode/` 配置、`.clangd` 索引配置）                              |

> 若 CubeMX 固件包路径与上述不一致，需同步修改 `cmake/stm32cubemx/CMakeLists.txt` 中的 `MX_Include_Dirs` 与 `STM32_Drivers_Src` 路径。

### 5.2 编译步骤

```powershell
# 1) 配置（生成 Ninja 构建文件到 build\Debug）
cmake --preset Debug

# 2) 编译
cmake --build build/Debug

# 发布构建（开启优化）：
# cmake --preset Release
# cmake --build build/Release
```

**产物：** `build\Debug\ElectronicControl_test.elf`（同时生成 `.map` 内存映射文件）

> 提示：默认构建类型为 `Debug`（`-O0`/含调试信息）；需要体积与性能优化时使用 `Release` 预设。

### 5.3 参数调整速查

| 想改什么          | 改哪里                                                                                                          |
| ----------------- | --------------------------------------------------------------------------------------------------------------- |
| PWM 频率          | `Core/Src/tim.c` → `htim3.Init.Prescaler` / `htim3.Init.Period`（保持乘积为 72000 即 1kHz，乘积 720 为 100kHz） |
| 亮度曲线上下限    | `Core/Src/main.c` → 映射公式中的 `ADC_FULL_SCALE`，或对 `ccr` 做饱和限制                                        |
| 滤波平滑程度      | `Core/Src/main.c` → `MOVING_AVG_LEN`（越长越平滑但响应越慢，须同步调整 `s_adc_filter` 数组长度）                |
| ADC 采样时间      | `Core/Src/adc.c` → `sConfig.SamplingTime`（信号源阻抗较高时应适当加长）                                         |
| 更换输入/输出引脚 | `ElectronicControl_test.ioc` 中用 CubeMX 重新分配，再重新生成代码                                               |

---

## 6. 已知注意事项与可改进点

1. **`s_adc_raw` 未声明 `volatile`**：该变量会被 DMA（硬件）修改，严格来说应声明为 `volatile uint16_t`。当前代码在 Cortex-M3 上的对齐半字访问是原子操作，且循环中每次都会重新读取，实际功能正常；但加上 `volatile` 语义更严谨，也避免以后开启高等级优化时被编译器优化掉。
2. **ADC 满量程与 CCR 上限不齐**：4095 映射后为 999 而非 1000，占空比最高 99.9%，实际使用无差别；若需要严格 100%，可在映射后做 `if (ccr > s_pwm_period) ccr = s_pwm_period;` 的钳位。
3. **无软件过流/限幅保护**：`ccr` 由 ADC 直接决定，若电位器接触不良导致 ADC 读数跳变，亮度会随之跳变，可通过加大滤波窗口或加入变化率限制（斜坡）改善。
4. **DMA 中断被使能但未使用**：`MX_DMA_Init()` 使能了 `DMA1_Channel1_IRQn`，中断服务程序仅为转发给 HAL，当前无业务逻辑；若不需要可关闭以节省极少量开销。
5. **`Error_Handler()` 关中断死循环**：`HAL_TIM_PWM_Start()` 或 `HAL_ADC_Start_DMA()` 失败时会静默卡死，调试时可在其中加入 LED 快闪或串口打印以便定位。
