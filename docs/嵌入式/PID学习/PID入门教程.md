# 江科大PID入门教程

PID理论部分这里不做介绍，详细请看教程[江科大PID入门教程](https://www.bilibili.com/video/BV1G9zdYQEr3?p=2)

- **比例（P）**： 与当前误差成正比，快速响应误差。
- **积分（I）**： 与过去一段时间误差的累积成正比，用于消除静差（稳态误差）。
- **微分（D）**： 与当前误差的变化率成正比，用于预测误差趋势，抑制超调。

## 一、PID模板程序实现

以下示例代码皆使用标准库实现

确定一个调控周期T，每隔时间T，程序执行一次PID调控![image-20250910151437445](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250910151437445.png)	

- PID实现可以通过以上三种形式，一般情况下不使用第一种，delay会使整个程序停止运行；
- 第二种也是最常用的，使用定时器来设置调控周期。但要注意资源访问冲突的问题，要防止在main跟中断函数中同时对同一个资源进行访问，因为main跟中断函数是多线程执行，同时对一个资源进行访问会出问题。
- 第三种可以防止资源访问冲突的问题，但也有弊端。当主程序出问题时，PID调控会被中断，这会使调控周期T不准确，因此在使用第三种方法时要确保主程序能顺利执行

### 位置式PID程序实现

计算公式如下：

![image-20250910152640610](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250910152640610.png)

![image-20250910152659522](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250910152659522.png)

### 增量式PID程序实现(控制器内积分,输出全量)

公式如下：

![image-20250910153454762](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250910153454762.png)

![image-20250910153506472](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250910153506472.png)

### 位置式PID与增量式PID的选择

| 特性                 | 位置式PID                                                    | 增量式PID                                                    |
| :------------------- | :----------------------------------------------------------- | :----------------------------------------------------------- |
| **输出性质**         | **绝对量**，直接是执行机构的目标位置（如阀门开度0%-100%）    | **增量/变化量**，是相对于上次输出的调整值（如阀门再开大5%）  |
| **积分作用**         | **显式积分**，需要对所有历史误差进行累加（∑e(k)∑*e*(*k*)）   | **隐式积分**，通过u(k)=u(k−1)+Δu(k)*u*(*k*)=*u*(*k*−1)+Δ*u*(*k*)实现积分效果，无累加和 |
| **计算量**           | 较大（每次都要计算累加和）                                   | 较小（只用到最近三次误差值）                                 |
| **积分饱和问题**     | **非常严重**。当系统存在较大偏差或长时间饱和（如阀门已全开但误差仍很大），积分项会累积到一个非常大的值，导致系统退出饱和需要很长时间，引起大幅超调振荡。 | **本质上不易积分饱和**。因为输出是增量，一旦控制量达到饱和（如阀门全开），计算出的增量无法执行，但不会累积巨大的积分项。一旦误差反向，控制能立即反向作用。 |
| **对执行机构的要求** | 要求执行机构具有**位置记忆功能**（如伺服电机、带位置反馈的阀门）。因为输出是绝对位置，执行机构必须能精确地到达该位置。 | 要求执行机构具有**增量控制功能**（如步进电机、通过脉冲控制的阀门）。控制器告诉它“走多少步”，而不是“走到哪里”。 |
| **无扰动切换**       | **较难实现**。若手动切换到自动，需要将初始输出u(0)*u*(0)设置为当前手动时的阀位，且积分项要初始化为一个合适值，否则会产生冲击。 | **极易实现**。手动模式下，增量输出Δu(k)Δ*u*(*k*)自然为0。切换到自动时，无需特殊处理，系统平滑过渡，因为输出是增量，不会对系统产生剧烈冲击。 |
| **编程实现**         | 需要保存误差累加和。                                         | 只需保存最近两次的误差值e(k−1),e(k−2)*e*(*k*−1),*e*(*k*−2)即可。 |
| **抗干扰性**         | 较差。如果计算出的u(k)*u*(*k*)因干扰产生误差，会直接影响被控对象。 | 较好。一次计算的错误只影响本次增量，错误不会持续累积到下一次。 |

| 选择依据     | 优先选用位置式PID                | 优先选用增量式PID                                            |
| :----------- | :------------------------------- | :----------------------------------------------------------- |
| **执行机构** | 伺服电机等需要绝对位置指令的机构 | 步进电机、变频器等接受增量指令的机构                         |
| **核心关切** | 概念直观，易于理解               | **防止积分饱和**、**无扰动切换**、**安全性**                 |
| **系统资源** | 计算和存储资源相对充足           | 单片机等资源受限的嵌入式系统                                 |
| **主流应用** | 伺服位置控制等                   | **过程控制（温度、压力、流量等）、大多数计算机数字控制系统** |

简单来说，可以这样记：**增量式PID用得更广泛，尤其是在对安全性和可靠性要求高的工业控制中。而位置式PID则在一些特定的、要求绝对位置输出的场合更为直观。** 在现代控制实践中，即使采用增量式算法，其输出也常常会通过累加转换为绝对量来驱动执行机构，从而兼顾两种算法的优点。

## 二、基础驱动代码编写

### 2.1 OLED模块

<img src="./PID%E7%AE%97%E6%B3%95.assets/image-20250925161150261.png" alt="image-20250925161150261" style="zoom:50%;" /><img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925161156549.png" alt="image-20250925161156549" style="zoom:50%;" />

通过原理图看得出OLED的`SCL`与`SDA`分别接到`PB8`与`PB9`，驱动代码直接移植即可，在CubeMX中设置`PB8/9`为``GPIO_Output`。并设置为开漏输出`Output Open Drain`。

### 2.2 LED模块

该项目没有外接LED，使用`STM32F103C8T6`内置的LED模块，接在`PC13`上。设置`PC13`为`GPIO_Output`，并设置`GPIO output level=High`、`GPIO mode=Output Push Pull`
驱动代码如下：

```c
#include "gpio.h"
#include "stm32f1xx_hal.h"

void LED_Init(void) {
    MX_GPIO_Init();
}

void LED_ON(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void LED_OFF(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void LED_Turn(void) {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

```

### 2.3 KEY模块

![image-20250925184911046](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925184911046.png)

![image-20250925184954792](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925184954792.png)

根据图示设置`PB10、PB11、PA11、PA12`引脚为`GPIO_Input`且为上拉输入

![image-20250925192300248](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925192300248.png)

该项目使用定时器实现非阻塞式按键消抖，使用TIM1定时器，并开启`TIM1 update interrupt`TIM1更新中断

![image-20250925184030919](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925184030919.png)

`Key.c`

```c
#include "Key.h"
#include "Delay.h"
#include "gpio.h"

uint8_t Key_Num;

void Key_Init(void) {
    MX_GPIO_Init();
}

uint8_t Key_GetNum(void) {
    uint8_t Temp;
    if (Key_Num) {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }
    return 0;
}

uint8_t Key_GetState(void) {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 0) {  // KEY1
        return 1;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 0) {  // KEY2
        return 2;
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) {  // KEY3
        return 3;
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0) {  // KEY4
        return 4;
    }
    return 0;
}

void Key_Tick(void) {
    static uint8_t Count;
    static uint8_t CurrState, PrevState;

    Count++;
    if (Count >= 20) {
        Count = 0;

        PrevState = CurrState;
        CurrState = Key_GetState();

        if (CurrState == 0 && PrevState != 0) {
            Key_Num = PrevState;
        }
    }
}

```

### 2.4 电位器RP

![image-20250925192812160](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925192812160.png)![image-20250925192839394](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925192839394.png)

![image-20250925193324735](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925193324735.png)

电位器接PA2,3,4,5，根据STM32F103C8T6引脚复位功能图看到这四个电位器旋钮对应的ADC通道为ADC1/2的通道2~5

配置PA2,3,4,5为ADC2_IN2,3,4,5。并设置时钟分频6分频

![image-20250925195713117](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925195713117.png)![image-20250925195722225](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925195722225.png)

**为何选择6分频**：

**ADC最大时钟频率限制**

- STM32F1系列的ADC最大时钟频率通常为14MHz
- 超过此频率会导致转换结果不准确
- 如果PCLK2为72MHz，6分频后ADC时钟为12MHz（72MHz/6 = 12MHz）
- 12MHz低于14MHz的最大限制，保证了ADC的正常工作

`RP.c`驱动代码

```c
#include "RP.h"
#include "adc.h"
void RP_Init(void) {
    MX_ADC2_Init();
}

/**
  * @brief  获取指定电位器的ADC转换值
  * @param  n 电位器编号，有效值为1-4，分别对应ADC通道2-5
  * @retval 电位器ADC转换结果，16位无符号整数
  * @note   该函数通过ADC2获取指定电位器的模拟量转换值
  */
uint16_t RP_GetValue(uint8_t n) {
    uint16_t value;  // 用于存储转换值
    // 初始化ADC2句柄
    ADC_HandleTypeDef hadc2;
    hadc2.Instance = ADC2;

    // 配置ADC通道参数
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    // 根据输入参数n选择对应的ADC通道
    if (n == 1) {
        sConfig.Channel = ADC_CHANNEL_2;
    } else if (n == 2) {
        sConfig.Channel = ADC_CHANNEL_3;
    } else if (n == 3) {
        sConfig.Channel = ADC_CHANNEL_4;
    } else if (n == 4) {
        sConfig.Channel = ADC_CHANNEL_5;
    }

    // 配置ADC通道，如配置失败则调用错误处理函数
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // 启动ADC软件转换
    HAL_ADC_Start(&hadc2);

    // 等待转换完成
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);

    // 获取转换值
    value = HAL_ADC_GetValue(&hadc2);

    // 停止ADC转换
    HAL_ADC_Stop(&hadc2);

    return value;
}

```

### 2.5 电机

下图是两种不同规格的编码电机，同一时间最多只能接两个电机`M+`与`M-`是直流电机的两个引脚

![image-20250925200818209](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925200818209.png)

下图为TB6612电机驱动模块`M1+`与`M1-`接的是`AO1/2`，A路对应控制引脚为PWMA的`AIN1/2`。

![image-20250925201007886](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925201007886.png)

`AIN1/2`分别接在`PB12/13`，由`PB12/13`来控制电机方向即可

<img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925201400211.png" alt="image-20250925201400211" style="zoom:67%;" />

![image-20250925201246212](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925201246212.png)

PA0默认复用TIM2_CH1

`pwm.c`

```c
#include "pwm.h"

void PWM_Init(void) {
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
}

void pwm_set_compare(uint16_t compare) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}

```

`motor.c`

```c
#include "motor.h"
#include "gpio.h"

void Motor_Init(void) {
    MX_GPIO_Init();
}

void Motor_SetPWM(int16_t PWM) {
    if (PWM > 0) {
        HAL_GPIO_WritePin(GPIOB, MOTOR_PIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, MOTOR_PIN2, GPIO_PIN_SET);
        pwm_set_compare(PWM);
    } else {
        HAL_GPIO_WritePin(GPIOB, MOTOR_PIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, MOTOR_PIN2, GPIO_PIN_RESET);
        pwm_set_compare(-PWM);
    }
}

```

### 2.6 电机编码器

编码器(Encoder)用来测量电机旋转的实际速度和位置

![image-20250925204307664](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925204307664.png)

`EA/EB`是电机的两个编码引脚

<img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925204348310.png" alt="image-20250925204348310" style="zoom:67%;" />

![image-20250925204448969](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925204448969.png)

![image-20250925213015298](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250925213015298.png)

设置一个通道上升沿一个通道下降沿是为了顺时针转时，speed是正值，逆时针转speed是负值。也可以两个都设置为上升沿，然后在`Encoder_Get`中返回`-temp`

`encoder.c`

```c
#include "encoder.h"

void Encoder_Init(void) {
    HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
}


/**
  * @brief  获取编码器计数值并清零
  * @retval 编码器计数值增量
  */
int16_t Encoder_Get(void) {
    int16_t temp;
    temp = __HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return temp;
}

```

定时器中断回调函数：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t count = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();
        count++;
        if (count >= 50) {
            count = 0;
            speed = Encoder_Get();
            location += speed;
        }
    }
}
```

count用于分频，目前是每speed个边沿/50ms。测量周期越长，测速精度越高，速度刷新就慢。测量周期越短，测速精度越低，速度刷新就快。在PID控速中，编码器的读取周期和PID调控周期，保持一致是最好的

**PID控制周期**（也称为采样周期或调节周期）是指PID控制器执行一次完整控制运算的时间间隔。

在PID控制系统中，控制器会周期性地读取被控对象的当前状态（如通过传感器），计算当前误差（目标值与实际值之差），然后根据PID算法（比例-积分-微分）计算出控制输出，并将该输出应用到被控对象上。这个过程重复进行，而两次执行之间的时间间隔就是PID控制周期。

### 2.7 串口

![image-20250926094148332](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926094148332.png)

<img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926094203593.png" alt="image-20250926094203593" style="zoom:67%;" />

![image-20250926094245463](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926094245463.png)

配置CubeMX
![image-20250926094358982](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926094358982.png)

`serial.c`想要实现不定长的串口收发需要用到DMA

```c
#include "serial.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// 定义接收缓冲区大小
#define SERIAL_RX_BUFFER_SIZE 5
// 定义合理的超时时间
#define SERIAL_TIMEOUT 1000

extern UART_HandleTypeDef huart1;

uint8_t re_data[SERIAL_RX_BUFFER_SIZE];
volatile uint8_t uart_tx_busy = 0;  // 添加发送忙标志

void Serial_Init(void) {
    uart_tx_busy = 0;  // 初始化发送状态
    HAL_UART_Receive_IT(&huart1, re_data, SERIAL_RX_BUFFER_SIZE);
}

/**
  * @brief  串口发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void Serial_SendByte(uint8_t Byte) {
    HAL_UART_Transmit(&huart1, &Byte, 1, SERIAL_TIMEOUT);  //使用HAL库发送一个字节
}

/**
  * @brief  串口发送一个数组
  * @param  Array 要发送数组的首地址
  * @param  Length 要发送数组的长度
  * @retval 无
  */
void Serial_SendArray(uint8_t* Array, uint16_t Length) {
    HAL_UART_Transmit(&huart1, Array, Length, SERIAL_TIMEOUT);  //直接发送整个数组
}

/**
  * @brief  串口发送一个字符串
  * @param  String 要发送字符串的首地址
  * @retval 无
  */
void Serial_SendString(char* String) {
    HAL_UART_Transmit(&huart1, (uint8_t*)String, strlen(String), SERIAL_TIMEOUT);  //直接发送整个字符串
}

/**
  * @brief  次方函数（内部使用）
  * @param  X 底数
  * @param  Y 指数
  * @retval 返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y) {
    uint32_t Result = 1;  //设置结果初值为1
    while (Y--)           //执行Y次
    {
        Result *= X;  //将X累乘到结果
    }
    return Result;
}

/**
  * @brief  串口发送数字
  * @param  Number 要发送的数字，范围：0~4294967295
  * @param  Length 要发送数字的长度，范围：0~10
  * @retval 无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length) {
    uint8_t i;
    // 优化：避免重复计算幂，使用累除法
    uint32_t divisor = 1;
    for (i = 0; i < Length - 1; i++) {
        divisor *= 10;
    }

    for (i = 0; i < Length; i++) {
        Serial_SendByte(Number / divisor % 10 + '0');
        divisor /= 10;
    }
}

/**
  * @brief  格式化打印函数
  * @param  format 格式化字符串
  * @param  ... 可变参数
  * @retval 无
  */
void Serial_Printf(char* format, ...) {
    static char str[100];  // 使用静态变量避免栈溢出
    va_list ap;
    va_start(ap, format);
    vsnprintf(str, sizeof(str), format, ap);  //使用安全版本防止缓冲区溢出
    va_end(ap);

    // 等待上一次发送完成，避免冲突，但增加超时机制
    uint32_t timeout = 1000; // 设置超时时间
    while (uart_tx_busy && timeout--) {
        HAL_Delay(1);
    }

    uart_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, strlen(str)) != HAL_OK) {
        uart_tx_busy = 0; // 如果发送失败，清除忙标志
    }
}

/**
  * @brief  UART接收完成回调函数
  * @param  huart UART句柄指针
  * @retval 无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    // 重新启动接收中断
    HAL_UART_Receive_IT(&huart1, re_data, SERIAL_RX_BUFFER_SIZE);
}

/**
  * @brief  UART发送完成回调函数
  * @param  huart UART句柄指针
  * @retval 无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    uart_tx_busy = 0;  // 发送完成，清除忙标志
}

```

`serial_dma.c`

```c
#include "serial_dma.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// 定义接收缓冲区大小
#define SERIAL_DMA_RX_BUFFER_SIZE 256

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

// DMA串口接收相关变量
uint8_t dma_rx_buffer[SERIAL_DMA_RX_BUFFER_SIZE];
uint8_t dma_tx_buffer[SERIAL_DMA_RX_BUFFER_SIZE];
volatile uint16_t dma_rx_head = 0;
volatile uint16_t dma_rx_tail = 0;
volatile uint8_t dma_uart_tx_busy = 0;

/**
  * @brief  DMA串口初始化
  * @retval 无
  */
void Serial_DMA_Init(void) {
    dma_uart_tx_busy = 0;
    dma_rx_head = 0;
    dma_rx_tail = 0;

    // 启动空闲中断和DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buffer, SERIAL_DMA_RX_BUFFER_SIZE);
}

/**
  * @brief  DMA串口发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void Serial_DMA_SendByte(uint8_t Byte) {
    HAL_UART_Transmit(&huart1, &Byte, 1, 1000);
}

/**
  * @brief  DMA串口发送一个数组
  * @param  Array 要发送数组的首地址
  * @param  Length 要发送数组的长度
  * @retval 无
  */
void Serial_DMA_SendArray(uint8_t* Array, uint16_t Length) {
    HAL_UART_Transmit(&huart1, Array, Length, 1000);
}

/**
  * @brief  DMA串口发送一个字符串
  * @param  String 要发送字符串的首地址
  * @retval 无
  */
void Serial_DMA_SendString(char* String) {
    HAL_UART_Transmit(&huart1, (uint8_t*)String, strlen(String), 1000);
}

/**
  * @brief  DMA串口发送数字
  * @param  Number 要发送的数字，范围：0~4294967295
  * @param  Length 要发送数字的长度，范围：0~10
  * @retval 无
  */
void Serial_DMA_SendNumber(uint32_t Number, uint8_t Length) {
    uint8_t i;
    // 优化：避免重复计算幂，使用累除法
    uint32_t divisor = 1;
    for (i = 0; i < Length - 1; i++) {
        divisor *= 10;
    }

    for (i = 0; i < Length; i++) {
        Serial_DMA_SendByte(Number / divisor % 10 + '0');
        divisor /= 10;
    }
}

/**
  * @brief  DMA格式化打印函数
  * @param  format 格式化字符串
  * @param  ... 可变参数
  * @retval 无
  */
void Serial_DMA_Printf(char* format, ...) {
    static char str[256];  // 使用静态变量避免栈溢出
    va_list ap;
    va_start(ap, format);
    vsnprintf(str, sizeof(str), format, ap);
    va_end(ap);

    // 等待上一次发送完成
    while (dma_uart_tx_busy) {
        // 空等待
    }

    dma_uart_tx_busy = 1;
    memcpy(dma_tx_buffer, str, strlen(str));
    if (HAL_UART_Transmit_DMA(&huart1, dma_tx_buffer, strlen(str)) != HAL_OK) {
        dma_uart_tx_busy = 0;
    }
}

/**
  * @brief  DMA串口获取接收数据长度
  * @retval 接收数据长度
  */
uint16_t Serial_DMA_GetRxLength(void) {
    return (dma_rx_head >= dma_rx_tail) ? (dma_rx_head - dma_rx_tail) : (SERIAL_DMA_RX_BUFFER_SIZE - dma_rx_tail + dma_rx_head);
}

/**
  * @brief  UART空闲中断回调函数
  * @param  huart UART句柄指针
  * @retval 无
  */
void HAL_UART_RxIdleCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        // 计算接收到的数据长度
        uint16_t rx_data_len = SERIAL_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        // 更新接收数据头指针
        dma_rx_head = rx_data_len;

        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buffer, SERIAL_DMA_RX_BUFFER_SIZE);
    }
}

/**
  * @brief  UART发送完成回调函数
  * @param  huart UART句柄指针
  * @retval 无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        dma_uart_tx_busy = 0;
    }
}

/**
  * @brief  UART接收事件回调函数
  * @param  huart UART句柄指针
  * @param  Size  接收到的数据大小
  * @retval 无
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart1) {
        // HAL_UART_Transmit_DMA(&huart1, dma_rx_buffer, Size);

        // 更新接收缓冲区的头指针位置
        dma_rx_head = (dma_rx_head + Size) % SERIAL_DMA_RX_BUFFER_SIZE;

        // uint16_t length = Serial_DMA_GetRxLength();
        // Serial_DMA_Printf("Rx Length: %d\r\n", length);

        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buffer, SERIAL_DMA_RX_BUFFER_SIZE);
    }
}

```

### 2.8 角度传感器

<img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20251001203924555.png" alt="image-20251001203924555" style="zoom:50%;" />

<img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20251001204049139.png" alt="image-20251001204049139" style="zoom:50%;" />

![image-20251001204132089](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20251001204132089.png)

引脚接J1则使用PB0

![image-20251001205552954](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20251001205552954.png)

驱动代码

```c
#include "AD.h"
#include "adc.h"

extern ADC_HandleTypeDef hadc1;

void AD_Init(void) {
    MX_ADC1_Init();
}

uint16_t AD_GetValue(void) {
    HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

```



## 三、PID基础实验-PID闭环控制实验

### 3.1 位置式PID定速控制

**目标**：按键控制目标速度值增减，然后PID调控，使得编码器测量得到的实际速度和设置的目标速度保持一致，且即使出现负载变化，实际速度，也都能很好的维持在目标速度附近。

设定调控周期T，每隔时间T，程序执行一次PID调控（此处设置T与编码器测量周期一致）

定义全局变量：

- target为目标速度，由用户指定
- actual为实际速度，由编码器测得
- out表示控制力度，作用于设置电机PWM的函数

```c
float target, actual, out;
float kp = 0.2, ki = 0.2, kd = 0;
float error0, error1, errorInt;
```

为了调控周期T与编码器测量周期一致，将PID调控代码写在编码器测量周期一起

```c
/* 1ms一次 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t count = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();
        count++;
        if (count >= 40) {
            count = 0;

            /* ===PID调控=== */
            /* 获取实际值 */
            actual = Encoder_Get();

            /* 获取本次误差和上次误差 */
            error1 = error0;
            error0 = target - actual;

            /* 误差积分（累加） */
            errorInt += error0;

            /* PID计算 */
            out = kp * error0 + ki * errorInt + kd * (error0 - error1);

            /* 输出限幅 */
            if (out > 100) out = 100;
            if (out < -100) out = -100;

            /* 执行控制 */
            Motor_SetPWM(out);
        }
    }
}
```

out传入的是占空比，| 0~100 |

---

可能出现的现象：

当出现起步电机就满速运行，可能是极性出问题。即`Encoder_Get()`输出的值与传感器输入函数`Motor_SetPWM()`的极性不同（相反）。解决办法可以将`Encoder_Get()`返回的值取负值即可或`Motor_SetPWM()`传入的时取负。

#### 调参

先确定Kp，再确定Ki，最后确定Kd

Kp参数的数量级确定：
此处将目标target增大，直到out=100，继续增大target，actual不变，则actual的值为输出的范围。
输入值范围为out：-100~100。则参数数量级为：输入/输出，得到0.几，这是数量级。具体数值还需进一步调参

通过电位器调整参数

```c
int main(void) {
    ...
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* 通过电位器修改PID参数 */
        /* 电位器取值范围0-4095 */
        /* RP_GetValue(1) / 4095.0归一化到0-1的范围再乘以2，就是将范围扩大到0-2 */
        kp = RP_GetValue(1) / 4095.0 * 2;
        ki = RP_GetValue(2) / 4095.0 * 2;
        kd = RP_GetValue(3) / 4095.0 * 2;

        /* 获取目标值，经测试得到act最大为90，且target不能过于极限*/
        /* 140 将范围扩大到0-140 */
        /* - 70 将范围偏移到-70到+70 */
        target = RP_GetValue(4) / 4095.0 * 140 - 70;

        OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", kp);
        OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", ki);
        OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", kd);

        OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", target);
        OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", actual);
        OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", out);
        OLED_Update();

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

```

调整PID调控代码
```c
/* 误差积分（累加） */
// errorInt += error0;
if (ki != 0) {
    errorInt += error0;
} else {
    errorInt = 0;
}
```

当 `ki = 0` 时，意味着积分项在PID控制中不起作用（因为积分部分的计算是 `ki * errorInt`）。在这种情况下：

1. **避免积分累积浪费资源**：当ki为0时，即使继续累加errorInt也不会影响最终的控制输出，因为 `0 * errorInt = 0`。所以继续累加errorInt是浪费计算资源的行为。
2. **防止积分饱和**：即使当前ki为0，如果errorInt已经**积累了一个很大的值**，当用户后续调整ki为非零值时，这个大的积分值会立即对输出产生影响，可能导致系统突然产生很大的输出，引起超调或震荡。通过在ki为0时将errorInt清零，可以避免这种情况。
3. **提高系统稳定性**：这种处理方式确保了当积分作用被禁用时（ki=0），积分项不会积累历史误差，当重新启用积分作用时，系统能从一个干净的状态开始。

关于是否需要加入kd，只有当快act快接近tar时，呈现一头扎向tar，则需要加入kd来防止超调。

![image-20250926171447236](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926171447236.png)

**通过测试得到kp=1.10，ki=0.66，kd=0效果较为理想**

### 3.2 增量式PID定速控制

流程与位置式PID一样，全局变量与PID调参需要修改

```c
float target, actual, out;
float kp, ki, kd;
float error0, error1, error2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t count = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();
        count++;
        if (count >= 40) {
            count = 0;

            /* ===PID调控=== */
            /* 获取实际值 */
            actual = Encoder_Get();

            /* 获取本次误差，上次误差，上上次误差 */
            error2 = error1;
            error1 = error0;
            error0 = target - actual;

            /* PID计算 */
            out += kp * (error0 - error1) + ki * error0 + kd * (error0 - 2 * error1 + error2);

            /* 输出限幅 */
            if (out > 100) out = 100;
            if (out < -100) out = -100;

            /* 执行控制 */
            Motor_SetPWM(out);
        }
    }
}
```

### 3.3 位置式PID定位置控制

全局变量：

- target为目标位置，由用户指定
- actual为实际位置，由编码器测得
- out表示控制力度，作用于设置电机PWM的函数

```c
float target, actual, out;
float kp, ki, kd;
float error0, error1, errorInt;

/* 获取目标位置值，实测转一圈act为204 */
target = RP_GetValue(4) / 4095.0 * 408 - 204;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t count = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();
        count++;
        if (count >= 40) {
            count = 0;

            /* ===PID调控=== */
            /* 获取实际位置值 */
            actual += Encoder_Get();

            /* 获取本次误差和上次误差 */
            error1 = error0;
            error0 = target - actual;

            /* 误差积分（累加） */
            // errorInt += error0;
            if (ki != 0) {
                errorInt += error0;
            } else {
                errorInt = 0;
            }

            /* PID计算 */
            out = kp * error0 + ki * errorInt + kd * (error0 - error1);

            /* 输出限幅 */
            if (out > 100) out = 100;
            if (out < -100) out = -100;

            /* 执行控制 */
            Motor_SetPWM(out);
        }
    }
}
```

![image-20250926175316817](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926175316817.png)

纯kp控制效果已经很理想了，具体数值为kp=0.72，ki=0，kd=0.2

可以看到，位置式PID**没有产生稳态误差**。稳态误差产生的原因是当输出值为0时，实际值还会自发地偏移，而在这里，输出值为0时，电机的位置不会自发偏移。所以定位置控制没有稳态误差。

实际值与目标值有误差的原因：因为当输出的驱动力比较小的时候，电机转不起来，无法改变位置。这个误差可以称为**调控误差**

### 3.4 增量式PID定位置控制

实测kp=0.72，ki=0.01，kd=0.2效果较为理想

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t count = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();
        count++;
        if (count >= 40) {
            count = 0;

            /* ===PID调控=== */
            /* 获取实际值 */
            actual += Encoder_Get();

            /* 获取本次误差，上次误差，上上次误差 */
            error2 = error1;
            error1 = error0;
            error0 = target - actual;

            /* PID计算 */
            out += kp * (error0 - error1) + ki * error0 + kd * (error0 - 2 * error1 + error2);

            /* 输出限幅 */
            if (out > 100) out = 100;
            if (out < -100) out = -100;

            /* 执行控制 */
            Motor_SetPWM(out);
        }
    }
}
```

![image-20250926181156208](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250926181156208.png)

## 四、PID算法改进

- 积分限幅：限制积分的幅度，防止积分深度饱和
- 积分分离：误差小于一个限度才开始积分，反之则去掉积分部分
- 变速积分：根据误差的大小调整积分的速度
- 不完全微分：给微分项加入一阶惯性单元(低通滤波器)
- 微分先行：将对误差的微分替换为对实际值的微分
- 输出偏移：在非0输出时，给输出值加一个固定偏移
- 输入死区：误差小于一个限度时不进行调控

### 4.1 积分限幅

- 要解决的问题：如果执行器因为卡住、断电、损坏等原因不能消除误差，则误差积分会无限制加大，进而达到深度饱和状态，此时PID控制器会持续输出最大的调控力，即使后续执行器恢复正常， PID控制器在短时间内也会维持最大的调控力，直到误差积分从深度饱和状态退出
- **积分限幅实现思路**：对误差积分或积分项输出进行判断，如果幅值超过指定阈值，则进行限制

模拟电机卡住、断电、损坏的情况，添加全局变量`motor_flag`

```diff
--- 02-位置式PID定速控制/Core/Src/main.c
+++ 06-位置式PID定速控制-积分限幅/Core/Src/main.c
@@ -18,6 +18,7 @@
 /* USER CODE BEGIN PV */
+uint8_t motor_flag = 1;
 
 uint8_t key_num;
 int16_t pwm;
@@ -45,6 +46,10 @@
   while (1) {
+    key_num = Key_GetNum();
+    if (key_num == 1) {
+      motor_flag = !motor_flag;
+    }
+
+    if (motor_flag) {
+      LED_ON();
+    } else {
+      LED_OFF();
+    }
+
     /* 通过电位器修改PID参数 */
@@ -78,7 +89,7 @@
     OLED_Update();
 
-    Serial_Printf("%f,%f,%f\r\n", target, actual, out);
+    Serial_Printf("%f,%f,%f,%f\r\n", target, actual, out, errorInt);
 
     /* USER CODE END WHILE */
@@ -215,11 +226,15 @@
       /* 误差积分（累加） */
       // errorInt += error0;
       if (ki != 0) {
         errorInt += error0;
+        /* 积分限幅 */
+        if (errorInt > 1000) errorInt = 1000;
+        if (errorInt < -1000) errorInt = -1000;
       } else {
         errorInt = 0;
       }
 
       /* PID计算 */
@@ -234,7 +249,11 @@
       if (out < -100) out = -100;
 
+      /* 执行控制 */
+      if (motor_flag) {
+        Motor_SetPWM(out);
+      } else {
+        Motor_SetPWM(0);
+      }
+
-      /* 执行控制 */
-      Motor_SetPWM(out);
+
     }
   }
```

当`motor_flag=0`时，电机停转，此时`act=0`但`target`有值，误差始终存在，误差积分会不断累加，经过一段时间，误差积分就会陷入深度饱和状态。即该误差积分的值已经远超正常范围。此时让`motor_flag=1`电机立刻满速旋转，且无论如何修改target，电机都是满速运行。改变target只会影响误差值恢复正常的速度。误差不变导致积分持续正向累积，恢复后因超调产生反向误差，积分开始负向积分，从而使积分值逐渐恢复正常。

![image-20250929095904047](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929095904047.png)

#### 程序实现

![image-20250929100206016](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929100206016.png)

对于误差积分的上下限判断，可以通过实测`errorInt`，给`target`最大的值，测出`errorInt`。另一种方法则是通过`输出上下限/ki`

```c
/* 误差积分（累加） */
errorInt += error0;

/* 积分限幅 */
if (errorInt > 95) errorInt = 95;
if (errorInt < -95) errorInt = -95;

/* PID计算 */
...
```

### 4.2 积分分离

- 要解决的问题：积分项作用一般位于调控后期，用来消除持续的误差，调控前期一般误差较大且不需要积分项作用，如果此时仍然进行积分,则调控进行到后期时，积分项可能已经累积了过大的调控力，这会导致超调
- 积分分离实现思路：对误差大小进行判断，如果误差绝对值小于指定阈值，则加入积分项作用，反之，则直接将误差积分清零或不加入积分项作用

![image-20250929103150095](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929103150095.png)

阈值的判断通过实测，在给ki=0时，用手施加力使转盘偏移目标位置，最后PD控制电机贴近目标位置，得到误差最多不会超过5

![image-20250929103758810](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929103758810.png)

```c
/* 误差积分（累加） */
if (fabs(error0) < 6) {
    errorInt += error0;
} else {
    errorInt = 0;
}
```

最后测得ki=0.38时效果较好

使用积分分离在位置式PID定位置控制中，既可以实现积分项消除误差和对抗外力的效果，又可以避免过度积分导致超调的问题

### 4.3 变速积分

- 要解决的问题：如果积分分离阈值没有设定好，被控对象正好在阈值之外停下来，则此时控制器完全没有积分作用，误差不能消除
- 变速积分实现思路：变速积分是积分分离的升级版，变速积分需要设计一个函数值随误差绝对值增大而减小的函数，函数值作为调整系数，用于调整误差积分的速度或积分项作用的强度

![image-20250929105036712](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929105036712.png)

k为衰减速度

#### 非线性变速积分程序实现

![image-20250929105113916](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929105113916.png)

注意：该程序无抗积分饱和的效果，后续最好加上积分限幅

```c
/* 获取本次误差和上次误差 */
error1 = error0;
error0 = target - actual;

float c = 1 / (1 * fabs(error0) + 1);

/* 误差积分（累加） */
errorInt += c * error0;
```

![image-20250929110339601](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929110339601.png)

测得ki=0.25，k=1.1效果较为理想，但是还是没有普通的积分分离效果好

### 4.4 微分先行

要解决的问题：普通PID的微分项对误差进行微分，当目标值大幅度跳变时，误差也会瞬间大幅度跳变，这会导致微分项突然输出一个很大的调控力，如果系统的目标值频繁大幅度切换,则此时的微分项不利于系统稳定

微分先行实现思路：将对误差的微分替换为对实际值的微分

![image-20250929190130543](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929190130543.png)

<img src="./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929190101541.png" alt="image-20250929190101541" style="zoom:67%;" />

微分先行PID的微分项输出是`-kd`，这是因为该微分是对实际值的微分，从图中看到，实际值在目标值变化之后，微分是正的，加上`-`则就实现了阻尼。

![image-20250929192933036](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929192933036.png)

![image-20250929185916449](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929185916449.png)

可以看到当设置kp=0，ki=0时，此时为纯D项控制，当快速修改target，act依旧能贴合tar，这就违背了kd对act产生阻尼的初衷

#### 程序实现

![image-20250929190526340](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929190526340.png)

程序基于[3.3 位置式PID定位置控制](#33-pid)

```diff
/* USER CODE BEGIN PV */
-float target, actual, out;
+float target, actual, out, actual1;
+float diffOut;  // 微分项
/* USER CODE END PV */

-diffOut = kd * (error0 - error1);
+diffOut = -kd * (actual - actual1);
```

### 4.5 不完全微分

要解决的问题：传感器获取的实际值经常会受到**噪声干扰**，而PID控制器中的微分项对噪声最为敏感，这些噪声干扰可能会导致微分项输出抖动，进而影响系统性能

不完全微分实现思路：给微分项加入一阶惯性单元（低通滤波器)

![image-20250929191818352](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929191818352.png)

实际上就是对微分项进行加权平均

![image-20250929191951131](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929191951131.png)

图中绿线为理想状态下的无噪声的误差波形，红线是有噪声的误差波形，对P项的影响就是绿线与红线之间的误差，影响较小，对I项的影响也较小，I项是虚线下的面积，总面积相差不大；**对D项影响较大**，比如绿色斜线在无噪声情况下是正的，而在有噪声情况下变为负，影响较大。

![image-20250929192840296](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929192840296.png)

#### 程序实现

![image-20250929192950292](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929192950292.png)

模拟噪声

```diff
/* ===PID调控=== */
/* 获取实际位置值 */
actual += Encoder_Get();

+/* 模拟噪声，随机数范围-2~2 */
+actual += rand() % 10 - 5;

/* 获取本次误差和上次误差 */
error1 = error0;
error0 = target - actual;
```

![image-20250929193941104](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929193941104.png)

加入滤波

```diff
/* 误差积分（累加） */
...

+float a = 0.9;  // 加权
+diffOut = (1 - a) * (kd * (error0 - error1)) + a * diffOut;

/* PID计算 */
+out = kp * error0 + ki * errorInt + diffOut;
```

![image-20250929194423054](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929194423054.png)

可以看到diffOut的输出变的平缓了。但实际位置由于存在噪声，无法做到紧贴target

### 4.6 输出偏移

要解决的问题：对于一些启动需要一定力度的执行器，若输出值较小，执行器可能完全无动作，这可能会引起调控误差，同时会降低系统响应速度

输出偏移实现思路：若输出值为0，则正常输出0，不进行调控；若输出值非0，则给输出值加一个固定偏移，跳过执行器无动作的阶段

![image-20250929202003844](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929202003844.png)

#### 程序实现

![image-20250929202642565](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929202642565.png)

通过实测得到当out>=7时，电机才开始转动。实测方法：通过按键逐步增大out，直到电机转动。

```c
/* PID计算 */
out = kp * error0 + ki * errorInt + kd * (error0 - error1);

float offset = 7;
if (out > 0) {
    out += offset;
} else if (out < 0) {
    out -= offset;
} else {
    out = 0;
}

/* 输出限幅 */
if (out > 100) out = 100;
if (out < -100) out = -100;
```

加入输出偏移后，out不会出现-offset~offset之间的值，但是电机会出现频繁的抖动，这是因为PID一直处于调控状态。解决办法：输入死区

### 4.7 输入死区

要解决的问题：在某些系统中，输入的目标值或实际值有微小的噪声波动，或者系统有一定的滞后，这些情况可能会导致执行器在误差很小时频繁调控，不能最终稳定下来

输入死区实现思路：若误差绝对值小于一个限度，则固定输出0，不进行调控

![image-20250929203736969](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929203736969.png)

#### 程序实现

![image-20250929203906793](./PID%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.assets/image-20250929203906793.png)

```c
/* 获取本次误差和上次误差 */
error1 = error0;
error0 = target - actual;

if (fabs(error0) < 2) {
    out = 0;
    errorInt = 0;
} else {
    /* 误差积分（累加） */
    // errorInt += error0;
    if (ki != 0) {
        errorInt += error0;
    } else {
        errorInt = 0;
    }

    /* PID计算 */
    out = kp * error0 + ki * errorInt + kd * (error0 - error1);

    float offset = 7;
    if (out > 0) {
        out += offset;
    } else if (out < 0) {
        out -= offset;
    } else {
        out = 0;
    }
}

/* 输出限幅 */
if (out > 100) out = 100;
if (out < -100) out = -100;
```

