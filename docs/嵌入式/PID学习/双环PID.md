# 双环PID

## 一、多环串级PID

单环PD只能对被控对象的一个物理量进行闭环控制，而当用户需要对被控对象的多个维度物理量（例如：速度、位置、角度等）进行控制时，则需要多个PID控制环路，即多环PID,多个PID串级连接，因此也称作串级PID

多环PID相较于单环PID,功能上，可以实现对更多物理量的控制，性能上，可以使系统拥有更高的准确性、稳定性和响应速度

![image-20251001102733771](./%E5%8F%8C%E7%8E%AFPID.assets/image-20251001102733771.png)

定位置控制单环与双环比较

![image-20251001103638980](./%E5%8F%8C%E7%8E%AFPID.assets/image-20251001103638980.png)

### 程序实现

![image-20251001104038666](./%E5%8F%8C%E7%8E%AFPID.assets/image-20251001104038666.png)

![image-20251001104342329](./%E5%8F%8C%E7%8E%AFPID.assets/image-20251001104342329.png)

在进行PID调参时，要先对内环PID进行调参，则需要将外环PID控制的代码注释

封装PID结构体

`PID.h`

```c
#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct PID_t {
  float target, actual, out;
  float kp, ki, kd;
  float error0, error1, errorInt;

  float maxOut, minOut;
} PID_t;

void PID_Update(PID_t* p);

#endif  // !__PID_H

```

`PID.c`

```c
#include "PID.h"

void PID_Update(PID_t* p) {
  p->error1 = p->error0;
  p->error0 = p->target - p->actual;

  if (p->ki != 0) {
    p->errorInt += p->error0;
  } else {
    p->errorInt = 0;
  }

  p->out = p->kp * p->error0 + p->ki * p->errorInt + p->kd * (p->error0 - p->error1);

  if (p->out > p->maxOut) p->out = p->maxOut;
  if (p->out < p->minOut) p->out = p->minOut;
}

```

在`main.c`中声明全局变量

```c
/* USER CODE BEGIN PV */
uint8_t key_num;
int16_t pwm;
int16_t speed, location;

PID_t inner = {
    .kp = 1.1,
    .ki = 0.66,
    .kd = 0,

    .maxOut = 100,
    .minOut = -100};

PID_t outer = {
    .kp = 0.3,
    .ki = 0,
    .kd = 0.2,

    .maxOut = 100,
    .minOut = -100};

/* USER CODE END PV */
```

中断回调函数

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t count1 = 0, count2 = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();

        /* 内环调控，速度控制 */
        count1++;
        if (count1 >= 40) {
            count1 = 0;

            /* ===PID调控=== */
            /* 获取实际值 */
            speed = Encoder_Get();
            location += speed;

            inner.actual = speed;

            PID_Update(&inner);

            /* 执行控制 */
            Motor_SetPWM(inner.out);
        }

        /* 外环调控，位置控制 */
        count2++;
        if (count2 >= 40) {
            count2 = 0;

            /* ===PID调控=== */
            /* 获取实际值 */
            outer.actual = location;

            PID_Update(&outer);

            /* 内环输入为外环输出 */
            inner.target = outer.out;
        }
    }
}
```

## 二、倒立摆

![image-20251002101119196](./%E5%8F%8C%E7%8E%AFPID.assets/image-20251002101119196.png)

在只有内环时，能维持倒立摆不倒的情况下，转盘没有固定的位置，会一直旋转来维持稳定。当加入外环位置环后，固定转盘的位置，输出角度值，让内环在这个角度值稳定，并输出PWM给电机，电机再旋转使转盘固定在目标位置

内环角度环PID调参：

```c
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CENTER_ANGLE 1995  // 中心角度
#define CENTER_RANGE 500   // 中心角度偏移范围
/* USER CODE END PD */

/* USER CODE BEGIN PV */
uint8_t KeyNum;
uint8_t RunState;  // 倒立摆运行状态
uint16_t Angle;    // 角度传感器
int16_t Location;  // 电机位置

/* 内环角度环 */
PID_t AnglePID = {
    .Target = CENTER_ANGLE,

    .Kp = 0.3,
    .Ki = 0.01,
    .Kd = 0.4,

    .OutMax = 100,
    .OutMin = -100,
};
/* 外环位置环 */
PID_t LocationPID = {
    .Target = 0,

    .Kp = 1,
    .Ki = 0.01,
    .Kd = 10,

    /* 外环输出位内环角度target */
    /* 限幅为限制内环角度在中心角度的区间范围+-100 */
    .OutMax = 100,
    .OutMin = -100,
};
/* USER CODE END PV */

int main(void) {
    // ...
    All_Init();
    while (1) {
        /* 按键控制倒立摆启停 */
        KeyNum = Key_GetNum();
        if (KeyNum == 1) {
            RunState = !RunState;
        } else if (KeyNum == 2) {
            LocationPID.Target += 204;
        } else if (KeyNum == 3) {
            LocationPID.Target -= 204;
        }

        if (RunState) {
            LED_ON();
        } else {
            LED_OFF();
        }

        AnglePID.Kp = RP_GetValue(1) / 4095.0 * 1;
        AnglePID.Ki = RP_GetValue(2) / 4095.0 * 1;
        AnglePID.Kd = RP_GetValue(3) / 4095.0 * 1;

        OLED_Printf(0, 0, OLED_6X8, "Angle");
        OLED_Printf(0, 12, OLED_6X8, "Kp:%05.3f", AnglePID.Kp);
        OLED_Printf(0, 20, OLED_6X8, "Ki:%05.3f", AnglePID.Ki);
        OLED_Printf(0, 28, OLED_6X8, "Kd:%05.3f", AnglePID.Kd);
        OLED_Printf(0, 40, OLED_6X8, "Tar:%04.0f", AnglePID.Target);
        OLED_Printf(0, 48, OLED_6X8, "Act:%04d", Angle);
        OLED_Printf(0, 56, OLED_6X8, "Out:%+04.0f", AnglePID.Out);
        OLED_Update();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    static uint16_t Count1 = 0, Count2 = 0;
    if (htim->Instance == TIM1) {
        Key_Tick();  // 消抖

        Angle = AD_GetValue();  // 获取角度传感器数据
        Location += Encoder_Get();
        /* 判断倒立摆是否在可调区间内 */
        if (Angle < CENTER_ANGLE - CENTER_RANGE || Angle > CENTER_ANGLE + CENTER_RANGE) RunState = 0;

        if (RunState) {
            /* 内环角度环PID控制 */
            Count1++;
            if (Count1 >= 5) {  // 5ms执行一次
                Count1 = 0;
                AnglePID.Actual = Angle;

                PID_Update(&AnglePID);
                Motor_SetPWM(AnglePID.Out);
            }

            /* 外环位置环PID控制 */
            Count2++;
            if (Count2 >= 50) {
                Count2 = 0;
                LocationPID.Actual = Location;

                PID_Update2(&LocationPID, 6);
                /* 内环目标值=中心角度+外环输出 */
                AnglePID.Target = CENTER_ANGLE - LocationPID.Out;
            }

        } else {
            Motor_SetPWM(0);
        }
    }
}
/* USER CODE END 4 */

```

当倒立摆在倒立时电机始终往一个方向跑，则说明中心角度值出现偏差，适当修改中心角度值即可。

外环位置环加入KP项后，发现电机越来越快，说明极性错了，将`AnglePID.Target = CENTER_ANGLE + LocationPID.Out`改为`AnglePID.Target = CENTER_ANGLE - LocationPID.Out`

在外环加入ki后需要加入积分分离

```c
#include "PID.h"
#include <math.h>

void PID_Update(PID_t* p) {
  p->Error1 = p->Error0;
  p->Error0 = p->Target - p->Actual;

  if (p->Ki != 0) {
    p->ErrorInt += p->Error0;
  } else {
    p->ErrorInt = 0;
  }

  p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

  if (p->Out > p->OutMax) p->Out = p->OutMax;
  if (p->Out < p->OutMin) p->Out = p->OutMin;
}

/* 加入积分分离 */
void PID_Update2(PID_t* p, float error_threshold) {
  p->Error1 = p->Error0;
  p->Error0 = p->Target - p->Actual;

  if (fabs(p->Error0) < error_threshold) {
    p->ErrorInt += p->Error0;
  } else {
    p->ErrorInt = 0;
  }

  p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

  if (p->Out > p->OutMax) p->Out = p->OutMax;
  if (p->Out < p->OutMin) p->Out = p->OutMin;
}

```

### 倒立摆自启程序

自动启摆流程

- 如果摆杆位于右侧区间且处于最高点位置，则横杆需要向左施加瞬时驱动力
- 如果摆杆位于左侧区间且处于最高点位置，则横杆需要向右施加瞬时驱动力
- 如果摆杆进入中心区间，则启摆结束，开始执行PD控制程序

![image-20251002143607886](./%E5%8F%8C%E7%8E%AFPID.assets/image-20251002143607886.png)

```c
#define CENTER_ANGLE 1995  // 中心角度
#define CENTER_RANGE 500   // 中心角度偏移范围

#define START_PWM 35    // 启摆的瞬时PWM
#define START_TIME 100  // 启摆延时时间

int main(){
    // ...
    /* 按键控制倒立摆启停 */
    KeyNum = Key_GetNum();
    if (KeyNum == 1) {
        if (RunState == 0) {  // 停止状态，启动倒立摆
            RunState = 21;
        } else {
            RunState = 0;
        }
    }

    if (KeyNum == 2) {
        LocationPID.Target += 204;
    }
    if (KeyNum == 3) {
        LocationPID.Target -= 204;
    }

    //...

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    /* 内环分频、外环分频、启摆延时、角度值采样分频 */
    static uint16_t Count1 = 0, Count2 = 0, CountTime = 0, Count3 = 0;
    static int16_t Angle0, Angle1, Angle2;
    if (htim->Instance == TIM1) {
        Key_Tick();  // 消抖

        Angle = AD_GetValue();  // 获取角度传感器数据
        Location += Encoder_Get();

        if (RunState == 0) {  // 停止
            Motor_SetPWM(0);
        } else if (RunState == 1) {  // 根据角度值确定下一步要转入的状态
            Count3++;
            if (Count3 >= 40) {
                Count3 = 0;
                Angle2 = Angle1;
                Angle1 = Angle0;
                Angle0 = Angle;

                /* 判断三次采样值在右侧同时中间的Angle1是最低点 */
                if (Angle0 > CENTER_ANGLE + CENTER_RANGE &&
                    Angle1 > CENTER_ANGLE + CENTER_RANGE &&
                    Angle2 > CENTER_ANGLE + CENTER_RANGE &&
                    Angle1 < Angle0 && Angle1 < Angle2) {
                    RunState = 21;  // 在右侧就顺时针摆动
                }

                /* 判断三次采样值在左侧同时中间的Angle1是最高点 */
                if (Angle0 < CENTER_ANGLE - CENTER_RANGE &&
                    Angle1 < CENTER_ANGLE - CENTER_RANGE &&
                    Angle2 < CENTER_ANGLE - CENTER_RANGE &&
                    Angle1 > Angle0 && Angle1 > Angle2) {
                    RunState = 31;  // 在左侧就逆时针摆动
                }

                /* 三次采样值在中心区间 */
                if (Angle0 > CENTER_ANGLE - CENTER_RANGE &&
                    Angle0 < CENTER_ANGLE + CENTER_RANGE &&
                    Angle1 > CENTER_ANGLE - CENTER_RANGE &&
                    Angle1 < CENTER_ANGLE + CENTER_RANGE &&
                    Angle2 > CENTER_ANGLE - CENTER_RANGE &&
                    Angle2 < CENTER_ANGLE + CENTER_RANGE) {
                    Location = 0;
                    AnglePID.ErrorInt = 0;  // 清零积分，防止积分累积
                    LocationPID.ErrorInt = 0;
                    RunState = 4;  // 进入倒立摆程序
                }
            }
        } else if (RunState == 21) {  // 顺时针摆动PWM+
            Motor_SetPWM(START_PWM);
            CountTime = START_TIME;
            RunState = 22;
        } else if (RunState == 22) {  // 延时计数计时
            CountTime--;
            if (CountTime == 0) {
                RunState = 23;
            }
        } else if (RunState == 23) {  // 逆时针摆动PWM-
            Motor_SetPWM(-START_PWM);
            CountTime = START_TIME;
            RunState = 24;
        } else if (RunState == 24) {  // 延时计数计时
            CountTime--;
            if (CountTime == 0) {
                Motor_SetPWM(0);
                RunState = 1;
            }
        } else if (RunState == 31) {
            Motor_SetPWM(-START_PWM);
            CountTime = START_TIME;
            RunState = 32;
        } else if (RunState == 32) {
            CountTime--;
            if (CountTime == 0) {
                RunState = 33;
            }
        } else if (RunState == 33) {
            Motor_SetPWM(START_PWM);
            CountTime = START_TIME;
            RunState = 34;
        } else if (RunState == 34) {
            CountTime--;
            if (CountTime == 0) {
                Motor_SetPWM(0);
                RunState = 1;
            }
        } else if (RunState == 4) {
            /* 判断倒立摆是否在可调区间内 */
            if (Angle < CENTER_ANGLE - CENTER_RANGE || Angle > CENTER_ANGLE + CENTER_RANGE) RunState = 0;

            /* 内环角度环PID控制 */
            Count1++;
            if (Count1 >= 5) {  // 5ms执行一次
                Count1 = 0;
                AnglePID.Actual = Angle;

                PID_Update(&AnglePID);
                Motor_SetPWM(AnglePID.Out);
            }

            /* 外环位置环PID控制 */
            Count2++;
            if (Count2 >= 50) {
                Count2 = 0;
                LocationPID.Actual = Location;

                PID_Update2(&LocationPID, 6);
                /* 内环目标值=中心角度+外环输出 */
                AnglePID.Target = CENTER_ANGLE - LocationPID.Out;
            }
        }
    }
}
```

```c
/* 三次采样值在中心区间 */
if (Angle0 > CENTER_ANGLE - CENTER_RANGE &&
    Angle0 < CENTER_ANGLE + CENTER_RANGE &&
    Angle1 > CENTER_ANGLE - CENTER_RANGE &&
    Angle1 < CENTER_ANGLE + CENTER_RANGE &&
    Angle2 > CENTER_ANGLE - CENTER_RANGE &&
    Angle2 < CENTER_ANGLE + CENTER_RANGE) {
    Location = 0;
    AnglePID.ErrorInt = 0;  // 清零积分，防止积分累积
    LocationPID.ErrorInt = 0;
    RunState = 4;  // 进入倒立摆程序
}
```

在将倒立摆长时间处于一个偏差位置时，积分项会积累过大，导致每次启摆都会立即退出倒立摆状态，这是因为积分项积累过大使得`Ki*ErrorInt`输出值非常大，会立刻给电机一个巨大的PWM，导致偏移然后退出倒立摆状态。此处清零积分的操作也可以在执行PID时加入积分限幅来实现相同的效果
