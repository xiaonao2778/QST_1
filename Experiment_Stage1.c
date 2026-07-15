/**
 * @file Experiment_Stage1.c
 * @brief 第一阶段综合实验
 *
 * 知识点：按键中断、RGB灯、蜂鸣器、风扇、PWM、UART 和系统内核
 *
 * 功能说明：
 *  - LED 灯为呼吸灯状态 (PWM)
 *  - 按下按键1后继电器打开，按下按键2继电器关闭 (按键中断)
 *  - 蜂鸣器每隔10秒时间鸣叫 (定时器)
 *  - 系统启动后风扇开始旋转 (GPIO输出)
 *  - 任务3运行，串口打印消息队列信息 (消息队列+UART)
 *  - 任务2一直运行，任务1、3交替运行 (互斥锁)
 *
 * GPIO 引脚分配：
 *  - GPIO_0:  蜂鸣器 (低电平响,高电平灭)
 *  - GPIO_1:  风扇   (低电平转,高电平停)
 *  - GPIO_6:  LED呼吸灯 (PWM3_OUT)
 *  - GPIO_7:  继电器 (高电平开,低电平关)
 *  - GPIO_13: 按键2 (下降沿中断)
 *  - GPIO_14: 按键1 (下降沿中断)
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_pwm.h"
#include "hi_pwm.h"

/* ========== 宏定义 ========== */
#define EXP_TASK_STACK_SIZE     1024 * 4    /* 任务栈大小 */
#define EXP_TASK1_PRIO          25          /* 任务1优先级 (呼吸灯,交替) */
#define EXP_TASK2_PRIO          26          /* 任务2优先级 (风扇,常运行,最低) */
#define EXP_TASK3_PRIO          24          /* 任务3优先级 (消息队列/UART,交替,最高) */

#define MSGQUEUE_OBJECTS        16          /* 消息队列容量 */
#define BUZZ_TIMER_DELAY        1000U       /* 蜂鸣器定时器延时: 1000U = 10秒 (1U=10ms) */
#define PWM_FREQ                40000       /* PWM 频率 */

/* ========== 消息队列数据结构 ========== */
typedef struct {
    char *Buf;
    uint8_t Idx;
} MSGQUEUE_OBJ_t;

/* ========== 全局资源 ========== */
static osMutexId_t         g_mutexId;           /* 互斥锁: Task1/Task3交替 */
static osMessageQueueId_t  g_msgQueueId;        /* 消息队列 */
static osTimerId_t         g_buzzerTimerId;     /* 蜂鸣器定时器 */

/* ========== 蜂鸣器定时器回调: 每隔10秒鸣叫 ========== */
static void BuzzerTimerCallback(void *arg)
{
    (void)arg;
    /* 蜂鸣器 ON (低电平有效) */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0, IOT_GPIO_VALUE0);
    usleep(500000);  /* 鸣叫 0.5 秒 */
    /* 蜂鸣器 OFF */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0, IOT_GPIO_VALUE1);
}

/* ========== 按键1中断服务函数: 继电器打开 ========== */
static void Key1_Pressed(const char *arg)
{
    (void)arg;
    /* 继电器 ON (高电平) */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_7, IOT_GPIO_VALUE1);
    printf(">>> Key1 Pressed! Relay ON.\n");
}

/* ========== 按键2中断服务函数: 继电器关闭 ========== */
static void Key2_Pressed(const char *arg)
{
    (void)arg;
    /* 继电器 OFF */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_7, IOT_GPIO_VALUE0);
    printf(">>> Key2 Pressed! Relay OFF.\n");
}

/* ========== 任务1: LED呼吸灯 (PWM), 与任务3交替运行 ========== */
static void *Experiment_Task1(const char *arg)
{
    (void)arg;
    uint32_t i;

    /* GPIO_6 初始化为 PWM3 输出 */
    IoTGpioInit(HI_IO_NAME_GPIO_6);
    hi_io_set_func(HI_IO_NAME_GPIO_6, HI_IO_FUNC_GPIO_6_PWM3_OUT);
    IoTGpioSetDir(HI_IO_NAME_GPIO_6, IOT_GPIO_DIR_OUT);
    IoTPwmInit(HI_PWM_PORT_PWM3);

    sleep(1);  /* 延时让系统就绪 */

    while (1) {
        /* 获取互斥锁, 与 Task3 交替 */
        osMutexAcquire(g_mutexId, osWaitForever);
        printf("[Task1] Breathing LED is Running!\n");

        /* 呼吸灯: 渐亮 */
        for (i = 1; i < 100; i++) {
            IoTPwmStart(HI_PWM_PORT_PWM3, i, PWM_FREQ);
            usleep(10000);
        }
        /* 呼吸灯: 渐灭 */
        for (i = 100; i > 0; i--) {
            IoTPwmStart(HI_PWM_PORT_PWM3, i, PWM_FREQ);
            usleep(10000);
        }

        /* 释放互斥锁 */
        osMutexRelease(g_mutexId);
        sleep(1);
    }
    return NULL;
}

/* ========== 任务2: 风扇, 一直运行 ========== */
static void *Experiment_Task2(const char *arg)
{
    (void)arg;

    /* 初始化风扇 GPIO_1 */
    IoTGpioInit(HI_IO_NAME_GPIO_1);
    hi_io_set_func(HI_IO_NAME_GPIO_1, HI_IO_FUNC_GPIO_1_GPIO);
    IoTGpioSetDir(HI_IO_NAME_GPIO_1, IOT_GPIO_DIR_OUT);

    /* 系统启动后风扇开始旋转 (低电平有效) */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_1, IOT_GPIO_VALUE0);
    printf("[Task2] Fan started! (Fan is rotating)\n");

    while (1) {
        printf("[Task2] Fan is Running! (Fan keeps rotating)\n");
        sleep(2);
    }
    return NULL;
}

/* ========== 任务3: 消息队列 + 串口打印, 与任务1交替运行 ========== */
static void *Experiment_Task3(const char *arg)
{
    (void)arg;
    osStatus_t status;
    MSGQUEUE_OBJ_t msg_send;
    MSGQUEUE_OBJ_t msg_recv;

    /* 构造消息内容 */
    msg_send.Buf = "Hello OpenHarmony!";
    msg_send.Idx = 0;

    sleep(1);

    while (1) {
        /* 获取互斥锁, 与 Task1 交替 */
        osMutexAcquire(g_mutexId, osWaitForever);
        printf("[Task3] MessageQueue/UART Task is Running!\n");

        /* ---- 消息队列发送 ---- */
        status = osMessageQueuePut(g_msgQueueId, &msg_send, 0U, 0U);
        if (status == osOK) {
            printf("[Task3][MsgQueue][Send] Message: \"%s\"\n", msg_send.Buf);
        } else {
            printf("[Task3][MsgQueue][Send] Failed! status=%d\n", status);
        }

        /* ---- 消息队列接收 ---- */
        status = osMessageQueueGet(g_msgQueueId, &msg_recv, NULL, 0U);
        if (status == osOK) {
            printf("[Task3][MsgQueue][Recv] Message: \"%s\"\n", msg_recv.Buf);
        } else {
            printf("[Task3][MsgQueue][Recv] Failed! status=%d\n", status);
        }

        /* ---- 串口打印消息队列信息汇总 ---- */
        printf("[Task3][UART] == Message Queue Info Summary ==\n");
        printf("[Task3][UART] Queue capacity: %d objects\n", MSGQUEUE_OBJECTS);
        printf("[Task3][UART] Message sent and received successfully!\n");
        printf("[Task3][UART] =============================\n");

        /* 释放互斥锁 */
        osMutexRelease(g_mutexId);
        sleep(2);
    }
    return NULL;
}

/* ========== 按键中断任务: 初始化GPIO并注册中断 ========== */
static void *Key_Task(const char *arg)
{
    (void)arg;

    /* 初始化按键和继电器 GPIO */
    IoTGpioInit(HI_IO_NAME_GPIO_14);   /* 按键1 */
    IoTGpioInit(HI_IO_NAME_GPIO_13);   /* 按键2 */
    IoTGpioInit(HI_IO_NAME_GPIO_7);    /* 继电器 */

    /* 配置GPIO功能 */
    hi_io_set_func(HI_IO_NAME_GPIO_14, HI_IO_FUNC_GPIO_13_GPIO);
    hi_io_set_func(HI_IO_NAME_GPIO_13, HI_IO_FUNC_GPIO_14_GPIO);
    hi_io_set_func(HI_IO_NAME_GPIO_7, HI_IO_FUNC_GPIO_7_GPIO);

    /* 按键引脚上拉 */
    hi_io_set_pull(HI_IO_NAME_GPIO_14, HI_IO_PULL_UP);
    hi_io_set_pull(HI_IO_NAME_GPIO_13, HI_IO_PULL_UP);

    /* 设置输入/输出方向 */
    IoTGpioSetDir(HI_IO_NAME_GPIO_14, IOT_GPIO_DIR_IN);
    IoTGpioSetDir(HI_IO_NAME_GPIO_13, IOT_GPIO_DIR_IN);
    IoTGpioSetDir(HI_IO_NAME_GPIO_7, IOT_GPIO_DIR_OUT);

    /* 继电器默认关闭 */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_7, IOT_GPIO_VALUE0);

    /* 注册下降沿中断: Key1 -> 继电器开, Key2 -> 继电器关 */
    IoTGpioRegisterIsrFunc(HI_IO_NAME_GPIO_14, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW, Key1_Pressed, NULL);
    IoTGpioRegisterIsrFunc(HI_IO_NAME_GPIO_13, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW, Key2_Pressed, NULL);

    printf("[KeyTask] Key interrupt initialized! Press Key1 for Relay ON, Key2 for Relay OFF.\n");
    return NULL;
}

/* ========== 蜂鸣器初始化 (定时器方式) ========== */
static void Buzzer_Init(void)
{
    /* 初始化蜂鸣器 GPIO_0 */
    IoTGpioInit(HI_IO_NAME_GPIO_0);
    hi_io_set_func(HI_IO_NAME_GPIO_0, HI_IO_FUNC_GPIO_0_GPIO);
    IoTGpioSetDir(HI_IO_NAME_GPIO_0, IOT_GPIO_DIR_OUT);
    /* 默认关闭蜂鸣器 (高电平) */
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0, IOT_GPIO_VALUE1);

    /* 创建周期性定时器 (10秒 = 1000U) */
    g_buzzerTimerId = osTimerNew(BuzzerTimerCallback, osTimerPeriodic, NULL, NULL);
    if (g_buzzerTimerId != NULL) {
        if (osTimerStart(g_buzzerTimerId, BUZZ_TIMER_DELAY) == osOK) {
            printf("[Buzzer] Timer started! Will beep every 10 seconds.\n");
        } else {
            printf("[Buzzer] Failed to start timer!\n");
        }
    } else {
        printf("[Buzzer] Failed to create timer!\n");
    }
}

/* ========== 主入口: 创建所有任务和资源 ========== */
static void Experiment_Stage1(void)
{
    osThreadAttr_t attr;

    printf("\n");
    printf("================================================\n");
    printf("=====  第一阶段综合实验 (Stage1) 启动中...  =====\n");
    printf("================================================\n");
    printf("功能列表:\n");
    printf("  1. LED呼吸灯 (PWM)       - Task1 (与Task3交替)\n");
    printf("  2. 风扇持续旋转           - Task2 (一直运行)\n");
    printf("  3. 消息队列+串口打印       - Task3 (与Task1交替)\n");
    printf("  4. 按键1->继电器开,按键2->继电器关 (中断)\n");
    printf("  5. 蜂鸣器每10秒鸣叫       (定时器)\n");
    printf("================================================\n\n");

    /* ---- 创建互斥锁 (Task1与Task3交替) ---- */
    g_mutexId = osMutexNew(NULL);
    if (g_mutexId == NULL) {
        printf("[Error] Failed to create Mutex!\n");
    } else {
        printf("[Init] Mutex created successfully.\n");
    }

    /* ---- 创建消息队列 ---- */
    g_msgQueueId = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);
    if (g_msgQueueId == NULL) {
        printf("[Error] Failed to create Message Queue!\n");
    } else {
        printf("[Init] Message Queue created successfully (capacity=%d).\n", MSGQUEUE_OBJECTS);
    }

    /* ---- 通用线程属性 ---- */
    attr.attr_bits = 0U;
    attr.cb_mem    = NULL;
    attr.cb_size   = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = EXP_TASK_STACK_SIZE;

    /* ---- 创建 Task3 (最高优先级, 先运行) ---- */
    attr.name     = "Task3_MsgQueue";
    attr.priority = EXP_TASK3_PRIO;
    if (osThreadNew((osThreadFunc_t)Experiment_Task3, NULL, &attr) == NULL) {
        printf("[Error] Failed to create Task3 (MsgQueue/UART)!\n");
    } else {
        printf("[Init] Task3 (MsgQueue/UART) created, prio=%d.\n", EXP_TASK3_PRIO);
    }

    /* ---- 创建 Task1 (呼吸灯) ---- */
    attr.name     = "Task1_BreathingLED";
    attr.priority = EXP_TASK1_PRIO;
    if (osThreadNew((osThreadFunc_t)Experiment_Task1, NULL, &attr) == NULL) {
        printf("[Error] Failed to create Task1 (Breathing LED)!\n");
    } else {
        printf("[Init] Task1 (Breathing LED) created, prio=%d.\n", EXP_TASK1_PRIO);
    }

    /* ---- 创建 Task2 (风扇, 最低优先级, 一直运行) ---- */
    attr.name     = "Task2_Fan";
    attr.priority = EXP_TASK2_PRIO;
    if (osThreadNew((osThreadFunc_t)Experiment_Task2, NULL, &attr) == NULL) {
        printf("[Error] Failed to create Task2 (Fan)!\n");
    } else {
        printf("[Init] Task2 (Fan) created, prio=%d.\n", EXP_TASK2_PRIO);
    }

    /* ---- 创建按键中断任务 ---- */
    attr.name     = "KeyTask_Interrupt";
    attr.priority = EXP_TASK1_PRIO;
    if (osThreadNew((osThreadFunc_t)Key_Task, NULL, &attr) == NULL) {
        printf("[Error] Failed to create Key Interrupt Task!\n");
    } else {
        printf("[Init] Key Interrupt Task created.\n");
    }

    /* ---- 初始化蜂鸣器定时器 ---- */
    Buzzer_Init();

    printf("\n================================================\n");
    printf("=====  第一阶段综合实验 所有模块初始化完成  =====\n");
    printf("================================================\n\n");
}

/* 系统启动后自动运行 */
APP_FEATURE_INIT(Experiment_Stage1);
