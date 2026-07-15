#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "Peripheral.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_log.h"
#include "hal_bsp_ap3216c.h"
#include "hal_bsp_ssd1306.h"
#include "nfc.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_errno.h"
#include "iot_i2c.h"
#include "hi_i2c.h"
#include "hi_adc.h"

#include "hal_bsp_nfc.h"
#include "hal_bsp_log.h"

#include "config.h"
#include "mpu6050.h"

uint8_t displayBuff_0[20] = {0};
uint8_t displayBuff_1[20] = {0};
uint8_t displayBuff_2[20] = {0};
uint8_t displayBuff_3[20] = {0};

osSemaphoreId_t sem1;//定义信号量

float temperature = 0;//温度数据
float humidity = 0;//湿度数据

uint16_t ir = 0;//人体红外传感器
uint16_t als = 0;//接近传感器
uint16_t ps = 0;//光照强度传感器

hi_s16 mpu_dates[7];
hi_u8 mpu_ds[14] = {0}; 
hi_u8 mpu_ln = 14;

#define IoT_KEY2_GPIO_13        13

osMutexId_t g_mutexId;               /* 互斥锁 Task1/Task3交替 */
osMessageQueueId_t g_msgQueueId;     /* 消息队列 */
#define MSGQUEUE_OBJECTS 16

//获取电压函数
float Get_Voltage()
{
    unsigned int ret;
    unsigned short data;

    ret = hi_adc_read(HI_ADC_CHANNEL_6, &data, HI_ADC_EQU_MODEL_8, HI_ADC_CUR_BAIS_DEFAULT, 0xff);
    if(ret != IOT_SUCCESS){
        printf("ADC Read Fail\n");
        return 0;
    }

    return (float)data * 1.8 * 4 / 4096.0;
}

//外设功能初始化，OLED进行清屏操作，释放信号量使获取信号量函数得到信号量，读取传感器数据并将数据保存到数组中。
static void Stage2_Task1(void)
{
    float Voltage;

    Peripheral_Init();//外设功能初始化
    hi_io_set_pull(IoT_KEY2_GPIO_13,HI_IO_PULL_UP);//上拉,让按键未按下时GPIO_13保持高电平状态

    SSD1306_CLS(); // 清屏
    Mpu6050_Init();

    while(1){
        osMutexAcquire(g_mutexId, osWaitForever);       /* 获取互斥锁, 与Task3交替 */
	//释放sem1信号量
	osSemaphoreRelease(sem1);
	printf("sem1 has released\n");
	
	//读取温湿度传感器
	SHT20_ReadData(&temperature, &humidity);
	printf("temperature:%.2f;humidity:%.2f\r\n", temperature, humidity);
	
	//读取三合一传感器
	AP3216C_ReadData(&ir, &als, &ps);
	printf("ir:%d;als:%d;ps:%d\r\n", ir, als, ps);
	
	//ii读取初始化
	Mpu6050_Measure_By(mpu_ds, mpu_ln);
	
	//iic信息采集
	Mpu6050_Measure_Sh(mpu_dates);
	
	//保存到对应列表
	printf("accel x = %d, accel y = %d, accel z = %d, gyro x = %d, gyro y = %d, gyro z = %d,\r\n",
	       mpu_dates[0], mpu_dates[1], mpu_dates[2],
	       mpu_dates[3], mpu_dates[4], mpu_dates[5], mpu_dates[6]);
        
        osMutexRelease(g_mutexId);                      /* 释放互斥锁 */
        sleep(2);
    }
}

//函数功能说明：此线程在阻塞3秒后清楚屏幕数据，在获得信号量之后将传感器数据信息通过OLED示出来
static void Stage2_Task2(void)
{       
    sleep(3);
    SSD1306_CLS();//清屏
    while(1){
    
    //等待sem1信号量
    osSemaphoreAcquire(sem1, osWaitForever);
    printf("sem1 has gotten\n");
    
    //温湿度
    memset(displayBuff_0, 0, sizeof(displayBuff_0));
    sprintf((char*)displayBuff_0, "T:%.1fC H:%.1f%%", temperature, humidity);
    SSD1306_ShowStr(0, 0, displayBuff_0, 16);
    
    /// 人体红外传感器 接近传感器 光照强度传感器
    memset(displayBuff_1, 0, sizeof(displayBuff_1));
    sprintf((char*)displayBuff_1, "IR:%d ALS:%d", ir, als);
    SSD1306_ShowStr(0, 2, displayBuff_1, 16);
       
    memset(displayBuff_2, 0, sizeof(displayBuff_2));
    sprintf((char*)displayBuff_2, "PS:%d", ps);
    SSD1306_ShowStr(0, 4, displayBuff_2, 16);
    }    
}

//函数功能说明：此线程在阻塞1秒后进行NFC的初始化，并将存储在NFC的数据读取出来 存储到相应地址中
static void Stage2_Task3(void)
{
    uint8_t ndefLen = 0;//ndef包的长度
    uint8_t ndef_Header = 0;//ndef消息开始标志位-用不到

    uint8_t ssd[20]={0};
    uint8_t secret[20]={0};
    uint8_t Keep_Ssd_Flag = 0;
    uint8_t Keep_Sec_Flag = 0;
    uint8_t Keep_Ssd_long = 0;
    uint8_t Keep_Sec_long = 0;


    sleep(1);

    //NFC初始化
    if(nfc_Init() != true){
        printf("nfc_Init Failed\n");
    }
    
    // 读整个数据的包头部分，读出整个数据的长度
    if(NT3HReadHeaderNfc(&ndefLen, &ndef_Header) == 0){
        printf("NT3HReadHeaderNfc Failed\n");
        return;
    }
    
    // 加上头部字节
    ndefLen += NDEF_HEADER_SIZE;
    
    //申请内存
    uint8_t *ndefBuff = (uint8_t *)malloc(ndefLen + 1);
    if(ndefBuff == NULL){
        printf("ndefBuff malloc Failed!\n");
        return;
    }
    
    //获取NFC中的数据包
    if(get_NDEFDataPackage(ndefBuff, ndefLen) != 0){
        printf("get_NDEFDataPackage Failed\n");
        free(ndefBuff);
        return;
    }
    
    //将数据包中以","为分割线标记记号
    for(size_t i = 0; i < ndefLen; i++){
        if(0x6e == ndefBuff[i]){
            Keep_Ssd_Flag = i + 1;
        }
        if(0x2c == ndefBuff[i]){
            Keep_Sec_Flag = i + 1;
            break;
        }
    }
    
     //提取账号，密码长度
     Keep_Ssd_long = Keep_Sec_Flag - Keep_Ssd_Flag - 1;
     Keep_Sec_long = ndefLen - Keep_Sec_Flag;
     
     //将账号存入数组
     for(size_t i = 0; i < Keep_Ssd_long; i++){
         ssd[i] = ndefBuff[i + Keep_Ssd_Flag];
     }
     
     //将密码存入数组
     for(size_t i = 0; i < Keep_Sec_long; i++){
         secret[i] = ndefBuff[i + Keep_Sec_Flag];
     }
     
     free(ndefBuff);


    printf("\n");

    printf("Keep_Ssd_Flag:%s\n",ssd);
    printf("Keep_Sec_Flag:%s\n",secret);

    /* ---- 消息队列循环: 串口打印消息队列信息, 与Task1交替 ---- */
    while(1){
        osMutexAcquire(g_mutexId, osWaitForever);       /* 获取互斥锁, 与Task1交替 */
        printf("[Task3] Message Queue Task is Running!\n");

        /* 消息队列发送: 发送NFC读取的账号 */
        osStatus_t status;
        uint8_t msg_buf[20] = {0};
        memcpy(msg_buf, ssd, sizeof(ssd) > 20 ? 20 : sizeof(ssd));
        status = osMessageQueuePut(g_msgQueueId, msg_buf, 0U, 0U);
        if(status == osOK){
            printf("[Task3][MsgQueue][Send] Message: \"%s\"\n", msg_buf);
        }else{
            printf("[Task3][MsgQueue][Send] Failed! status=%d\n", status);
        }

        /* 消息队列接收 */
        uint8_t msg_recv[20] = {0};
        status = osMessageQueueGet(g_msgQueueId, msg_recv, NULL, 0U);
        if(status == osOK){
            printf("[Task3][MsgQueue][Recv] Message: \"%s\"\n", msg_recv);
        }else{
            printf("[Task3][MsgQueue][Recv] Failed! status=%d\n", status);
        }

        /* 串口打印消息队列信息 */
        printf("[Task3][UART] == Message Queue Info ==\n");
        printf("[Task3][UART] Capacity: %d\n", MSGQUEUE_OBJECTS);
        printf("[Task3][UART] NFC SSID: %s\n", ssd);
        printf("[Task3][UART] NFC Secret: %s\n", secret);
        printf("[Task3][UART] ==========================\n");

        osMutexRelease(g_mutexId);                      /* 释放互斥锁 */
        sleep(3);
    }
}

//函数功能说明：创建三个任务函数
static void Experiment_Stage2(void)
{
    osThreadAttr_t attr;
    attr.attr_bits = 
0U
;
    attr.cb_mem = 
NULL
;
    attr.cb_size = 
0U
;
    attr.stack_mem = 
NULL
;
    attr.stack_size = 
1024 * 4
;

    attr.name = 
"Stage2_Task1"
;
    attr.priority = 
25
;
    if(osThreadNew((osThreadFunc_t)Stage2_Task1,NULL,&attr) == NULL
){
        printf("Falied to create Stage2_Task1!\n"
);
    }

    attr.name = 
"Stage2_Task2"
;
    attr.priority = 
25
;
    if(osThreadNew((osThreadFunc_t)Stage2_Task2,NULL,&attr) == NULL
){
        printf("Falied to create Stage2_Task2!\n"
);
    }

    attr.name = 
"Stage2_Task3"
;
    attr.priority = 
25
;
    if(osThreadNew((osThreadFunc_t)Stage2_Task3,NULL,&attr) == NULL
){
        printf("Falied to create Stage2_Task3!\n"
);
    }

    sem1 = osSemaphoreNew(
4,0,NULL);//创建信号量初始值为0，最大值为4
    if(sem1 == NULL
){
        printf("Falied to create Semaphore1!\n"
);
    }

    g_mutexId = osMutexNew(NULL);                                         /* 创建互斥锁 Task1/Task3交替 */
    if(g_mutexId == NULL){
        printf("Falied to create Mutex!\n");
    }

    g_msgQueueId = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint8_t[20]), NULL);  /* 创建消息队列 */
    if(g_msgQueueId == NULL){
        printf("Falied to create Message Queue!\n");
    }
}

APP_FEATURE_INIT(Experiment_Stage2);