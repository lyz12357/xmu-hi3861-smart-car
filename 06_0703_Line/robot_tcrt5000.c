#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"

//查阅机器人板原理图可知
//左边的红外传感器通过GPIO11与3861芯片连接
//右边的红外传感器通过GPIO12与3861芯片连接
#define GPIO11 11
#define GPIO12 12

//获取红外传感器输出的电平高低
void get_tcrt5000_value (void) {
    IotGpioValue id_status; //声明变量id_status

    IoTGpioGetInputVal(GPIO11, &id_status);//获取GPIO11引脚的输入电平值

    //如果GPIO11输入电平值是低电平,串口打印“left black”,说明左边的红外传感器识别到了黑色（此时传感器灯熄灭）
    if (id_status == IOT_GPIO_VALUE0) {
        printf("left black\r\n");
    }
    else
    {
        printf("left white\r\n");
    }

    IoTGpioGetInputVal(GPIO12, &id_status);//获取GPIO12引脚的输入电平值

    //如果GPIO12输入电平值是低电平,串口打印“right black”,说明右边的红外传感器识别到了黑色（此时传感器灯熄灭）
    if (id_status == IOT_GPIO_VALUE0) {
        printf("right black\r\n");
    }
    else
    {
        printf("right white\r\n");
    }

}

void RobotTask(void) {

    printf("start test tcrt5000\r\n");
    
    //循环执行获取左右两个传感器值的任务,并且每次获取之间会等待2秒钟
    while (1) {
        hi_udelay(10000);
        get_tcrt5000_value();
    }
    
}

static void RobotDemo(void)
{
    osThreadAttr_t attr;

    attr.name = "RobotTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew(RobotTask, NULL, &attr) == NULL) {
        printf("[RobotDemo] Falied to create RobotTask!\n");
    }
}

//使用OpenHarmony启动恢复模块接口APP_FEATURE_INIT()启动RobotDemo业务
APP_FEATURE_INIT(RobotDemo);  
