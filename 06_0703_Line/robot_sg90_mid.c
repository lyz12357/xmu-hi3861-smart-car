#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"

//查阅机器人板原理图可知，SG90舵机通过GPIO2与3861连接
//SG90舵机的控制需要MCU产生一个周期为20ms的脉冲信号，以0.5ms到2.5ms的高电平来控制舵机转动的角度
#define GPIO2 2

//输出20000微秒的脉冲信号(x微秒高电平,20000-x微秒低电平)
void set_angle( unsigned int duty) {
    IoTGpioSetDir(GPIO2, IOT_GPIO_DIR_OUT);//设置GPIO2为输出模式

    //GPIO2输出x微秒高电平
    IoTGpioSetOutputVal(GPIO2, IOT_GPIO_VALUE1);
    hi_udelay(duty);

    //GPIO2输出20000-x微秒低电平
    IoTGpioSetOutputVal(GPIO2, IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/*Turn 45 degrees to the left of the steering gear
1、依据角度与脉冲的关系，设置高电平时间为1000微秒
2、连续发送10次脉冲信号，控制舵机向左旋转45度
*/
void engine_turn_left_45(void)
{
    for (int i = 0; i <10; i++) 
    {
        set_angle(1000);
    }
}

/*Turn 90 degrees to the left of the steering gear
1、依据角度与脉冲的关系，设置高电平时间为500微秒
2、连续发送10次脉冲信号，控制舵机向左旋转90度
*/
void engine_turn_left_90(void)
{
    for (int i = 0; i <10; i++) 
    {
        set_angle(500);
    }
}

/*Turn 45 degrees to the right of the steering gear
1、依据角度与脉冲的关系，设置高电平时间为2000微秒
2、连续发送10次脉冲信号，控制舵机向右旋转45度
*/
void engine_turn_right_45(void)
{
    for (int i = 0; i <10; i++) 
    {
        set_angle(2000);
    }
}

/*Turn 90 degrees to the right of the steering gear
1、依据角度与脉冲的关系，设置高电平时间为2500微秒
2、连续发送10次脉冲信号，控制舵机向右旋转90度
*/
void engine_turn_right_90(void)
{
    for (int i = 0; i <10; i++) 
    {
        set_angle(2500);
    }
}

/*The steering gear is centered
1、依据角度与脉冲的关系，设置高电平时间为1500微秒
2、连续发送10次脉冲信号，控制舵机居中 (舵机内部有一个基准电路，产生周期为20ms，宽度为1.5ms的基准信号，该信号定义的位置为舵机转角的中间位)
*/
void regress_middle(void)
{
    for (int i = 0; i <10; i++) 
    {
        set_angle(1500);
    }
}

/*任务实现*/
void RobotTask(void* parame) {
    (void)parame;
    printf("The steering gear is centered\r\n");
    regress_middle();
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