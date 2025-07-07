#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "iot_pwm.h"
#include "hi_pwm.h"

#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIOFUNC 0

// 通过查询机器人板硬件原理图可知，机器人板上电机驱动芯片使用的是L9110S。此器件能够驱动一个直流无刷电机，集成了电机正转/反转/停止/刹车四个功能。
// 左右两轮电机各由一个L9110S驱动,GPOI0和GPIO1控制左轮,GPIO9和GPIO10控制右轮。通过输入GPIO的电平高低控制车轮正转/反转/停止/刹车

// gpio_control用于控制GPIO引脚的输出电平值
void gpio_control (unsigned int  gpio, IotGpioValue value) {
    hi_io_set_func(gpio, GPIOFUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);//将GPIO引脚设置为输出方向，表示该引脚将用作输出信号的发送端。在输出模式下，可以通过控制GPIO引脚的输出电平值
    IoTGpioSetOutputVal(gpio, value);//设置GPIO引脚的输出电平值。
}

// 小车后退
// GPIO0为低电平，GPIO1为高电平，左轮反转
// GPIO9为低电平，GPIO10为高电平，右轮反转
void car_backward(void) {
    gpio_control(GPIO0, IOT_GPIO_VALUE0);
    gpio_control(GPIO1, IOT_GPIO_VALUE1);
    gpio_control(GPIO9, IOT_GPIO_VALUE0);
    gpio_control(GPIO10, IOT_GPIO_VALUE1);
}

// 小车前进
// GPIO0为高电平，GPIO1为低电平，左轮正转
// GPIO9为高电平，GPIO10为低电平，右轮正转
void car_forward(void) {
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);
}

// 小车左转
// GPIO0和1为低电平,左轮停止
// GPIO9为高电平，GPIO10为低电平，右轮正转
void car_left(void) {
    gpio_control(GPIO0, IOT_GPIO_VALUE0);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);
}

// 小车右转
// GPIO0为高电平，GPIO1为低电平，左轮正转
// GPIO9和10为低电平,右轮停止
void car_right(void) {
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE0);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);
}

// 小车停止
// GPIO0和1为高电平,左轮刹车
// GPIO9和10为高电平,右边轮刹车
void car_stop(void) {
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE1);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE1);
}

void RobotTask(void* parame) {
    (void)parame;
    printf("start test l9110s\r\n");
    car_forward();
    osDelay(500);
    car_backward();
    osDelay(500);
    car_left();
    osDelay(500);
    car_right();
    osDelay(500);
    car_stop();
    osDelay(500);
}

//新建业务入口函数RobotDemo
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

//使用OpenHarmony启动恢复模块接口APP_FEATURE_INIT()启动业务
APP_FEATURE_INIT(RobotDemo);  