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
#include <hi_io.h>
#include <hi_gpio.h>
#include <hi_task.h>
#include <unistd.h>
#include <hi_types_base.h>
#define APP_DEMO_ADC
#include <hi_adc.h>
#include <hi_stdlib.h>
#include <hi_early_debug.h>

// ADC key
#define KEY_EVENT_NONE      0
#define KEY_EVENT_S1      1
#define KEY_EVENT_S2      2
#define KEY_EVENT_S3      3
#define KEY_EVENT_S4      4
#define GPIO5 5
#define ADC_TEST_LENGTH  64
#define VLT_MIN 100

// 通过查询机器人板硬件原理图可知，机器人板上电机驱动芯片使用的是L9110S。此器件能够驱动一个直流无刷电机，集成了电机正转/反转/停止/刹车四个功能。
// 左右两轮电机各由一个L9110S驱动,GPOI0和GPIO1控制左轮,GPIO9和GPIO10控制右轮。通过输入GPIO的电平高低控制车轮正转/反转/停止/刹车
#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIOFUNC 0
#define PWMFUNC 5

//查阅机器人板原理图可知
//左边的红外传感器通过GPIO11与3861芯片连接
//右边的红外传感器通过GPIO12与3861芯片连接
#define GPIO11 11
#define GPIO12 12

#define TCRT_WHITE 0
#define TCRT_BLACK 1

hi_u16 g_adc_buf[ADC_TEST_LENGTH] = { 0 };
int key_status = KEY_EVENT_NONE;
char key_flg = 0;

//PWM frequence Hz
const uint16_t freq = 8000;
//left wheel base speed(duty)
const int16_t LW_base_speed = 50;
//right wheel base speed(duty)
const int16_t RW_base_speed = 50;
int16_t LW_add_speed = 0;
int16_t RW_add_speed = 0;

typedef struct
{
    uint8_t LEFT;
    uint32_t LEFT_TIMESTAMP;
    uint8_t RIGHT;
    uint32_t RIGHT_TIMESTAMP;
}tcrt_rct;

/* asic adc test  */
int get_key_event(void)
{
    int tmp = key_status;
    key_status = KEY_EVENT_NONE;
    return tmp;
}
// ADC 相关
hi_void convert_to_voltage(hi_u32 data_len)
{
    hi_u32 i;
    float vlt_max = 0;
    float vlt_min = VLT_MIN;

    float vlt_val = 0;

    hi_u16 vlt;
    for (i = 0; i < data_len; i++) {
        vlt = g_adc_buf[i];
        float voltage = (float)vlt * 1.8 * 4 / 4096.0;  /* vlt * 1.8 * 4 / 4096.0: Convert code into voltage */
        vlt_max = (voltage > vlt_max) ? voltage : vlt_max;
        vlt_min = (voltage < vlt_min) ? voltage : vlt_min;
    }
//    printf("vlt_min:%.3f, vlt_max:%.3f \n", vlt_min, vlt_max);

    vlt_val = (vlt_min + vlt_max)/2.0;

    if((vlt_val > 0.4) && (vlt_val < 0.6))
    {
        if(key_flg == 0)
        {
            key_flg = 1;
            key_status = KEY_EVENT_S1;
        }
    }
    if((vlt_val > 0.8) && (vlt_val < 1.1))
    {
        if(key_flg == 0)
        {
            key_flg = 1;
            key_status = KEY_EVENT_S2;
        }
    }

    if((vlt_val > 0.01) && (vlt_val < 0.3))
    {
        if(key_flg == 0)
        {
            key_flg = 1;
            key_status = KEY_EVENT_S3;
        }
    }

    if(vlt_val > 3.0)
    {
        key_flg = 0;
        key_status = KEY_EVENT_NONE;
    }
}
// ADC 相关
void app_demo_adc_test(void)
{
    hi_u32 ret, i;
    hi_u16 data;  /* 10 */

    memset_s(g_adc_buf, sizeof(g_adc_buf), 0x0, sizeof(g_adc_buf));
 
    for (i = 0; i < ADC_TEST_LENGTH; i++) {
        ret = hi_adc_read((hi_adc_channel_index)HI_ADC_CHANNEL_2, &data, HI_ADC_EQU_MODEL_1, HI_ADC_CUR_BAIS_DEFAULT, 0);
        if (ret != HI_ERR_SUCCESS) {
            printf("ADC Read Fail\n");
            return;
        }
        g_adc_buf[i] = data;
    }
    convert_to_voltage(ADC_TEST_LENGTH);

}
// 电机GPIO初始化
void car_init()
{
    IoTGpioInit(GPIO0);
    IoTGpioInit(GPIO1);
    IoTGpioInit(GPIO10);
    IoTGpioInit(GPIO9);

    hi_io_set_func(GPIO0, HI_IO_FUNC_GPIO_0_PWM3_OUT);
    hi_io_set_func(GPIO1,HI_IO_FUNC_GPIO_1_PWM4_OUT);
    hi_io_set_func(GPIO9,HI_IO_FUNC_GPIO_9_PWM0_OUT);
    hi_io_set_func(GPIO10,HI_IO_FUNC_GPIO_10_PWM1_OUT);

    IoTGpioSetDir(GPIO0,IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(GPIO1,IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(GPIO9,IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(GPIO10,IOT_GPIO_DIR_OUT);

    IoTPwmInit(HI_PWM_PORT_PWM0);
    IoTPwmInit(HI_PWM_PORT_PWM1);
    IoTPwmInit(HI_PWM_PORT_PWM3);
    IoTPwmInit(HI_PWM_PORT_PWM4);
}

// gpio_control用于控制GPIO引脚的输出电平值
void gpio_control (unsigned int  gpio, IotGpioValue value) {
    hi_io_set_func(gpio, GPIOFUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);//将GPIO引脚设置为输出方向，表示该引脚将用作输出信号的发送端。在输出模式下，可以通过控制GPIO引脚的输出电平值
    IoTGpioSetOutputVal(gpio, value);//设置GPIO引脚的输出电平值。
}

// pwm_control用于控制PWM引脚的输出占空比
void pwm_control (unsigned int gpio,unsigned int pwm_ch, uint16_t duty) {
    hi_io_set_func(gpio, PWMFUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);//将PWM引脚设置为输出方向，表示该引脚将用作输出信号的发送端。
    IoTPwmStart(pwm_ch, duty, freq);//设置PWM引脚的占空比。
}

//set speed to 0 to stop and 100 to break and -1~-99 to back
void car_setspeed(int16_t lspeed, int16_t rspeed)
{
    if (lspeed < 100 && lspeed > 0)
    {
        pwm_control(GPIO0, HI_PWM_PORT_PWM3, lspeed);
        pwm_control(GPIO1, HI_PWM_PORT_PWM4, 1);
    }
    else if (lspeed > -100 && lspeed < 0)
    {
        pwm_control(GPIO1, HI_PWM_PORT_PWM4, -lspeed);
        pwm_control(GPIO0, HI_PWM_PORT_PWM3, 1);
    }

    if (rspeed < 100 && rspeed > 0)
    {
        pwm_control(GPIO9, HI_PWM_PORT_PWM0, rspeed);
        pwm_control(GPIO10, HI_PWM_PORT_PWM1, 1);
    }
    else if (rspeed > -100 && rspeed < 0)
    {
        pwm_control(GPIO10, HI_PWM_PORT_PWM1, -rspeed);
        pwm_control(GPIO9, HI_PWM_PORT_PWM0, 1);
    }
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
//获取时间
uint32_t HAL_GetTick(void)
{
    uint32_t msPerTick = 1000 / osKernelGetTickFreq(); // 10ms
    uint32_t tickMs = osKernelGetTickCount() * msPerTick;

    uint32_t csPerMs = osKernelGetSysTimerFreq() / 1000; // 160K cycle/ms
    uint32_t csPerTick = csPerMs * msPerTick; // 1600K cycles/tick
    uint32_t restMs = osKernelGetSysTimerCount() % csPerTick / csPerMs;

    return tickMs + restMs;
}
//获取红外传感器输出的电平高低
void get_tcrt5000_value (tcrt_rct *status) {
    IotGpioValue id_status; //声明变量id_status

    IoTGpioGetInputVal(GPIO11, &id_status);//获取GPIO11引脚的输入电平值

    //如果GPIO11输入电平值是低电平,串口打印“left black”,说明左边的红外传感器识别到了黑色（此时传感器灯熄灭）
    if (id_status == IOT_GPIO_VALUE1) {
        // printf("left black\r\n");
        status->LEFT = TCRT_BLACK;
        status->LEFT_TIMESTAMP = HAL_GetTick();
    }
    else
    {
        // printf("left white\r\n");
        status->LEFT = TCRT_WHITE;
    }

    IoTGpioGetInputVal(GPIO12, &id_status);//获取GPIO12引脚的输入电平值

    //如果GPIO12输入电平值是低电平,串口打印“right black”,说明右边的红外传感器识别到了黑色（此时传感器灯熄灭）
    if (id_status == IOT_GPIO_VALUE1) {
        // printf("right black\r\n");
        status->RIGHT = TCRT_BLACK;
        status->RIGHT_TIMESTAMP = HAL_GetTick();
    }
    else
    {
        // printf("right white\r\n");
        status->RIGHT = TCRT_WHITE;
    }

}

void RobotTask(void* parame) {
    (void)parame;

    //ADC init
    hi_u32 ret;
    IoTGpioInit(GPIO5);
    hi_io_set_func(GPIO5, HI_IO_FUNC_GPIO_5_GPIO); /* uart1 rx */
    hi_gpio_set_dir(GPIO5, HI_GPIO_DIR_IN);

    //PWM init
    car_init();
    tcrt_rct sensor_status;
    sensor_status.LEFT_TIMESTAMP = 0;
    sensor_status.RIGHT_TIMESTAMP = 0;

    //按键设置
    while (1)
    {
        app_demo_adc_test();
        int ret;
        ret = get_key_event();
        if (ret == KEY_EVENT_S3)
        {
            break;
        }
    }
    
    car_setspeed(1, 1);
    hi_udelay(1000000);
    car_setspeed(75, 75);
    hi_udelay(10000);
    car_setspeed(LW_base_speed, RW_base_speed);
    
    //第一次路口检测
    while ((sensor_status.LEFT_TIMESTAMP == 0 && sensor_status.RIGHT_TIMESTAMP == 0) ||
            abs((int64_t)sensor_status.LEFT_TIMESTAMP - 
                (int64_t)sensor_status.RIGHT_TIMESTAMP) > 200)
    {
        get_tcrt5000_value(&sensor_status);
        // printf("L:%d R:%dd\n", sensor_status.LEFT, sensor_status.RIGHT);
        //left on line
        if (sensor_status.LEFT == TCRT_BLACK && sensor_status.RIGHT == TCRT_WHITE)
        {
            LW_add_speed = -49;
            RW_add_speed = 40;
        }
        else if (sensor_status.RIGHT == TCRT_BLACK && sensor_status.LEFT == TCRT_WHITE)
        {
            LW_add_speed = 40;
            RW_add_speed = -49;
        }
        else if (sensor_status.LEFT == TCRT_BLACK && sensor_status.RIGHT == TCRT_BLACK)
        {
            LW_add_speed = -49;
            RW_add_speed = -49;
        }
        else
        {
            LW_add_speed = 0;
            RW_add_speed = 0;
        }
        car_setspeed(LW_base_speed + LW_add_speed, RW_base_speed + RW_add_speed);
        hi_udelay(20000);
    }

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