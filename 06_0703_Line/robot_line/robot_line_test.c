/**
 * @file robot_line_test.c
 * @author lyz12357
 * @version V0.5.1
 * @date 2025.7.8
 * @brief
 * 厦门大学信息学院小学期电子设计课程-基于鸿蒙hi3861的循迹避障小车核心代码
 * @copyright
 * CopyRight (c)  2025-2027   lyz12357@qq.com   All Right Reseverd
*/
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
#include "iot_i2c.h"
#include "iot_errno.h"
#include "ssd1306.h"
#include "iot_watchdog.h"
#include "hi_watchdog.h"

// 超声波相关
#define GPIO8 8
#define GPIO7 7

// OLED
#define OLED_I2C_BAUDRATE 400*1000
#define GPIO13 13
#define GPIO14 14

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

//红外传感器预设参数值
#define TCRT_WHITE 0
#define TCRT_BLACK 1

//红外检测函数返回的结构体，包含了检测时间戳和检测量
typedef struct
{
    uint8_t LEFT;
    uint32_t LEFT_TIMESTAMP;
    uint8_t RIGHT;
    uint32_t RIGHT_TIMESTAMP;
}tcrt_rct;

//超声波检测函数返回的枚举量
typedef enum
{
    HCSR_FAR,
    HCSR_NEAR
}hscr_rct;

//用于oled屏显的小车状态枚举量
typedef enum
{
    CAR_STATUS_FORWARD,
    CAR_STATUS_BACKWARD,
    CAR_STATUS_BREAK
}car_status;

//adc按键缓冲区
hi_u16 g_adc_buf[ADC_TEST_LENGTH] = { 0 };
//adc按键状态
int key_status = KEY_EVENT_NONE;
char key_flg = 0;
//pwm频率
const uint16_t freq = 8000;
tcrt_rct tcrt_status;
hscr_rct hcsr_status = HCSR_FAR;
//定时巡线使用的时间戳变量
uint32_t start_timestamp, end_timestamp, wasted_time;
//巡线使用的通用速度变量
int8_t LW_speed, RW_speed;
//速度偏移量，用于调整巡线时直行的速度
short W_offset_speed = 0;
//任务选择量，用于选择赛道
short Line_Task = 1;

/**
 * @brief oled显示
 * @param status 小车状态枚举量
 */
void oled_show_status(car_status status)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(2, 0);
    switch (status)
    {
    case CAR_STATUS_FORWARD:
        ssd1306_DrawString("FWD", Font_16x26, White);
        break;
    case CAR_STATUS_BACKWARD:
        ssd1306_DrawString("BWD", Font_16x26, White);
        break;
    case CAR_STATUS_BREAK:
        ssd1306_DrawString("BRK", Font_16x26, White);
        break;
    }
    ssd1306_UpdateScreen();
}

// 超声波相关
void GetDistance(hscr_rct *status) {
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    //IoTWatchDogDisable();
    hi_io_set_func(GPIO8, GPIOFUNC);

    IoTGpioSetDir(GPIO8, IOT_GPIO_DIR_IN);//GPIO_8设置为输入引脚
    IoTGpioSetDir(GPIO7, IOT_GPIO_DIR_OUT);//GPIO_7设置为输出引脚

    //GPIO_7输出一个脉冲触发信号到超声波测距模块
    IoTGpioSetOutputVal(GPIO7, IOT_GPIO_VALUE1);
    hi_udelay(20);
    IoTGpioSetOutputVal(GPIO7, IOT_GPIO_VALUE0);

    //超声波测距模块接收到GPIO_7输出的脉冲触发信号后,模块输出回响信号(高电平)到GPIO_8
    while (1) {
        IoTGpioGetInputVal(GPIO8, &value);

        //测量回响信号(高电平)时间
        if ( value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }

    }
    //距离=高电平时间*0.034 / 2
    distance = time * 0.034 / 2;
    if (distance > 10.0)
        *status = HCSR_FAR;
    else
        *status = HCSR_NEAR;
}

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

    if((vlt_val > 0.25) && (vlt_val < 0.7))
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
    printf("%d\n\r", vlt_val);
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

/**
 * @brief pwm调整函数
 * @note 无需修改
 */
void pwm_control (unsigned int gpio,unsigned int pwm_ch, uint16_t duty) {
    hi_io_set_func(gpio, PWMFUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);//将PWM引脚设置为输出方向，表示该引脚将用作输出信号的发送端。
    IoTPwmStart(pwm_ch, duty, freq);//设置PWM引脚的占空比。
}

/**
 * @brief 电机速度设置函数
 * 
 * @param lspeed 左轮速度
 * @param rspeed 右轮速度
 * @return void
 * @note 速度设为[1,99]以正转，设为[-99,-1]以反转，实际由于硬件特性设置为[-40,40]时电机停转
 */
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

/******Line：小车巡线使用的函数******/

/**
 * @brief 小车传感器、电机等gpio口的初始化
 * @return 无
 * @param 无
*/
void Line_Init()
{
    // OLED init
    IoTGpioInit(GPIO13);
    IoTGpioInit(GPIO14);
    hi_io_set_func(GPIO13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(GPIO14, HI_IO_FUNC_GPIO_14_I2C0_SCL);
    IoTI2cInit(0, OLED_I2C_BAUDRATE);
    hi_udelay(20000);
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);

    //ADC init
    IoTGpioInit(GPIO5);
    hi_io_set_func(GPIO5, HI_IO_FUNC_GPIO_5_GPIO); /* uart1 rx */
    hi_gpio_set_dir(GPIO5, HI_GPIO_DIR_IN);

    //PWM init
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

/**
 * @brief 定时巡线
 * 在固定的时间内巡线，遇到障碍时自动停车并计算耗时
 * 
 * @param patrol_time_ms 巡线时间，单位毫秒
 * @param LW_fw_speed 左轮直走速度
 * @param RW_fw_speed 右轮直走速度
 * @param LW_turn_speed 压线时左轮转向速度
 * @param RW_turn_speed 压线时右轮转向速度
 * @return void
 * @note 使用定时巡线时，途中检测到的障碍物越多，定时越不准确
 * @warning 没有对速度（占空比）进行检测，当速度绝对值大于99时会溢出
 * 
 */
void Line_Patrol_Time(uint32_t patrol_time_ms, short LW_fw_speed, short RW_fw_speed, short LW_turn_speed, short RW_turn_speed)
{
    tcrt_status.LEFT_TIMESTAMP = 0;
    tcrt_status.RIGHT_TIMESTAMP = 0;
    oled_show_status(CAR_STATUS_FORWARD);
    start_timestamp = HAL_GetTick();
    end_timestamp = HAL_GetTick();
    wasted_time = 0;
    while ((end_timestamp - start_timestamp) < patrol_time_ms + wasted_time)
    {
        get_tcrt5000_value(&tcrt_status);
        GetDistance(&hcsr_status);
        if (hcsr_status == HCSR_NEAR)
        {
            uint32_t start = HAL_GetTick();
            car_setspeed(1, 1);
            oled_show_status(CAR_STATUS_BREAK);
            while (1)
            {
                hi_udelay(50000);
                GetDistance(&hcsr_status);
                if (hcsr_status == HCSR_FAR)
                    break;
            }
            oled_show_status(CAR_STATUS_FORWARD);
            uint32_t end = HAL_GetTick();
            wasted_time += (end - start + 20);
        }
        //left on line
        if (tcrt_status.LEFT == TCRT_BLACK && tcrt_status.RIGHT == TCRT_WHITE)
        {
            LW_speed = -LW_turn_speed;
            RW_speed = RW_turn_speed;
        }
        else if (tcrt_status.RIGHT == TCRT_BLACK && tcrt_status.LEFT == TCRT_WHITE)
        {
            LW_speed = LW_turn_speed;
            RW_speed = -RW_turn_speed;
        }
        else if (tcrt_status.LEFT == TCRT_BLACK && tcrt_status.RIGHT == TCRT_BLACK)
        {
            LW_speed = 1;
            RW_speed = 1;
        }
        else
        {
            LW_speed = LW_fw_speed + W_offset_speed;
            RW_speed = RW_fw_speed + W_offset_speed;
        }
        car_setspeed(LW_speed, RW_speed);
        hi_udelay(20000);
        end_timestamp = HAL_GetTick();
    }
    oled_show_status(CAR_STATUS_BREAK);
    car_setspeed(1,1);
    hi_udelay(20000);
}

/**
 * @brief 路口巡线
 * 两个传感器都检测到黑线时认为到达路口，停止巡线
 * 
 * @param LW_fw_speed 左轮直走速度
 * @param RW_fw_speed 右轮直走速度
 * @param LW_turn_speed 压线时左轮转向速度
 * @param RW_turn_speed 压线时右轮转向速度
 * @return void
 * @warning 没有对速度（占空比）进行检测，当速度绝对值大于99时会溢出
 * 
 */
void Line_Patrol_Crossing(short LW_fw_speed, short RW_fw_speed, short LW_turn_speed, short RW_turn_speed)
{
    tcrt_status.LEFT_TIMESTAMP = 0;
    tcrt_status.RIGHT_TIMESTAMP = 0;
    oled_show_status(CAR_STATUS_FORWARD);
    get_tcrt5000_value(&tcrt_status);
    while ((tcrt_status.LEFT_TIMESTAMP == 0 && tcrt_status.RIGHT_TIMESTAMP == 0) ||
            abs((int64_t)tcrt_status.LEFT_TIMESTAMP - 
                (int64_t)tcrt_status.RIGHT_TIMESTAMP) > 200)
    {
        get_tcrt5000_value(&tcrt_status);
        GetDistance(&hcsr_status);
        if (hcsr_status == HCSR_NEAR)
        {
            car_setspeed(1, 1);
            oled_show_status(CAR_STATUS_BREAK);
            while (1)
            {
                hi_udelay(50000);
                GetDistance(&hcsr_status);
                if (hcsr_status == HCSR_FAR)
                    break;
            }
            oled_show_status(CAR_STATUS_FORWARD);
        }
        //left on line
        if (tcrt_status.LEFT == TCRT_BLACK && tcrt_status.RIGHT == TCRT_WHITE)
        {
            LW_speed = -LW_turn_speed;
            RW_speed = RW_turn_speed;
        }
        else if (tcrt_status.RIGHT == TCRT_BLACK && tcrt_status.LEFT == TCRT_WHITE)
        {
            LW_speed = LW_turn_speed;
            RW_speed = -RW_turn_speed;
        }
        else if (tcrt_status.LEFT == TCRT_BLACK && tcrt_status.RIGHT == TCRT_BLACK)
        {
            LW_speed = 1;
            RW_speed = 1;
        }
        else
        {
            LW_speed = LW_fw_speed + W_offset_speed;
            RW_speed = RW_fw_speed + W_offset_speed;
        }
        car_setspeed(LW_speed, RW_speed);
        hi_udelay(20000);
    }
    oled_show_status(CAR_STATUS_BREAK);
    car_setspeed(1,1);
    hi_udelay(20000);
}

/**
 * @brief 启动电机
 * 启动电机并延时等待
 * 
 * @param time_ms 延时时间
 * @param LW_fw_speed 左轮速度
 * @param RW_fw_speed 右轮速度
 * @return void
 * @note 该函数不会停止电机，停止电机可以使用car_setspeed()
 */
void Line_Motor_start(uint32_t time_ms, short LW_fw_speed, short RW_fw_speed)
{
    car_setspeed(LW_fw_speed, RW_fw_speed);
    hi_udelay(time_ms*1000);
}

void Line_Task1()
{
    Line_Motor_start(500, 1, 1);
    Line_Motor_start(10, 77, 75);
    car_setspeed(57, 55);
    Line_Patrol_Crossing(87, 85, 77, 75);
}

void Line_Task2()
{
    Line_Motor_start(500, 1, 1);
    Line_Motor_start(10, 77, 75);
    car_setspeed(57, 55);
    Line_Patrol_Crossing(72, 70, 87, 85);
    Line_Motor_start(300, 57,55);
    Line_Patrol_Crossing(72, 70, 87, 85);
    Line_Motor_start(300, 57,55);
    Line_Patrol_Time(10500, 72, 70, 87, 85);
    Line_Patrol_Time(3000, 57, 55, 77, 75);
    Line_Patrol_Crossing(72, 70, 87, 85);
}

void Line_Task3()
{
    Line_Motor_start(500, 1, 1);
    Line_Motor_start(10, 77, 75);
    car_setspeed(57, 55);
    Line_Patrol_Crossing(59, 55, 79, 75);
    Line_Motor_start(100, 57, 55);
    Line_Motor_start(100, 77, 1);
    Line_Patrol_Crossing(67, 65, 77, 75);
    Line_Motor_start(100, 57, 55);
    Line_Motor_start(100, 1, 75);
    Line_Patrol_Time(6000, 57, 70, 77, 90);
    Line_Patrol_Time(4000, 57, 55, 77, 75);
    Line_Patrol_Time(3500, 72, 70, 87, 85);
    Line_Patrol_Time(2000, 57, 55, 77, 75);
    Line_Patrol_Time(8000, 62, 60, 77, 75);
    Line_Patrol_Time(5000, 72, 50, 82, 70);
    Line_Patrol_Crossing(72, 70, 87, 85);
}

void RobotTask(void* parame) {
    (void)parame;

    IoTWatchDogDisable();

    Line_Init();

    // 按键设置 - Line Task
    char task_buf[2];
    sprintf(&task_buf, "%d", Line_Task);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(2,0);
    ssd1306_DrawString("Task", Font_16x26, White);
    ssd1306_SetCursor(2, 27);
    ssd1306_DrawString(&task_buf, Font_16x26, White);
    ssd1306_UpdateScreen();
    while (1)
    {
        app_demo_adc_test();
        int ret;
        ret = get_key_event();
        if (ret == KEY_EVENT_S3)
        {
            ssd1306_Fill(Black);
            ssd1306_SetCursor(2,0);
            ssd1306_UpdateScreen();
            break;
        }
        if (ret == KEY_EVENT_S1)
            Line_Task--;
        if (ret == KEY_EVENT_S2)
            Line_Task++;
        if (Line_Task > 3)
            Line_Task = 1;
        if (Line_Task < 1)
            Line_Task = 3;
        sprintf(&task_buf, "  ");
        ssd1306_SetCursor(2, 27);
        ssd1306_DrawString(&task_buf, Font_16x26, White);
        ssd1306_UpdateScreen();
        sprintf(&task_buf, "%d", Line_Task);
        ssd1306_SetCursor(2, 27);
        ssd1306_DrawString(&task_buf, Font_16x26, White);
        ssd1306_UpdateScreen();
        hi_udelay(10000);
    }

    //按键设置 - V offset
    char w_offset_speed_buf[3];
    sprintf(&w_offset_speed_buf, "%d", W_offset_speed);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(2,0);
    ssd1306_DrawString("V_off", Font_16x26, White);
    ssd1306_SetCursor(2, 27);
    ssd1306_DrawString(&w_offset_speed_buf, Font_16x26, White);
    ssd1306_UpdateScreen();
    while (1)
    {
        app_demo_adc_test();
        int ret;
        ret = get_key_event();
        if (ret == KEY_EVENT_S3)
        {
            ssd1306_Fill(Black);
            ssd1306_SetCursor(2,0);
            ssd1306_UpdateScreen();
            break;
        }
        if (ret == KEY_EVENT_S1)
            W_offset_speed--;
        if (ret == KEY_EVENT_S2)
            W_offset_speed++;
        if (W_offset_speed > 5)
            W_offset_speed = 5;
        if (W_offset_speed < -5)
            W_offset_speed = -5;
        sprintf(&w_offset_speed_buf, "  ");
        ssd1306_SetCursor(2, 27);
        ssd1306_DrawString(&w_offset_speed_buf, Font_16x26, White);
        ssd1306_UpdateScreen();
        sprintf(&w_offset_speed_buf, "%d", W_offset_speed);
        ssd1306_SetCursor(2, 27);
        ssd1306_DrawString(&w_offset_speed_buf, Font_16x26, White);
        ssd1306_UpdateScreen();
        hi_udelay(10000);
    }

    switch (Line_Task)
    {
    case 1:
        Line_Task1();
        break;
    case 2:
        Line_Task2();
        break;
    case 3:
        Line_Task3();
        break;
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
    attr.stack_size = 40960;
    attr.priority = osPriorityNormal;

    if (osThreadNew(RobotTask, NULL, &attr) == NULL) {
        printf("[RobotDemo] Falied to create RobotTask!\n");
    }
}

//使用OpenHarmony启动恢复模块接口APP_FEATURE_INIT()启动业务
APP_FEATURE_INIT(RobotDemo);
