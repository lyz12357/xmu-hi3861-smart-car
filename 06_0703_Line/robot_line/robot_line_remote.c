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


#include "hi_wifi_api.h"
//#include "wifi_sta.h"
#include "lwip/ip_addr.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"

#include "MQTTClient.h"

// 通过查询机器人板硬件原理图可知，机器人板上电机驱动芯片使用的是L9110S。此器件能够驱动一个直流无刷电机，集成了电机正转/反转/停止/刹车四个功能。
// 左右两轮电机各由一个L9110S驱动,GPOI0和GPIO1控制左轮,GPIO9和GPIO10控制右轮。通过输入GPIO的电平高低控制车轮正转/反转/停止/刹车
#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIOFUNC 0
#define PWMFUNC 5

#define APP_INIT_VAP_NUM    2
#define APP_INIT_USR_NUM    2

int wifi_ok_flg = 0;
static struct netif *g_lwip_netif = NULL;

static MQTTClient mq_client;
unsigned char *onenet_mqtt_buf;
unsigned char *onenet_mqtt_readbuf;
int buf_size;

short W_speed = 1;
const short freq = 8000;

typedef enum
{
    car_st,
    car_fw,
    car_bw,
    car_lt,
    car_rt
}car_status_enum;

car_status_enum car_status = car_st;

Network n;
MQTTPacket_connectData data = MQTTPacket_connectData_initializer; 

/* clear netif's ip, gateway and netmask */
void hi_sta_reset_addr(struct netif *pst_lwip_netif)
{
    ip4_addr_t st_gw;
    ip4_addr_t st_ipaddr;
    ip4_addr_t st_netmask;

    if (pst_lwip_netif == NULL) {
        printf("hisi_reset_addr::Null param of netdev\r\n");
        return;
    }

    IP4_ADDR(&st_gw, 0, 0, 0, 0);
    IP4_ADDR(&st_ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&st_netmask, 0, 0, 0, 0);

    netifapi_netif_set_addr(pst_lwip_netif, &st_ipaddr, &st_netmask, &st_gw);
}

void wifi_wpa_event_cb(const hi_wifi_event *hisi_event)
{
    if (hisi_event == NULL)
        return;

    switch (hisi_event->event) {
        case HI_WIFI_EVT_SCAN_DONE:
            printf("WiFi: Scan results available\n");
            break;
        case HI_WIFI_EVT_CONNECTED:
            printf("WiFi: Connected\n");
            netifapi_dhcp_start(g_lwip_netif);
            wifi_ok_flg = 1;
            break;
        case HI_WIFI_EVT_DISCONNECTED:
            printf("WiFi: Disconnected\n");
            netifapi_dhcp_stop(g_lwip_netif);
            hi_sta_reset_addr(g_lwip_netif);
            break;
        case HI_WIFI_EVT_WPS_TIMEOUT:
            printf("WiFi: wps is timeout\n");
            break;
        default:
            break;
    }
}

int hi_wifi_start_connect(void)
{
    int ret;
    errno_t rc;
    hi_wifi_assoc_request assoc_req = {0};

    /* copy SSID to assoc_req */
    rc = memcpy_s(assoc_req.ssid, HI_WIFI_MAX_SSID_LEN + 1, "lyztest", 7); /* 9:ssid length */
    if (rc != EOK) {
        return -1;
    }

    //热点加密方式
    assoc_req.auth = HI_WIFI_SECURITY_WPA2PSK;

    /* 热点密码 */
    memcpy(assoc_req.key, "0d0007211", 9);

    ret = hi_wifi_sta_connect(&assoc_req);
    if (ret != HISI_OK) {
        return -1;
    }

    return 0;
}

int hi_wifi_start_sta(void)
{
    int ret;
    char ifname[WIFI_IFNAME_MAX_SIZE + 1] = {0};
    int len = sizeof(ifname);
    const unsigned char wifi_vap_res_num = APP_INIT_VAP_NUM;
    const unsigned char wifi_user_res_num = APP_INIT_USR_NUM;
    unsigned int  num = WIFI_SCAN_AP_LIMIT;

    //这里不需要重复进行WiFi init，因为系统启动后就自己会做WiFi init
#if 0
    printf("_______>>>>>>>>>> %s %d \r\n", __FILE__, __LINE__);
    ret = hi_wifi_init(wifi_vap_res_num, wifi_user_res_num);
    if (ret != HISI_OK) {
        return -1;
    }
#endif
    ret = hi_wifi_sta_start(ifname, &len);
    if (ret != HISI_OK) {
        return -1;
    }

    /* register call back function to receive wifi event, etc scan results event,
     * connected event, disconnected event.
     */
    ret = hi_wifi_register_event_callback(wifi_wpa_event_cb);
    if (ret != HISI_OK) {
        printf("register wifi event callback failed\n");
    }

    /* acquire netif for IP operation */
    g_lwip_netif = netifapi_netif_find(ifname);
    if (g_lwip_netif == NULL) {
        printf("%s: get netif failed\n", __FUNCTION__);
        return -1;
    }

    /* 开始扫描附件的WiFi热点 */
    ret = hi_wifi_sta_scan();
    if (ret != HISI_OK) {
        return -1;
    }

    sleep(5);   /* sleep 5s, waiting for scan result. */

    hi_wifi_ap_info *pst_results = malloc(sizeof(hi_wifi_ap_info) * WIFI_SCAN_AP_LIMIT);
    if (pst_results == NULL) {
        return -1;
    }

    //把扫描到的热点结果存储起来
    ret = hi_wifi_sta_scan_results(pst_results, &num);
    if (ret != HISI_OK) {
        free(pst_results);
        return -1;
    }

    //打印扫描到的所有热点
    for (unsigned int loop = 0; (loop < num) && (loop < WIFI_SCAN_AP_LIMIT); loop++) {
        printf("SSID: %s\n", pst_results[loop].ssid);
    }
    free(pst_results);

    /* 开始接入热点 */
    ret = hi_wifi_start_connect();
    if (ret != 0) {
        return -1;
    }
    return 0;
}

//消息回调函数
void mqtt_callback(MessageData *msg_data)
{
    size_t res_len = 0;
    uint8_t *response_buf = NULL;
    char topicname[45] = { "$crsp/" };

    LOS_ASSERT(msg_data);

    // 获取主题名称
    char* topic = (char*)msg_data->topicName->lenstring.data;
    int topic_len = msg_data->topicName->lenstring.len;

    // 获取消息名称
    char* message = (char*)msg_data->message->payload;
    int message_len = msg_data->message->payloadlen;
    
    printf("topic %.*s receive a message\r\n", topic_len, topic);
    printf("message is %.*s\r\n", message_len, message);

    if(strncmp(topic, "car_ctl_sta", topic_len) == 0){
        if (strncmp(message, "car_fw", message_len) == 0) {
            car_setspeed(W_speed, W_speed);
            car_status = car_fw;
        }
        if (strncmp(message, "car_bw", message_len) == 0) {
            car_setspeed(-W_speed, -W_speed);
            car_status = car_bw;
        }
        if (strncmp(message, "car_lt", message_len) == 0) {
            car_setspeed(-W_speed, W_speed);
            car_status = car_lt;
        }
        if (strncmp(message, "car_rt", message_len) == 0) {
            car_setspeed(W_speed, -W_speed);
            car_status = car_rt;
        }
        if (strncmp(message, "car_st", message_len) == 0) {
            car_setspeed(1, 1);
            car_status = car_st;
        }
    }
    else if(strncmp(topic, "car_ctl_spd", topic_len) == 0){
        uint8_t first_bit = (*message) - '0',
                second_bit = (*(message + 1) - '0');
        if (message_len == 2)
            W_speed = first_bit * 10 + second_bit;
        else
            W_speed = first_bit;
        switch(car_status)
        {
        case car_fw:
            car_setspeed(W_speed, W_speed);
            break;
        case car_bw:
            car_setspeed(-W_speed, -W_speed);
            break;
        case car_lt:
            car_setspeed(-W_speed, W_speed);
            break;
        case car_rt:
            car_setspeed(W_speed, -W_speed);
            break;
        case car_st:
            break;
        }
    }
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

/**
 * @brief pwm调整函数
 * @note 无需修改
 */
void pwm_control(unsigned int gpio, unsigned int pwm_ch, uint16_t duty) {
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

void RobotTask(void* parame) {
    (void)parame;

    IoTWatchDogDisable();
    car_init();
    
    //连接热点
    hi_wifi_start_sta();
    while(wifi_ok_flg == 0)
    {
        usleep(30000);
    }
    usleep(2000000);

    int rc = 0;

	NetworkInit(&n);
	rc = NetworkConnect(&n, "192.168.15.231", 1883);
	printf("%d\n",rc);

    buf_size  = 4096+1024;
    onenet_mqtt_buf = (unsigned char *) malloc(buf_size);
    onenet_mqtt_readbuf = (unsigned char *) malloc(buf_size);
    if (!(onenet_mqtt_buf && onenet_mqtt_readbuf))
    {
        printf("No memory for MQTT client buffer!");
        return -2;
    }

	MQTTClientInit(&mq_client, &n, 1000, onenet_mqtt_buf, buf_size, onenet_mqtt_readbuf, buf_size);
    MQTTStartTask(&mq_client);

    data.keepAliveInterval = 30;
    data.cleansession = 1;
	data.clientID.cstring = "ohos_hi3861";
	//data.username.cstring = "yongz";
	//data.password.cstring = "123456";
	data.cleansession = 1;
    mq_client.defaultMessageHandler = mqtt_callback;
	//连接服务器
	rc = MQTTConnect(&mq_client, &data);

    MQTTMessage message;
    message.qos = QOS1;
	message.retained = 0;

    char msg_buf[20], status_buf[16], speed_buf[4];
    
	//订阅消息，并设置回调函数
	MQTTSubscribe(&mq_client, "car_ctl_sta", 0, mqtt_callback);
    MQTTSubscribe(&mq_client, "car_ctl_spd", 0, mqtt_callback);
	while(1)
	{
        switch(car_status)
        {
        case car_fw:
            sprintf(status_buf, "%s", "FWD at spd of ");
            break;
        case car_bw:
            sprintf(status_buf, "%s", "BWD at spd of");
            break;
        case car_lt:
            sprintf(status_buf, "%s", "TLT at spd of");
            break;
        case car_rt:
            sprintf(status_buf, "%s", "TRT at spd of");
            break;
        case car_st:
            sprintf(status_buf, "%s", "BRK but set to");
            break;
        }
		//发送消息
        sprintf(speed_buf, "%d", W_speed);
        sprintf(msg_buf, "%s %s", status_buf, speed_buf);
        message.payload = (void *)msg_buf;
        message.payloadlen = strlen(msg_buf);
        if (MQTTPublish(&mq_client, "car_status", &message) < 0)
        {
            printf("MQTTPublish faild !\r\n");
        }
        hi_udelay(500000);
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