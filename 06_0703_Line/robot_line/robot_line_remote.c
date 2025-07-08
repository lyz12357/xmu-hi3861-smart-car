#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifi_device.h"
#include "lwip/netifapi.h"
#include "lwip/api_shell.h"
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/sockets.h"
#include "hi_wifi_api.h"
#include "lwip/ip_addr.h"
#include "iot_watchdog.h"
#include "hi_watchdog.h"
#include "cJSON.h"

char recvline[1024];

// 修复函数参数：应该是 char* 而不是 char
void cotrl_handle(char *recvline, int ret)
{
    cJSON *recvjson;
    //进行json解析
    recvjson = cJSON_Parse(recvline);

    if(recvjson != NULL)
    {
        cJSON *cmdItem = cJSON_GetObjectItem(recvjson, "cmd");
        if(cmdItem != NULL && cmdItem->valuestring != NULL)
        {
            printf("cmd : %s\r\n", cmdItem->valuestring);
            
            if(strcmp("forward", cmdItem->valuestring) == 0)
            {
                set_car_status(CAR_STATUS_FORWARD);
                printf("forward\r\n");
            }

            if(strcmp("backward", cmdItem->valuestring) == 0)
            {
                set_car_status(CAR_STATUS_BACKWARD);
                printf("backward\r\n");
            }

            if(strcmp("left", cmdItem->valuestring) == 0)
            {
                set_car_status(CAR_STATUS_LEFT);
                printf("left\r\n");
            }

            if(strcmp("right", cmdItem->valuestring) == 0)
            {
                set_car_status(CAR_STATUS_RIGHT);
                printf("right\r\n");
            }

            if(strcmp("stop", cmdItem->valuestring) == 0)
            {
                set_car_status(CAR_STATUS_STOP);
                printf("stop\r\n");
            }
        }
        
        cJSON *modeItem = cJSON_GetObjectItem(recvjson, "mode");
        if(modeItem != NULL && modeItem->valuestring != NULL)
        {
            if(strcmp("step", modeItem->valuestring) == 0)
            {
                set_car_mode(CAR_MODE_STEP);
                printf("mode step\r\n");
            }

            if(strcmp("alway", modeItem->valuestring) == 0)
            {
                set_car_mode(CAR_MODE_ALWAY);
                printf("mode alway\r\n");
            }
        }

        cJSON_Delete(recvjson);
    }
}

void udp_thread(void *pdata)
{
    int ret;
    struct sockaddr_in servaddr;

    pdata = pdata;

    // 等待网络配置完成
    printf("udp_thread started, waiting for network...\r\n");
    osDelay(3000); // 等待3秒

    int sockfd = socket(PF_INET, SOCK_DGRAM, 0);
 
    //服务器 ip port
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(50001);

    // 多次尝试获取有效IP
    for(int i = 0; i < 10; i++) {
        struct netif *netif = netif_default;
        if(netif != NULL) {
            u32_t ip = ip4_addr_get_u32(ip_2_ip4(&netif->ip_addr));
            if(ip != 0 && ip != 0xFFFFFFFF) {
                printf("Car IP: %s\r\n", ipaddr_ntoa(&netif->ip_addr));
                printf("Netmask: %s\r\n", ipaddr_ntoa(&netif->netmask));
                printf("Gateway: %s\r\n", ipaddr_ntoa(&netif->gw));
                break;
            }
        }
        printf("Waiting for valid IP... attempt %d\r\n", i+1);
        osDelay(1000);
    }

    if(bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        printf("Bind failed!\r\n");
        return;
    }
    
    printf("UDP server listening on port 50001\r\n");
    
    while(1)
    {
        struct sockaddr_in addrClient;
        int sizeClientAddr = sizeof(struct sockaddr_in);

        memset(recvline, 0, sizeof(recvline));  // 修正参数顺序
        ret = recvfrom(sockfd, recvline, 1024, 0, (struct sockaddr*)&addrClient,(socklen_t*)&sizeClientAddr);
        
        if(ret>0)
        {
            char *pClientIP = inet_ntoa(addrClient.sin_addr);
 
            printf("%s-%d(%d) says:%s\n",pClientIP,ntohs(addrClient.sin_port),addrClient.sin_port, recvline);

            cotrl_handle(recvline, ret); 
        }
    }
}

void start_udp_thread(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "wifi_config_thread";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = 36;

    if (osThreadNew((osThreadFunc_t)udp_thread, NULL, &attr) == NULL) {
        printf("[LedExample] Falied to create LedTask!\n");
    }
}