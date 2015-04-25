#include <stdio.h>
#include "io.hpp"
#include "wireless.h"
#include "adc0.h"

/**
 * Wifi Data Package Structure
 * || 1 byte   | length - 1 ||
 * || commands |    Data    ||
 */

#define WIFI_CMD_REQPWR          0
#define WIFI_CMD_GET_STATUS      1
#define WIFI_CMD_GIVE_STATUS     2
#define WIFI_CMD_CTL_DIR         3
#define WIFI_CMD_TERMINATE       4
#define WIFI_CMD_IDLE            255

#define WIFI_DATA_MAX            256

#define WIFI_MASTER_ADDR         100

/**
 * Status Package Structure
 *
 * ||   1 byte   |      1 byte      |      1 byte      |     x byte     ||
 * || Error byte | ADC upper 4 bits | ADC lower 8 bits | Motor position ||
 */
#define WIFI_STATUS_IDX_ERR     1
#define WIFI_STATUS_IDX_ADCU    2
#define WIFI_STATUS_IDX_ADCL    3
#define WIFI_STATUS_IDX_MPOS    4
#define ADC_PORT                2

static char position = 0;
static unsigned char error = 0;
static SemaphoreHandle_t signalSlaveHeartbeat;

static void wifi_slave_heartbeat()
{
    char pkg[WIFI_DATA_MAX];
    unsigned short adc;
    int i = 0;

    adc = adc0_get_reading(ADC_PORT);
    printf("Slave: before sending adc = %d\n", adc);
    pkg[i++] = WIFI_CMD_GIVE_STATUS;
    pkg[i++] = error;
    pkg[i++] = (adc >> 8) & 0xf;
    pkg[i++] = adc & 0xff;
    pkg[i++] = position;
    wireless_send(WIFI_MASTER_ADDR, mesh_pkt_ack, pkg, i, 0);
}

static int wifi_pkt_decoding(mesh_packet_t *pkt)
{
    char len = pkt->info.data_len;
    char cmd = pkt->data[0];
    char pkg[WIFI_DATA_MAX];
    int i = 0;


    if (cmd != WIFI_CMD_IDLE) {
        if (mesh_get_node_address() == WIFI_MASTER_ADDR)
            printf("---Master: got cmd %x\n", cmd);
        else
            printf("---Slave: got cmd %x\n", cmd);
    }

    switch (cmd) {
        case WIFI_CMD_REQPWR:
            /* Master: Slave is requesting power */
            if (mesh_get_node_address() != WIFI_MASTER_ADDR)
                break;
            pkg[i++] = WIFI_CMD_GET_STATUS;
            if (!wireless_send(pkt->nwk.src, mesh_pkt_ack, pkg, i, 0))
                printf("Master: Failed to reply REQPWR\n");;
            break;
        case WIFI_CMD_GET_STATUS:
            /* Slave: Master is asking my status */
            if (mesh_get_node_address() == WIFI_MASTER_ADDR)
                break;
            wifi_slave_heartbeat();
            //xSemaphoreGive(signalSlaveHeartbeat);
            break;
        case WIFI_CMD_GIVE_STATUS:
            /* Master: Slave is giving its status */
            if (mesh_get_node_address() != WIFI_MASTER_ADDR)
                break;
            if (!len) {
                printf("CMD_GIVE_STATUS: No status received!\n");
                return cmd;
            }
            printf("Master: got ADC val: %d\n",
                    pkt->data[WIFI_STATUS_IDX_ADCU] << 8 |
                    pkt->data[WIFI_STATUS_IDX_ADCL]);
            break;
        case WIFI_CMD_CTL_DIR:
            /* Slave: Master is controlling my direction */
            if (mesh_get_node_address() == WIFI_MASTER_ADDR)
                break;
            break;
        case WIFI_CMD_TERMINATE:
            /* Slave: Master is terminating the power */
            if (mesh_get_node_address() == WIFI_MASTER_ADDR)
                break;
            break;
        case WIFI_CMD_IDLE:
            break;
        default:
            printf("Undefined wireless commands: 0x%x\n", cmd);
            break;
    }

    return 0;
}

static void wifi_receive_task(void *p)
{
    mesh_packet_t pkt;

    while (1) {
        if (!wireless_get_rx_pkt(&pkt, 1000))//portMAX_DELAY))
            continue;
        if (wifi_pkt_decoding(&pkt))
            printf("Failed to decode wireless packet.\n");
    }
}

static void wifi_slave_heartbeat_task(void *p)
{
    while (mesh_get_node_address() != WIFI_MASTER_ADDR) {
        if (!xSemaphoreTake(signalSlaveHeartbeat, portMAX_DELAY))
            continue;;
        wifi_slave_heartbeat();
        vTaskDelay(2000);
    }

    while(1);
}

void power_wifi_init()
{
    char cmd = WIFI_CMD_REQPWR;

    /* Select ADC0.3 pin-select functionality */
    LPC_PINCON->PINSEL1 &= ~(0x3 << 20);
    LPC_PINCON->PINSEL1 |= 0x1 << 20;

    signalSlaveHeartbeat = xSemaphoreCreateBinary();

    xTaskCreate(wifi_receive_task, "wifi_receive", STACK_BYTES(2048), 0, PRIORITY_LOW, NULL);
    xTaskCreate(wifi_slave_heartbeat_task, "wifi_slave_heartbeat", STACK_BYTES(2048), 0, PRIORITY_HIGH, NULL);

    if (!wireless_send(WIFI_MASTER_ADDR, mesh_pkt_ack, &cmd, sizeof(cmd), 0))
        printf("Slave: Failed to send REQPWR\n");
}
