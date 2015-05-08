#include <stdio.h>
#include "io.hpp"
#include "wireless.h"
#include "adc0.h"

#define DEBUG 1

#define pr_debug(fmt, ...) \
    do { \
        if (DEBUG) \
            printf("%s: %s: " fmt, \
                   WIFI_IS_MASTER() ? "Master" : "Slave", \
                   __func__, ##__VA_ARGS__); \
    } while (0)

#define pr_info(fmt, ...) \
    printf("%s: %s: " fmt, \
           WIFI_IS_MASTER() ? "Master" : "Slave", \
           __func__, ##__VA_ARGS__)

#define pr_err pr_info

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
#define WIFI_CMD_MOVE            5

#define WIFI_DATA_MAX            256

#define WIFI_MASTER_ADDR         100
#define WIFI_IS_MASTER()         (mesh_get_node_address() == WIFI_MASTER_ADDR)

/**
 * Status Package Structure
 *
 * ||   1 byte   |      1 byte      |      1 byte      |     x byte     ||
 * || Error byte | ADC upper 4 bits | ADC lower 8 bits | Motor position ||
 *
 * Error byte Structure
 *
 * || 8th bit  | 7th bit  | 6th bit  | 5th bit  | 4th bit  | 3rd bit  | 2nd bit  | 1st bit  ||
 * || reserved | reserved | reserved | reserved | reserved | reserved | reserved | Busy bit ||
 */
#define WIFI_STATUS_IDX_ERR     1
#define WIFI_STATUS_IDX_ADCU    2
#define WIFI_STATUS_IDX_ADCL    3
#define WIFI_STATUS_IDX_MPOS    4
#define ADC_PORT                2

/**
 * Motion Control Package Structure
 *
 * || 1 byte  |   1 byte    |   1 byte    |   1 byte    ||
 * || Command | Parameter 1 | Parameter 2 | Parameter 3 ||
 */
#define WIFI_MOVE_IDX_CMD       0
#define WIFI_MOVE_IDX_PARAM1    1
#define WIFI_MOVE_IDX_PARAM2    2
#define WIFI_MOVE_IDX_PARAM3    3

#define CMD_UNPACK(p)           (((p) >> 24) & 0xff)
#define PARAM1_UNPACK(p)        (((p) >> 16) & 0xff)
#define PARAM2_UNPACK(p)        (((p) >> 8) & 0xff)
#define PARAM3_UNPACK(p)        (((p) >> 0) & 0xff)

static char position = 0;
static unsigned char busy = 0;
static unsigned char error = 0;
static QueueHandle_t comm_queue;
static QueueHandle_t motion_queue;
static SemaphoreHandle_t signalSlaveHeartbeat;

static void wifi_slave_heartbeat()
{
    char pkg[WIFI_DATA_MAX];
    unsigned short adc;
    int i = 0;

    error = busy;
    adc = adc0_get_reading(ADC_PORT);
    pr_debug("before sending adc = %d\n", adc);
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
    int8_t steps;
    int i = 0;

    pr_debug("got cmd %x\n", cmd);

    switch (cmd) {
        case WIFI_CMD_REQPWR:
            /* Master: Slave is requesting power */
            if (mesh_get_node_address() != WIFI_MASTER_ADDR)
                break;
            pkg[i++] = WIFI_CMD_GET_STATUS;
            if (!wireless_send(pkt->nwk.src, mesh_pkt_ack, pkg, i, 0))
                pr_err("failed to reply REQPWR\n");;
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
                pr_err("no status received!\n");
                return cmd;
            }
            pr_debug("got ADC val: %d\n",
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
        case WIFI_CMD_MOVE:
            steps = pkt->data[WIFI_MOVE_IDX_PARAM1];
            if (!xQueueSend(motion_queue, &steps, 1000))
                pr_err("failed to pass MOVE cmd to next layer\n");
            break;
        default:
            pr_err("undefined wireless commands: 0x%x\n", cmd);
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
        if (!xQueueSend(comm_queue, &pkt, 1000))
            pr_err("failed to send packet to comm queue\n");
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

static void mid_comm_task(void *p)
{
    mesh_packet_t pkt;

    while (1) {
        if (!xQueueReceive(comm_queue, &pkt, 1000))
            continue;
        if (wifi_pkt_decoding(&pkt))
            pr_err("failed to decode wireless packet.\n");
    }
}

#define DIRECTION_PIN (1 << 1)
#define ENABLE_PIN    (1 << 0)
#define STEP_PIN      (1 << 2)

#define SPEED_MS  5

#define DRIVE_ON true
#define DRIVE_OFF false

#define CW true
#define CCW false

static void enableDrive(bool state)
{
    // P2.0 for ENABLE
    if (state)
        LPC_GPIO2->FIOSET = ENABLE_PIN;
    else
        LPC_GPIO2->FIOCLR = ENABLE_PIN;
}

static void toggleStep(void)
{
    if ((bool) (LPC_GPIO2->FIOPIN & STEP_PIN))
        LPC_GPIO2->FIOCLR = STEP_PIN;
    else
        LPC_GPIO2->FIOSET = STEP_PIN;
}

static void setDirection(bool direction)
{
    if (direction)
        LPC_GPIO2->FIOSET = DIRECTION_PIN;
    else
        LPC_GPIO2->FIOCLR = DIRECTION_PIN;
}

static void setStep(bool state)
{
    if (state)
        LPC_GPIO2->FIOSET = STEP_PIN;
    else
        LPC_GPIO2->FIOCLR = STEP_PIN;
}

static void motion_task(void *p)
{
    uint8_t steps;

    while (1) {
        if (!xQueueReceive(motion_queue, &steps, 1000))
            continue;
        pr_debug("%s: move %d steps\n", __func__, steps);
        busy = 1;
        enableDrive(false);
        setDirection(CCW);
        while (steps--) {
            toggleStep();
            vTaskDelay(SPEED_MS);
        }
        busy = 0;
    }
}

void power_wifi_init()
{
    char cmd = WIFI_CMD_REQPWR;

    /* Select ADC0.3 pin-select functionality */
    LPC_PINCON->PINSEL1 &= ~(0x3 << 20);
    LPC_PINCON->PINSEL1 |= 0x1 << 20;

    // set up enable,m direction, and step GPIO
    LPC_GPIO2->FIODIR |= DIRECTION_PIN + ENABLE_PIN + STEP_PIN;
    // set up pull down for all
    LPC_PINCON->PINMODE4 |= 3 + (3 << 2) + (3 << 4);
    LPC_PINCON->PINMODE_OD2 = DIRECTION_PIN + ENABLE_PIN + STEP_PIN;

    signalSlaveHeartbeat = xSemaphoreCreateBinary();

    comm_queue = xQueueCreate(20, sizeof(mesh_packet_t));
    motion_queue = xQueueCreate(10, sizeof(int8_t));

    xTaskCreate(wifi_receive_task, "wifi_receive", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(wifi_slave_heartbeat_task, "wifi_slave_heartbeat", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(mid_comm_task, "mid_comm_task", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(motion_task, "motion_task", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);

    if (!wireless_send(WIFI_MASTER_ADDR, mesh_pkt_ack, &cmd, sizeof(cmd), 0))
        pr_err("failed to send REQPWR\n");

    pr_info("initialized\n");
}
