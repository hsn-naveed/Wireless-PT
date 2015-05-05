#include <stdio.h>
#include "io.hpp"
#include "wireless.h"
#include "adc0.h"

#define DEBUG 1

#define pr_debug(fmt, ...) \
    do { \
        if (DEBUG) \
            printf(fmt, ##__VA_ARGS__); \
    } while (0)

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

    adc = adc0_get_reading(ADC_PORT);
    pr_debug("Slave: before sending adc = %d\n", adc);
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
    uint32_t tmp;
    int i = 0;

    if (mesh_get_node_address() == WIFI_MASTER_ADDR)
        pr_debug("---Master: got cmd %x\n", cmd);
    else
        pr_debug("---Slave: got cmd %x\n", cmd);

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
            pr_debug("Master: got ADC val: %d\n",
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
            tmp = pkt->data[WIFI_MOVE_IDX_CMD] << 24 |
                  pkt->data[WIFI_MOVE_IDX_PARAM1] << 16;
            if (!xQueueSend(comm_queue, &tmp, 1000))
                printf("failed to pass MOVE cmd to next layer\n");
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

static void mid_comm_task(void *p)
{
    uint32_t word;
    uint8_t steps;

    while (1) {
        if (!xQueueReceive(comm_queue, &word, 1000))
            continue;

        steps = PARAM1_UNPACK(word);
        switch (CMD_UNPACK(word)){
            case WIFI_CMD_MOVE:
                printf("sending steps %d\n", steps);
                if (busy) {
                    //TO DO FUNCTION ALREADY BUSY;
                    break;
                }
                if (!xQueueSend(motion_queue, &steps, 1000))
                    printf("The command could not be sent to motion queue\n");
                break;
            case WIFI_CMD_GET_STATUS:
                /* TODO Need to figure out a way to give feedback to wifi layer */
                break;
            default:
                break;
        }
    }
}

#define P2_0 (1 << 0)
#define P2_1 (1 << 1)
#define P2_2 (1 << 2)

#define DIRECTION_PIN P2_1
#define ENABLE_PIN    P2_0
#define STEP_PIN      P2_2

#define SPEED_MS  5

#define DRIVE_ON true
#define DRIVE_OFF false

#define CW true
#define CCW false

void enableDrive(bool state)
{
    // P2.0 for ENABLE
    if (state)
        LPC_GPIO2->FIOSET = ENABLE_PIN;
    else
        LPC_GPIO2->FIOCLR = ENABLE_PIN;
}

void toggleStep(void)
{
    if ((bool) (LPC_GPIO2->FIOPIN & STEP_PIN))
        LPC_GPIO2->FIOCLR = STEP_PIN;
    else
        LPC_GPIO2->FIOSET = STEP_PIN;
}

void setDirection(bool direction)
{
    if (direction)
        LPC_GPIO2->FIOSET = DIRECTION_PIN;
    else
        LPC_GPIO2->FIOCLR = DIRECTION_PIN;
}

void setStep(bool state)
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
        enableDrive(false);
        setDirection(CCW);
        while (steps--) {
            toggleStep();
            vTaskDelay(SPEED_MS);
        }
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

    comm_queue = xQueueCreate(20, sizeof(uint32_t));
    motion_queue = xQueueCreate(10, sizeof(uint8_t));

    xTaskCreate(wifi_receive_task, "wifi_receive", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(wifi_slave_heartbeat_task, "wifi_slave_heartbeat", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(mid_comm_task, "mid_comm_task", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(motion_task, "motion_task", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);

    if (!wireless_send(WIFI_MASTER_ADDR, mesh_pkt_ack, &cmd, sizeof(cmd), 0))
        printf("Slave: Failed to send REQPWR\n");
}
