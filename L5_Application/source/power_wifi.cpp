#include <stdio.h>
#include <stdlib.h>
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
#define WIFI_CMD_SCAN            5
#define WIFI_CMD_MOVE            6
#define WIFI_CMD_BROADCAST       255

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
#define ADC_PORT                3
#define ADC_AVERAGE_DEPTH       10000

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

static void wifi_master_broadcast(void *p)
{
    char pkg[WIFI_DATA_MAX];

    pkg[0] = WIFI_CMD_BROADCAST;
    wireless_send(MESH_BROADCAST_ADDR, mesh_pkt_ack, pkg, 1, 0);
}

static void wifi_slave_heartbeat(void *p)
{
    char pkg[WIFI_DATA_MAX];
    unsigned short adc = 0;
    int i = 0;

    error = busy;

    for (i = 0; i < ADC_AVERAGE_DEPTH; i++)
        adc += adc0_get_reading(ADC_PORT);
    adc /= ADC_AVERAGE_DEPTH;
    pr_debug("before sending adc = %d\n", adc);
    i = 0;
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
    int32_t steps;
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
            wifi_slave_heartbeat(NULL);
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
            pr_debug("%d mv", (pkt->data[WIFI_STATUS_IDX_ADCU] << 8 |
                    pkt->data[WIFI_STATUS_IDX_ADCL]) * 3300 / 4096);
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
            steps = cmd << 24 | pkt->data[WIFI_MOVE_IDX_PARAM1] << 16 |
                    pkt->data[WIFI_MOVE_IDX_PARAM2] << 8;
            if (!xQueueSend(motion_queue, &steps, 1000))
                pr_err("failed to pass MOVE cmd to next layer\n");
            break;
        case WIFI_CMD_SCAN:
            steps = cmd << 24;
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
    unsigned int adc, i;
    while (1) {
        adc = 0;
        for (i = 0; i < ADC_AVERAGE_DEPTH; i++) {
            adc += adc0_get_reading(3);
        }
        adc /= ADC_AVERAGE_DEPTH;
        printf("before sending adc = %d (%dmv)\n", adc, adc * 3300 / 4096);
        //vTaskDelay(900);
    }
    while (mesh_get_node_address() != WIFI_MASTER_ADDR) {
        if (!xSemaphoreTake(signalSlaveHeartbeat, portMAX_DELAY))
            continue;;
        wifi_slave_heartbeat(NULL);
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

#define SPEED_MS  5  // 5ms per toggle, 2 toggles per step, 200 steps/rev,  100 steps/s, 0.5 rev/s, 180 deg/s
#define STEPS_FULL_REV 400
#define ENERGY_SAMPLES (STEPS_FULL_REV/2) // each 2 STEPS_FULL_REV = 1 step
#define STEPS_PER_REV 200
#define DRIVE_ON false
#define DRIVE_OFF true

#define CW true
#define CCW false

enum commandType
{
    scan,
    set_speed,
    move,
    test_rotate_pos,
    test_rotate_neg,
    none
};

static uint16_t current_pos = 0;
static int16_t steps_todo = 0;
static int8_t steps_todo2 = 0;
static uint16_t current_speed = SPEED_MS;
static double energyArray[ENERGY_SAMPLES];
static commandType commandSequence[10];
static uint8_t energyArray_idx = 0;
static uint8_t busy_bit = 0;
static commandType command = none;
static uint8_t command_idx = 0;

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
    {
        LPC_GPIO2->FIOCLR = STEP_PIN;
    }
    else
    {
        LPC_GPIO2->FIOSET = STEP_PIN;
        if ( LPC_GPIO2->FIOSET & DIRECTION_PIN)
        {
            current_pos = (current_pos + 1) % STEPS_PER_REV;
        }
        else
        {
            if (current_pos > 0)
                current_pos--;
            else
                current_pos = STEPS_PER_REV - 1;
        }
    }
}

static uint8_t get_max_energy_pos(void)
{
    uint8_t max_sample_idx = 0;
    uint8_t cur_sample_idx = 0;

    while (cur_sample_idx < ENERGY_SAMPLES)
    {
        if (energyArray[cur_sample_idx] > energyArray[max_sample_idx])
        {
            max_sample_idx = cur_sample_idx;
        }
        cur_sample_idx++;
    }
    printf("max energy pos = %d \n", max_sample_idx);
    return max_sample_idx;
}

static void setDirection(bool direction)
{
    if (direction)
        LPC_GPIO2->FIOSET = DIRECTION_PIN;
    else
        LPC_GPIO2->FIOCLR = DIRECTION_PIN;
}

#if 0
static void setStep(bool state)
{
    if (state)
        LPC_GPIO2->FIOSET = STEP_PIN;
    else
        LPC_GPIO2->FIOCLR = STEP_PIN;
}
#endif

static void motion_task(void *p)
{
    uint8_t adc_sample_flag;
    uint32_t rx;

    while (1) {
        if (!xQueueReceive(motion_queue, &rx, 1000))
            continue;
        pr_debug("recevied %lx\n", rx);

        switch (CMD_UNPACK(rx)) {
            case WIFI_CMD_SCAN:
                // set busy bit
                busy_bit = 1;
                adc_sample_flag = 1;
                printf("SCANNING \n");
                steps_todo = STEPS_FULL_REV;
                energyArray_idx = 0;
                enableDrive(DRIVE_ON);
                setDirection(CCW);
                while (steps_todo > 0) {
                    if (adc_sample_flag) {
                        unsigned int adc = 0, i;
                        for (i = 0; i < ADC_AVERAGE_DEPTH; i++)
                            adc += adc0_get_reading(ADC_PORT);
                        energyArray[energyArray_idx++] = adc / ADC_AVERAGE_DEPTH;
                        printf(" ADC sample: %d \n", energyArray_idx);
                    }
                    adc_sample_flag ^= 1;
                    toggleStep();
                    steps_todo--;
                    vTaskDelay(current_speed);
                }

                steps_todo = get_max_energy_pos() - current_pos;
                pr_debug("currentPos = %d, steps_todo = %d\n", current_pos, steps_todo);

                vTaskDelay(10000);
                setDirection(steps_todo > 0 ? CW : CCW);
                while (steps_todo > 0) {
                    toggleStep();
                    steps_todo--;
                    vTaskDelay(current_speed);
                }
                printf("Scan ended at position: %d \n", current_pos);
                busy_bit = 0;
                break;
            case WIFI_CMD_MOVE:
                // set busy bit
                busy_bit = 1;
                steps_todo2 = PARAM1_UNPACK(rx);
                printf("MOVING %d STEPS \n", steps_todo2);
                if (steps_todo2 > 0)
                    setDirection(CW);
                else
                    setDirection(CCW);
                steps_todo2 = abs(steps_todo2);
                while (steps_todo2 > 0)
                {
                    toggleStep();
                    steps_todo2--;
                    vTaskDelay(current_speed);
                }
                busy_bit = 0;
                break;
            default:
                break;
        }

        printf(" testing commands \n");
        if (busy_bit == 0)
            command = commandSequence[command_idx++];
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

    // test scan
    commandSequence[0] = scan;
    commandSequence[1] = scan;

    signalSlaveHeartbeat = xSemaphoreCreateBinary();

    comm_queue = xQueueCreate(10, sizeof(mesh_packet_t));
    motion_queue = xQueueCreate(10, sizeof(int32_t));

    xTaskCreate(wifi_receive_task, "wifi_receive", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(wifi_slave_heartbeat_task, "wifi_slave_heartbeat", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(wifi_master_broadcast, "wifi_master_broadcast", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(mid_comm_task, "mid_comm_task", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);
    xTaskCreate(motion_task, "motion_task", STACK_BYTES(2048), 0, PRIORITY_MEDIUM, NULL);

    if (!wireless_send(WIFI_MASTER_ADDR, mesh_pkt_ack, &cmd, sizeof(cmd), 0))
        pr_err("failed to send REQPWR\n");

    pr_info("initialized\n");
}
