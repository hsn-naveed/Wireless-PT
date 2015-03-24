#include "io.hpp"
#include "queue.h"

typedef enum {
   shared_SensorQueueId,
} sharedHandleId_t;

/// Orientation type enumeration
typedef enum {
    invalid,
    up,
    down,
    left,
    right,
} orientation_t;

static char orientation_c[5][8] = {
        "invalid", "up", "down", "left", "right",
};

class orient_compute : public scheduler_task
{
    public:
        orient_compute(uint8_t priority) : scheduler_task("compute", 2048, priority)
        {
            /* We save the queue handle by using addSharedObject() */
            qid = xQueueCreate(1, sizeof(orientation_t));
            addSharedObject(shared_SensorQueueId, qid);
        }

        bool run(void *p)
        {
            /* Compute orientation here, and send it to the queue once a second */
            orientation_t orientation = invalid;
            if (AS.getZ() > 1000)
                orientation = up;
            else if (AS.getZ() < -1000)
                orientation = down;
            else if (AS.getY() + AS.getY() < -1000)
                orientation = left;
            else if (AS.getY() + AS.getY() > 1000)
                orientation = right;
            if (orientation) {
                printf("----------------------------------------\n");
                printf("Task compute: Before sending orientation\n");
                xQueueSend(qid, &orientation, portMAX_DELAY);
                printf("Task compute: After sending orientation\n");
            }
            vTaskDelay(1000);
            return true;
        }
    private:
        QueueHandle_t qid;
};

class orient_process : public scheduler_task
{
    public:
        orient_process (uint8_t priority) : scheduler_task("process", 2048, priority)
        {
            qid = getSharedObject(shared_SensorQueueId);
            /* Initialize GPIO1[0] to control LED9 */
            LPC_GPIO1->FIODIR |= BITS(0);
            /* Turn off LED initially */
            LPC_GPIO1->FIOPIN |= BITS(0);
        }

        bool run(void *p)
        {
            /* We first get the queue handle the other task added using addSharedObject() */
            orientation_t orientation = invalid;

            /* Sleep the task forever until an item is available in the queue */
            if (xQueueReceive(qid, &orientation, portMAX_DELAY))
            {
                printf("Task process: received %s\n", orientation_c[orientation]);
                if (orientation == left || orientation == right)
                    LPC_GPIO1->FIOPIN &= MASK(0);
                else
                    LPC_GPIO1->FIOPIN |= BITS(0);
            }

            return true;
        }
    private:
        QueueHandle_t qid;
};
