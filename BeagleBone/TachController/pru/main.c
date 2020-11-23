//  This code stated as a TI example from pru-software-support-package/examples

#include <stdint.h>
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_ecap.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table_0.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

// Host-0 Interrupt sets bit 30 in register R31
#define HOST_INT ((uint32_t) 1 << 30)

// The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
// PRU0 uses system event 16 (To ARM) and 17 (From ARM)
// PRU1 uses system event 18 (To ARM) and 19 (From ARM)
#define TO_ARM_HOST 16
#define FROM_ARM_HOST 17

// Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
// at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
#define CHAN_NAME "rpmsg-pru"
#define CHAN_DESC "Channel 30"
#define CHAN_PORT 30

// Used to make sure the Linux drivers are ready for RPMsg communication
// Found at linux-x.y.z/include/uapi/linux/virtio_config.h
#define VIRTIO_CONFIG_S_DRIVER_OK 4

#define RPMSG_BUF_HEADER_SIZE           16
uint8_t payload[RPMSG_BUF_SIZE - RPMSG_BUF_HEADER_SIZE];

static struct pru_rpmsg_transport s_transport;

void syscfgSetup()
{
    // Allow OCP master port access by the PRU so the PRU can read external memories
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0x0;
}

void intcSetup()
{
    // Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us
    CT_INTC.SICR_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;
}

void ecapSetup()
{
    // ECAP Control Register 1
    // Clear, which gets us must of the setting we want
    CT_ECAP.ECCTL1 = 0x00;

    // All counters reset upon event, giving difference measurements
    CT_ECAP.ECCTL1_bit.CTRRST1 = 0x1;
    CT_ECAP.ECCTL1_bit.CTRRST2 = 0x1;
    CT_ECAP.ECCTL1_bit.CTRRST3 = 0x1;
    CT_ECAP.ECCTL1_bit.CTRRST4 = 0x1;

    // ECAP Control Register 2
    // Clear, which gets us all of the setting we want
    CT_ECAP.ECCTL1 = 0x00;

    // ECAP Interrupt Enable Register
    // Clear, which gets us must of the setting we want
    CT_ECAP.ECEINT = 0x00;
    
    // interrupt on Capture Event 1
    CT_ECAP.ECEINT_bit.CEVT1 = 0x1;
}

void rpmsgSetup()
{
    volatile uint8_t *status;

    // Make sure the Linux drivers are ready for RPMsg communication 
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

    // Initialize the RPMsg transport structure 
    pru_rpmsg_init(&s_transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    // Create the RPMsg channel between the PRU and ARM user space using the transport structure. 
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &s_transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
}

void main(void)
{
    volatile uint32_t gpio;
    uint16_t src, dst, len;

    syscfgSetup();
    intcSetup();
    ecapSetup();
    rpmsgSetup();

    // Use pru0_pru_r30_5 as an output i.e., 100000 or 0x0020
    gpio = 0x0020;

    while (1) 
    {
        //
        //  Check for ECAP Capture Event 1
        //
        
        if (CT_ECAP.ECFLG_bit.CEVT1 == 0x1)
        {
            //  Grab the counter, which is a difference since last capture event 1
            uint32_t dt = CT_ECAP.CAP1;

            //  Clear the interrupt

            CT_ECAP.ECCLR_bit.CEVT1 = 0x1;
            CT_ECAP.ECCLR_bit.INT = 0x1;

            //  Update message buffer with new timing
            //  TODO
        }

        //  
        //  Check for Host Interrupt Event
        //

        if (__R31 & HOST_INT) 
        {
            // Clear the host interrupt envent
            CT_INTC.SICR_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;

            // turn on led when receiving / sending
            __R30 &= gpio;

            // Receive all available messages, multiple messages can be sent per kick 
            while (pru_rpmsg_receive(&s_transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) 
            {
                // Echo the message back to the same address from which we just received 
                pru_rpmsg_send(&s_transport, dst, src, payload, len);
            }

            // turn off led when done 
            __R30 &= gpio;
        }
    }
}
