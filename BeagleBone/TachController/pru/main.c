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

//  The output pins used to communicate status via connected LEDs

uint32_t pr1_pru1_gpo5  = 0x1 << 5;     // BBAI Header Pin P8.18
uint32_t pr1_pru1_gpo9  = 0x1 << 9;     // BBAI Header Pin P8.14
uint32_t pr1_pru1_gpo18 = 0x1 << 18;    // BBAI Header Pin P8.16

uint32_t pr1_ecap0_ecap_capin_apwm_o = 0x1 << 15;    // BBAI Header Pin P8.15

//  forward decl of internal methods

void initSysCfg();
void intcSetup();
void initECap();
void initRPMsg();
void initGPIO();

void updateRPMsg();
void updateECap();
void updateHeartbeat();

//
//  Main
//

void main(void)
{
    //  Init

    initSysCfg();
    intcSetup();
    initECap();
    //initRPMsg();
    initGPIO();

    while (1) 
    {
        //updateRPMsg();
        updateECap();
        updateHeartbeat();
    }
}

//
//  Init
//

void initSysCfg()
{
    // Allow OCP master port access by the PRU so the PRU can read external memories
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0x0;
}

void intcSetup()
{
    // Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us
    CT_INTC.SICR_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;
}

void initECap()
{
    // ECAP Control Register 1
    // Clear, which gets us most of the setting we want
    CT_ECAP.ECCTL1 = 0x00000000;

    // All counters reset upon event, giving difference measurements
    CT_ECAP.ECCTL1_bit.CTRRST1 = 0x1;
    CT_ECAP.ECCTL1_bit.CTRRST2 = 0x1;
    CT_ECAP.ECCTL1_bit.CTRRST3 = 0x1;
    CT_ECAP.ECCTL1_bit.CTRRST4 = 0x1;

    // Capture event 1 is rising edge and capture event 2 is on falling edge
    CT_ECAP.ECCTL1_bit.CAP1POL = 0x0;
    CT_ECAP.ECCTL1_bit.CAP2POL = 0x1;

    // ECAP Control Register 2
    // reset, which gets us most of the setting we want
    CT_ECAP.ECCTL2 = 0x00000000;
    CT_ECAP.ECCTL2_bit.STOPVALUE = 0x1;

    // ECAP Interrupt Enable Register
    // Clear, which gets us most of the setting we want
    CT_ECAP.ECEINT = 0x00000000;
    
    // interrupt on Capture Event 1 and Capture Event 2
    CT_ECAP.ECEINT_bit.CEVT1 = 0x1;
    CT_ECAP.ECEINT_bit.CEVT2 = 0x1;
    
    // need to set the mux mode of the pad register to select the pru eCap mode.
    // registers are named after muxmode 0.  This is the register name to our 
    // usage mapping
    //
    // ctrl_core_pad_vin2a_d2 in muxmod 11 = pr1_ecap0_ecap_capin_apwm_o;
    //

    uint32_t *ctrl_core_pad_pr1_ecap0_ecap_capin_apwm_o = (uint32_t *)0x4A003570;
    uint32_t *ctrl_core_pad_vin2a_d19 = (uint32_t *)0x4A0035B4;

    uint32_t pad_cfg_ecap = 0x0004000B; // fast slew, mode 11
    uint32_t pad_cfg_disabled = 0x0001000F; // Driver off

    //  Setup header pin P8.15 correctly for eCap input
    (*ctrl_core_pad_vin2a_d19) = pad_cfg_disabled;
    (*ctrl_core_pad_pr1_ecap0_ecap_capin_apwm_o) = pad_cfg_ecap;
}

void initRPMsg()
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

void initGPIO()
{
    // need to set the mux mode of the pad register to select the pru gpio mode.
    // registers are named after muxmode 0.  This is the register name to our 
    // usage mapping
    //
    // ctrl_core_pad_vin2a_d8 in muxmod 13 = pr1_pru1_gpio5
    // ctrl_core_pad_vin2a_d12 in muxmod 13 = pr1_pru1_gpio9
    // ctrl_core_pad_vin2a_d21 in muxmod 13 = pr1_pru1_gpio18
    //

    uint32_t *ctrl_core_pad_pr1_pru1_gpo5  = (uint32_t *)0x4A003588;
    uint32_t *ctrl_core_pad_pr1_pru1_gpo9  = (uint32_t *)0x4A003598;
    uint32_t *ctrl_core_pad_pr1_pru1_gpo18 = (uint32_t *)0x4A0035BC;

    uint32_t pad_cfg_gpo = 0x0000000D; // fast slew, pull down enabled, mode 13

    (*ctrl_core_pad_pr1_pru1_gpo5)  = pad_cfg_gpo;
    (*ctrl_core_pad_pr1_pru1_gpo9)  = pad_cfg_gpo;
    (*ctrl_core_pad_pr1_pru1_gpo18) = pad_cfg_gpo;

    // turn off all LEDs to start with
    
    __R30 &= ~(pr1_pru1_gpo5);
    __R30 &= ~(pr1_pru1_gpo9);
    __R30 &= ~(pr1_pru1_gpo18);
}

//
// Update
//


void updateECap()
{
    // Was an interupt generated
    
    if (CT_ECAP.ECFLG_bit.INT == 0x1)
    {
        if (CT_ECAP.ECFLG_bit.CEVT1 == 0x1)
        {
            // turn on the eCap trigger LED
            __R30 |= pr1_pru1_gpo18;
    
            //  Grab the counter, which is a difference since last capture event 1
            uint32_t dt = CT_ECAP.CAP1;
     
            //  Clear the interrupt
     
            CT_ECAP.ECCLR_bit.CEVT1 = 0x1;
     
            //  Update message buffer with new timing
            //  TODO
        }

        if (CT_ECAP.ECFLG_bit.CEVT2 == 0x1)
        {
            // turn 0ff the eCap trigger LED
            __R30 &= ~(pr1_pru1_gpo18);
    
            //  Clear the interrupt
     
            CT_ECAP.ECCLR_bit.CEVT2 = 0x1;
        }

        CT_ECAP.ECCLR_bit.INT = 0x1;
    }
}

void updateRPMsg()
{
    uint16_t src, dst, len;

    //  
    //  Check for Host Interrupt Event
    //

    if (__R31 & HOST_INT) 
    {
        // Clear the host interrupt envent
        CT_INTC.SICR_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;

        // Receive all available messages, multiple messages can be sent per kick 
        while (pru_rpmsg_receive(&s_transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) 
        {
            // Echo the message back to the same address from which we just received 
            pru_rpmsg_send(&s_transport, dst, src, payload, len);
        }
    }
}

void updateHeartbeat()
{
    __R30 |= pr1_pru1_gpo5;
    __R30 |= pr1_pru1_gpo9;
    
    //__R30 &= ~(pr1_pru1_gpo5);
    //__R30 &= ~(pr1_pru1_gpo9);
    //__R30 &= ~(pr1_pru1_gpo18);
}
