// Program expects that there are status LED connected at
//
// BBAI Header Pin P8.14 - PRU Heartbeat
// BBAI Header Pin P8.16 - eCap Activity
// BBAI Header Pin P8.18 - RPMsg Activity
// 
// Also, the square wave input signal is connect at
//
// BBAI Header Pin P8.15 - PRU 1 eCap input 
//
// This code stated as a TI example from pru-software-support-package/examples

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_ecap.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>

#include "pru_util.h"
#include "pru_time.h"

#include "resource_table_1.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

// -----------------------------------------------------------------------------
// file scope consts
// -----------------------------------------------------------------------------

static const uint32_t k_num_teeth = 20;
static const uint32_t k_cycles_to_micros = 200;
static const uint32_t k_dt_to_hz = 1000000;

static const uint32_t k_message_buffer_max_len = 496;
static const uint32_t k_message_delay_dt_max = 1000000;

// The PRU-ICSS system events used for RPMsg are defined in the Linux 
// device tree
//
// PRU0 uses system event 16 (To ARM) and 17 (From ARM)
// PRU1 uses system event 18 (To ARM) and 19 (From ARM)

static const uint32_t k_pru_to_host_event = 18;
static const uint32_t k_pru_from_host_event = 19;

static const uint32_t k_heartbeat_dt = 500000;

//  The output pins used to communicate status via connected LEDs

static const uint32_t pr1_pru1_gpo5  = 0x1 << 5;     // BBAI Header Pin P8.18
static const uint32_t pr1_pru1_gpo9  = 0x1 << 9;     // BBAI Header Pin P8.14
static const uint32_t pr1_pru1_gpo18 = 0x1 << 18;    // BBAI Header Pin P8.16

// -----------------------------------------------------------------------------
// file scope statics
// -----------------------------------------------------------------------------

static struct pru_rpmsg_transport s_transport;

static uint32_t s_signal_low_dt;
static uint32_t s_signal_high_dt;

static uint32_t s_heartbeat_start_time;

static char s_message_last_send_time;
static char s_message_buffer[496];
static uint32_t s_message_buffer_index;

static char s_tmp_message_buffer[496];

static uint16_t s_rpmsg_arm_addr;
static uint16_t s_rpmsg_pru_addr;

static uint8_t s_connected_to_arm;

// -----------------------------------------------------------------------------
//  forward decl of internal methods
// -----------------------------------------------------------------------------

void init_syscfg();
void init_intc();
void init_ecap();
void init_rpmsg();
void init_gpio();
void init_heartbeat();

void update_rpmsg();
void update_ecap();
void update_heartbeat();

// -----------------------------------------------------------------------------
//  Main
// -----------------------------------------------------------------------------

void main(void)
{
    // Init

    pru_time_init();

    init_syscfg();
    init_intc();
    init_ecap();
    init_rpmsg();
    init_gpio();
    init_heartbeat();

    // Update
    
    while (1) 
    {
        pru_time_update();

        update_rpmsg();
        update_ecap();
        update_heartbeat();
    }
}

void send_message()
{
    // Only send message if there is someone on the other side listing

    if (s_connected_to_arm)
    {
        pru_rpmsg_send(&s_transport, s_rpmsg_pru_addr, s_rpmsg_arm_addr, s_message_buffer, k_message_buffer_max_len);//s_message_buffer_index);
    }

    s_message_buffer_index = 0;
    memset(s_message_buffer, 0, k_message_buffer_max_len);

    s_message_last_send_time = pru_time_gettime();
}

void append_to_message(uint32_t dt)
{
    uint32_t timestamp = pru_time_gettime();
    uint32_t dt_per_revelution = dt * k_num_teeth;
    uint32_t rpm = k_dt_to_hz / dt_per_revelution;

    // Can't use sprintf beacuse it doesn't fit in PRU memory
    // so used an custom itoa impl
    
    char* message = s_tmp_message_buffer;

    char* timestamp_string = pru_util_itoa(timestamp, 10);
    int timestamp_string_len = strlen(timestamp_string);
    memcpy(message, timestamp_string, timestamp_string_len);

    message += timestamp_string_len;
    *message = ':';
    message++;

    char* rpm_string = pru_util_itoa(rpm, 10);
    int rpm_string_len = strlen(rpm_string);
    memcpy(message, rpm_string, rpm_string_len);

    message += rpm_string_len;
    *message = '\n';
    message++;
    *message = 0;

    // if it has been too long since last message send, or we will over
    // flow the buffer, send the current message and then add this to 
    // a new message buffer
    
    int32_t message_len = strlen(s_tmp_message_buffer);
    uint32_t current_time = pru_time_gettime();

    if (current_time - s_message_last_send_time > k_message_delay_dt_max ||
        s_message_buffer_index + message_len > k_message_buffer_max_len)
    {
        send_message();
    }

    memcpy(&s_message_buffer[s_message_buffer_index], s_tmp_message_buffer, message_len); 
    s_message_buffer_index += message_len;
}

// -----------------------------------------------------------------------------
//  Init Methods
// -----------------------------------------------------------------------------

void init_syscfg()
{
    // Allow OCP master port access by the PRU so the PRU can read external memories
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0x0;
}

void init_intc()
{
    // Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us
    CT_INTC.SICR_bit.STATUS_CLR_INDEX = k_pru_from_host_event;
}

void init_ecap()
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
    
    // Enable loading timer into capture registers
    CT_ECAP.ECCTL1_bit.CAPLDEN = 0x1;

    // ECAP Control Register 2
    // reset, which gets us most of the setting we want
    CT_ECAP.ECCTL2 = 0x00000000;

    // Only using first 2 capture events, so wrap after capture event 2
    CT_ECAP.ECCTL2_bit.STOPVALUE = 0x1;

    // Free running TSCNT
    CT_ECAP.ECCTL2_bit.TSCNTSTP = 0x1;

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

    //  init our dt counters
    s_signal_low_dt = 0;
    s_signal_high_dt = 0;
}

void init_rpmsg()
{
    volatile uint8_t *status;

    // Make sure the Linux drivers are ready for RPMsg communication 
    // Used to make sure the Linux drivers are ready for RPMsg communication
    // Found at linux-x.y.z/include/uapi/linux/virtio_config.h

    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & 0x04));

    // Initialize the RPMsg transport structure 

    pru_rpmsg_init(&s_transport, 
            &resourceTable.rpmsg_vring0, 
            &resourceTable.rpmsg_vring1, 
            k_pru_to_host_event, 
            k_pru_from_host_event);

    // Create the RPMsg channel between the PRU and ARM user space using the transport structure. 
    // Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
    // at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c

    uint32_t message_status;
    do
    {
         message_status = pru_rpmsg_channel(RPMSG_NS_CREATE, 
                 &s_transport, 
                 "rpmsg-pru", 
                 "PRUMessages", 
                 31);
    } while (message_status != PRU_RPMSG_SUCCESS);

    // Need to get first message from arm before we have established there is someone listing
    s_connected_to_arm = 0;

    // init message send time
    s_message_last_send_time = pru_time_gettime();

    // Clear out the message buffer
    s_message_buffer_index = 0;
    memset(s_message_buffer, 0, k_message_buffer_max_len);
}

void init_gpio()
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

void init_heartbeat()
{
    s_heartbeat_start_time = pru_time_gettime();
}

// -----------------------------------------------------------------------------
// Update Methods
// -----------------------------------------------------------------------------

void update_ecap()
{
    // Was an interupt generated
    
    if (CT_ECAP.ECFLG_bit.INT == 0x1)
    {
        if (CT_ECAP.ECFLG_bit.CEVT1 == 0x1)
        {
            // turn on the eCap trigger LED
            __R30 |= pr1_pru1_gpo18;
    
            //  Grab the counter, which is cycle difference since last capture
            //  event 1.  This is the time signal was low, right before event 1
            //  (signal goes high) triggers.  Store as micorseconds
            
            s_signal_low_dt = CT_ECAP.CAP1 / k_cycles_to_micros;
     
            //  Clear the interrupt
     
            CT_ECAP.ECCLR_bit.CEVT1 = 0x1;
        } 

        if (CT_ECAP.ECFLG_bit.CEVT2 == 0x1)
        {
            // turn 0ff the eCap trigger LED
            __R30 &= ~(pr1_pru1_gpo18);

            //  Grab the counter, which is cycle difference since last capture
            //  event 2.  This is the time signal was high, right before event 2
            //  (signal goes low) triggers.  Store as micorseconds
            
            s_signal_high_dt = CT_ECAP.CAP2 / k_cycles_to_micros;
    
            //  Clear the interrupt
     
            CT_ECAP.ECCLR_bit.CEVT2 = 0x1;

            //  We have 1 complete high - low cycle.  Write it out to the message
            append_to_message(s_signal_high_dt + s_signal_low_dt);
        }

        CT_ECAP.ECCLR_bit.INT = 0x1;
    }
}

void update_rpmsg()
{
    uint16_t message_len;

    //  
    //  Check for Host Interrupt Event
    //
 
    if (__R31 & k_pru_from_host_event)
    {
        // Clear the host interrupt envent
        CT_INTC.SICR_bit.STATUS_CLR_INDEX = k_pru_from_host_event;

        // Receive all available messages, multiple messages can be sent per kick 
        while (pru_rpmsg_receive(&s_transport, &s_rpmsg_arm_addr, &s_rpmsg_pru_addr, s_tmp_message_buffer, &message_len) == PRU_RPMSG_SUCCESS) 
        {
            __R30 |= pr1_pru1_gpo5;
        }

        s_connected_to_arm = 1;
    }
}

void update_heartbeat()
{
    uint32_t time = pru_time_gettime();
    
    if (time - s_heartbeat_start_time > k_heartbeat_dt)
    {
        s_heartbeat_start_time = time;

        if (__R30 & pr1_pru1_gpo9)
        {
            __R30 &= ~(pr1_pru1_gpo9);
        }
        else
        {
            __R30 |= pr1_pru1_gpo9;
        }
    }
}
