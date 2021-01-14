#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include <fcntl.h>
#include <string.h>
#include <sys/poll.h>

// -----------------------------------------------------------------------------
// file scope consts
// -----------------------------------------------------------------------------

static const char* k_pru_channel_device = "/dev/rpmsg_pru31";

static const char* k_pru_rproc_device_root = "/sys/class/remoteproc/remoteproc5/";
static const char* k_pru_firmware_root = "/lib/firmware/";
static const char* k_pru_firmware = "am57xx-pru1_1-tach_control";

static const uint32_t k_message_buffer_max_len = 512;

// -----------------------------------------------------------------------------
// file scope statics
// -----------------------------------------------------------------------------

static char s_message_buffer[512];
struct pollfd s_pru_channel;

// -----------------------------------------------------------------------------
//  forward decl of internal methods
// -----------------------------------------------------------------------------

void init_pru();
void init_rpmsg();

void update_rpmsg();

// -----------------------------------------------------------------------------
//  Main
// -----------------------------------------------------------------------------

int main(void)
{
    // Init

    init_pru();
    init_rpmsg();

    // Update

    while (1)
    {
        update_rpmsg();
    }

    close(s_pru_channel.fd);
}

void init_pru()
{
    char pru_rproc_device_state_filename[128];
    strcpy(pru_rproc_device_state_filename, k_pru_rproc_device_root);
    strcat(pru_rproc_device_state_filename, "state");

    char pru_rproc_device_firmware_filename[128];
    strcpy(pru_rproc_device_firmware_filename, k_pru_rproc_device_root);
    strcat(pru_rproc_device_firmware_filename, "firmware");

    char pru_firmware_filename[128];
    strcpy(pru_firmware_filename, k_pru_firmware_root);
    strcat(pru_firmware_filename, k_pru_firmware);

    int pru_rproc_device_state_fd;
    int pru_rproc_device_firmware_fd;

    // first chcek to make sure all the files are accesable
    
    if(access(pru_rproc_device_state_filename, F_OK) < 0) 
    {
        printf("can not acces (%s) need to run as root\n", pru_rproc_device_state_filename);
        exit(-1);
    }

    pru_rproc_device_state_fd = open (pru_rproc_device_state_filename, O_RDWR);

    if (pru_rproc_device_state_fd < 0)
    {
        printf("Could not open %s\n", pru_rproc_device_state_filename);
        exit(1);
    }

    if(access(pru_rproc_device_firmware_filename, F_OK) < 0) 
    {
        printf("can not acces (%s) need to run as root\n", pru_rproc_device_firmware_filename);
        exit(-1);
    }

    pru_rproc_device_firmware_fd = open (pru_rproc_device_firmware_filename, O_WRONLY);

    if (pru_rproc_device_firmware_fd < 0)
    {
        printf("Could not open %s\n", pru_rproc_device_firmware_filename);
        exit(1);
    }

    if(access(pru_firmware_filename, F_OK) < 0) 
    {
        printf("can not acces (%s)\n", pru_firmware_filename);
        exit(-1);
    }

    // stop the PRU, first check status.  If running, then stop
    
    char buf[128];
    if (read(pru_rproc_device_state_fd, buf, 128) <= 0)
    {
        printf("Could not read %s\n", pru_rproc_device_state_filename);
        exit(1);
    }

    if (strncmp(buf, "running", 7) == 0)
    {
        printf("Stopping %s\n", k_pru_rproc_device_root);

        write(pru_rproc_device_state_fd, "stop", 4);
    }

    // load the firmware to the PRU

    printf("Loading firmware %s\n", k_pru_firmware);

    write(pru_rproc_device_firmware_fd, k_pru_firmware, strlen(k_pru_firmware));
    close(pru_rproc_device_firmware_fd);

    // start the PRU

    printf("Starting %s\n", k_pru_rproc_device_root);

    write(pru_rproc_device_state_fd, "start", 5);
    close(pru_rproc_device_state_fd);

    // pause for 2 seconds before continuing

    usleep(2000000);
}

void init_rpmsg()
{
    s_pru_channel.fd = open(k_pru_channel_device, O_RDWR);

    // open the character device channel to the pru
    if (s_pru_channel.fd < 0) 
    {
        printf("Could not open pru device file%s\n", k_pru_channel_device);
        exit(-1);
    }

    // send the init message, this is how the PRU gets our  "address" 
    if (write(s_pru_channel.fd, "SYNC", 9) < 0)
    {
        printf("Could not open pru device file%s\n", k_pru_channel_device);
        exit(-1);
    }
}
        
void update_rpmsg()
{
    if (read(s_pru_channel.fd, s_message_buffer, k_message_buffer_max_len) > 0)
    {
        printf("%s", s_message_buffer);
    }
}  
