#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <signal.h>

// -----------------------------------------------------------------------------
// file scope consts
// -----------------------------------------------------------------------------

static const char* k_pru_channel_device = "/dev/rpmsg_pru31";

static const char* k_pru_rproc_device_root = "/sys/class/remoteproc/remoteproc5";
static const char* k_pru_firmware_root = "/lib/firmware";
static const char* k_pru_firmware = "am57xx-pru1_1-tach_control";
static const char* k_output_root = "./tach_output";

static const uint32_t k_message_buffer_max_len = 512;

// -----------------------------------------------------------------------------
// file scope statics
// -----------------------------------------------------------------------------

static char s_message_buffer[512];
static struct pollfd s_pru_channel;

static uint8_t s_recording;
static FILE* s_recording_file;

static uint32_t s_num_teeth;
static uint32_t s_prev_sample_time_usec;

// -----------------------------------------------------------------------------
//  forward decl of internal methods
// -----------------------------------------------------------------------------

void init_sig_handlers();
void init_pru();
void init_rpmsg();

void update_rpmsg();

void shutdown();

// -----------------------------------------------------------------------------
// SIGINT handler
// -----------------------------------------------------------------------------

void termination_handler (int signum)
{
    shutdown();
}

// -----------------------------------------------------------------------------
//  Main
// -----------------------------------------------------------------------------

int main(void)
{
    // Init

    init_sig_handlers();
    init_pru();
    init_rpmsg();

    // Update

    while (1)
    {
        update_rpmsg();
    }

}

void shutdown()
{
    write(s_pru_channel.fd, "EXIT", 4);
    close(s_pru_channel.fd);

    exit(0);
}

void init_sig_handlers()
{
    struct sigaction action;

    action.sa_handler = termination_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    sigaction (SIGINT, &action, NULL);
}

void init_pru()
{
    char pru_rproc_device_state_filename[128];
    sprintf(pru_rproc_device_state_filename, "%s/state", k_pru_rproc_device_root);

    char pru_rproc_device_firmware_filename[128];
    sprintf(pru_rproc_device_firmware_filename, "%s/firmware", k_pru_rproc_device_root);

    char pru_firmware_filename[128];
    sprintf(pru_firmware_filename, "%s/%s", k_pru_firmware_root, k_pru_firmware);

    int pru_rproc_device_state_fd;
    int pru_rproc_device_firmware_fd;

    pru_rproc_device_state_fd = open(pru_rproc_device_state_filename, O_RDWR);

    if (pru_rproc_device_state_fd < 0)
    {
        printf("Could not open %s\n", pru_rproc_device_state_filename);
        exit(1);
    }

    pru_rproc_device_firmware_fd = open(pru_rproc_device_firmware_filename, O_WRONLY);

    if (pru_rproc_device_firmware_fd < 0)
    {
        printf("Could not open %s\n", pru_rproc_device_firmware_filename);
        exit(1);
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
    s_recording = 0;
    memset(s_message_buffer, 0, sizeof(s_message_buffer));

    // Make sure output root exists
    
    struct stat st;

    if (stat(k_output_root, &st) < 0)
    {
        if (mkdir(k_output_root, 0777) < 0)
        {
            printf("Could not open recoding directory: %s\n", k_output_root);
            exit(-1);
        }
    }
    else if (!S_ISDIR(st.st_mode))
    {
        printf("%s exists, but is not a directory\n", k_output_root);
        exit(-1);
    }

    // open the character device channel to the pru

    s_pru_channel.fd = open(k_pru_channel_device, O_RDWR);

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

void get_timestamp(char *timestamp, size_t size)
{
    struct timeval time_value;
    time_t time_seconds;
    struct tm *time_tm;

    gettimeofday(&time_value, NULL);
    time_seconds = time_value.tv_sec;
    time_tm = localtime(&time_seconds);

    strftime(timestamp, size, "%Y-%m-%d_%H:%M:%S", time_tm);
}

void start_recording(char* message)
{
    s_recording = 1;
    s_prev_sample_time_usec = 0;

    char timestamp[64];
    get_timestamp(timestamp, sizeof(timestamp));

    // Open a file for recording our data
    
    char recording_filename[1024];
    sprintf(recording_filename, "%s/%s.csv", k_output_root, timestamp);

    s_recording_file = fopen(recording_filename, "w");

    if (s_recording_file == NULL)
    {
        printf("Could not open recoding file: %s\n", recording_filename);
        exit(-1);
    }

    uint32_t sample_rpm;

    sscanf(message, "START_MESSAGE:%u:%u", &sample_rpm, &s_num_teeth);
    printf("START RECORDING: %u\n", sample_rpm);

    uint32_t desired_dt = 0;
    
    if (sample_rpm != 0 && s_num_teeth != 0)
    {
        desired_dt = (uint32_t)(1000000 * 60 / (sample_rpm * s_num_teeth));
    }

    fprintf(s_recording_file, "host_time,pru_usec,rpm, dt,est dt, teeth\n");
    fprintf(s_recording_file, "%s,START,%u,0,%u,%u\n", timestamp, sample_rpm, desired_dt, s_num_teeth);
}

void stop_recording(char* message)
{
    uint32_t sample_rpm;

    char timestamp[64];
    get_timestamp(timestamp, sizeof(timestamp));

    sscanf(message, "STOP_MESSAGE:%u", &sample_rpm);
    printf("STOP RECORDING: %u\n", sample_rpm);
    fprintf(s_recording_file, "%s,STOP,%u\n", timestamp, sample_rpm);

    s_recording = 0;
    fclose(s_recording_file);

}

void process_message(char *message)
{
    char timestamp[64];
    get_timestamp(timestamp, sizeof(timestamp));

    char* entry = strtok(message, "\n");

    while (entry != NULL)
    {
        uint32_t sample_time_usec;
        uint32_t sample_rpm;

        sscanf(message, "%u:%u", &sample_time_usec, &sample_rpm);
        printf("%s %u -> %u\n", timestamp, sample_time_usec, sample_rpm);

        // If recording, persist to file

        if (s_recording)
        {
            uint32_t dt = sample_time_usec - s_prev_sample_time_usec;
            uint32_t desired_dt = 0;
    
            if (sample_rpm != 0 && s_num_teeth != 0)
            {
                desired_dt = (uint32_t)(1000000 * 60 / (sample_rpm * s_num_teeth));
            }

            fprintf(s_recording_file, "%s,%u,%u,%u,%u\n", timestamp, sample_time_usec, sample_rpm, dt, desired_dt);
        }

        s_prev_sample_time_usec = sample_time_usec;
        entry = strtok(NULL, "\n");
    }
}

void update_rpmsg()
{
    if (read(s_pru_channel.fd, s_message_buffer, k_message_buffer_max_len) > 0)
    {
        // check for special messages
        
        if (strncmp(s_message_buffer, "START", 5) == 0)
        {
            start_recording(s_message_buffer);
        }
        else if (strncmp(s_message_buffer, "STOP", 4) == 0)
        {
            stop_recording(s_message_buffer);
        }
        else if (strlen(s_message_buffer) > 0)
        {
            process_message(s_message_buffer);
        }
    }
}  
