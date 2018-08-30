/**
 * LCM module for connecting to a Hokuyo laser range finder (URG and UTM are
 * supported) and transmitting range data via LCM.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <sys/time.h>
#include <time.h>

#include <glib.h>
#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <lcmtypes/bot_core_sensor_status_t.h>

#include "urglib/urg_sensor.h"
#include "urglib/urg_utils.h"
#include "urglib/urg_errno.h"

#define MAX_ACM_DEVS 20
#define DEFAULT_BAUDRATE 115200
#define DEFAULT_PORT 10940
#define BUFFER_SIZE 64 + 2 + 6

#define TO_DEGREES(rad) ((rad)*180/M_PI)


static void
usage(const char *progname)
{
    fprintf (stderr, "usage: %s [options]\n"
             "\n"
             "  -h, --help             shows this help text and exits\n"
             "  -c, --channel CHAN     LCM channel name\n"
             "  -d, --device DEV       USB device file to connect to\n"
             "  -i, --id ID            Search for Hokuyo with serial id ID\n"
             "  -e, --ethernet IP      Connect via Ethernet (UTM-30LX-EW unit)\n"
             "  -a, --auto-lookup      Use BotParam to determine LCM channel\n"
             "                           (requires specified channel, device, or ethernet)\n"
             "  -l, --lcmurl URL       LCM URL\n"
             "  -s, --skipscans NUM    Publish every NUMth scan [defaults to 1]\n"
             "                           (reduce scan frequency)\n"
             "  -b, --skipbeams NUM    Read every NUMth beams  [defaults to 1]\n"
             "                           (reduce angular resolution)\n"
             "  -I, --intensity        Read Intensity data from the laser scanner\n"
             "                            MAY NOT WORK WITH URGs\n"
             "  -m, --multi-echo       Request multi-echo information for supported units\n"
             "                            (Currently supported only on UTM-30LX-EW)\n"
             "  -t, --timesync         Timestamp messages based on estimated sync to lidar times\n"
             , g_path_get_basename(progname));
}

static char **
_get_acm_devnames(void)
{
    char **result = (char**)calloc(1, (MAX_ACM_DEVS+1)*sizeof(char*));
    int n = 0;
    for (int i=0;i<MAX_ACM_DEVS;i++) {
        char devname[256];
        sprintf(devname,"/dev/ttyACM%d",i);
        if(g_file_test(devname, G_FILE_TEST_EXISTS)) {
            result[n] = g_strdup(devname);
            n++;
        }
    }
    return result;
}

static int
_connect_by_device(urg_t *urg, const char *device)
{
    if(urg_open(urg, URG_SERIAL, device, DEFAULT_BAUDRATE) != URG_NO_ERROR) {
        fprintf (stderr, "_connect_by_device: urg_open on device %s returned error = %s\n",
                 device, urg_error (urg));
        return 0;
    }

    return 1;
}

static int
_connect_any_device(urg_t *urg)
{
    char **devnames = _get_acm_devnames();
    if(!devnames[0]) {
        fprintf(stderr, "No Hokuyo detected\n");
    }
    for(int i=0; devnames[i]; i++) {
        fprintf(stdout, "Trying %s...\n", devnames[i]);
        if(_connect_by_device(urg, devnames[i])) {
            g_strfreev(devnames);
            return 1;
        }
    }
    g_strfreev(devnames);
    return 0;
}

static gboolean
_read_serialno(urg_t *urg, int *serialno)
{
    // read the serial number of the hokuyo.  This is buried within
    // a bunch of other crap.
    const char *serial_id = urg_sensor_serial_id (urg);
    char *eptr = NULL;
    int sn = strtol (serial_id, &eptr, 10);
    *serialno = sn;

    fprintf (stdout, "Got serial %d\n", sn);

    /*
     * int LinesMax = 5;
     * char version_buffer[LinesMax][BUFFER_SIZE];
     * char *version_lines[LinesMax];
     * for (int i = 0; i < LinesMax; ++i) {
     *     version_lines[i] = version_buffer[i];
     * }
     * int status = urg_versionLines(urg, version_lines, LinesMax);
     * if (status < 0) {
     *     fprintf(stderr, "urg_versionLines: %s\n", urg_error(urg));
     *     return 0;
     * }
     * const char *prefix = "SERI:";
     * int plen = strlen(prefix);
     *
     * for(int i = 0; i < LinesMax; ++i) {
     *     if(!strncmp(version_lines[i], prefix, plen)) {
     *         char *eptr = NULL;
     *         int sn = strtol(version_lines[i] + plen+1, &eptr, 10); //skip 1 since the first entry can sometimes be a letter (one of the hokuyos is H0803547)
     *         if(eptr != version_lines[i] + plen) {
     *             *serialno = sn;
     *             return 1;
     *         }
     *     }
     * }
     */
    return 0;
}

static gboolean
_connect_by_id(urg_t *urg, const char *desired_serialno)
{
    char **devnames = _get_acm_devnames();
    for(int i=0; devnames[i]; i++) {
        fprintf(stdout, "Trying %s...\n", devnames[i]);
        const char *devname = devnames[i];

        if(!_connect_by_device(urg, devname)) {
            continue;
        }

        if (!strcmp (desired_serialno, urg_sensor_serial_id (urg))) {
            fprintf(stdout, "Found unit with serial %s on %s\n",
                    urg_sensor_serial_id (urg), devname);
            g_strfreev(devnames);
            return 1;
        } else {
            fprintf(stdout, "Skipping %s (found serial #: %s, desired: %s)\n",
                    devname, urg_sensor_serial_id (urg), desired_serialno);
            urg_laser_off (urg);
            urg_close (urg);
        }
    }
    g_strfreev(devnames);
    return 0;
}

static gboolean
_connect(urg_t *urg, char *serialno, const char *device, const char *ip_address,
         int *data_max, bot_timestamp_sync_state_t **sync, int skipscans,
         int skipbeams, int readIntensities, int multi_echo, char *unit_serialno)
{
    if (!device && ip_address) {
        if (urg_open (urg, URG_ETHERNET, ip_address, DEFAULT_PORT) != URG_NO_ERROR) {
            fprintf (stderr, "_connect: urg_open returned error = %s\n",
                     urg_error (urg));
            return 0;
        }
    }
    else {
        if(serialno) {
            if(!_connect_by_id(urg, serialno))
                return 0;
        } else if(device) {
            if(!_connect_by_device(urg, device))
                return 0;
        } else {
            if(!_connect_any_device(urg))
                return 0;
        }
    }

    // If multiecho is requested, check to see whether it is supported
    if (multi_echo) {
        if (strcmp (urg_sensor_product_type (urg), "UTM-30LX-EW") != 0) {
            fprintf (stderr, "ERROR: Model %s does not support multi-echo\n",
                     urg_sensor_product_type (urg));
            urg_close (urg);
            return 0;
        }
    }

    // Print out Hokuyo details
    fprintf(stdout, "\n");
    fprintf(stdout, "Model:              %s\n", urg_sensor_product_type (urg));
    fprintf(stdout, "Serial number:      %s\n", urg_sensor_serial_id (urg));
    fprintf(stdout, "Firmware version:   %s\n", urg_sensor_firmware_version (urg));

    strncpy (unit_serialno, urg_sensor_serial_id (urg), 256*sizeof(char));

    if(data_max)
        *data_max = urg_max_data_size (urg);

    if(*sync)
        bot_timestamp_sync_free(*sync);
    *sync = bot_timestamp_sync_init(1000, 4294967296L, 1.001);
    // guessed at wrap-around based on 4 byte field.

    // estimate clock skew
    urg_start_time_stamp_mode (urg);
    for (int i = 0; i < 10; i++) {
        int64_t hokuyo_mtime = urg_time_stamp (urg);
        int64_t now = bot_timestamp_now();
        bot_timestamp_sync(*sync, hokuyo_mtime, now);
    }
    urg_stop_time_stamp_mode (urg);


    // Set scanning parameters
    int min_step, max_step;
    urg_step_min_max (urg, &min_step, &max_step);
    fprintf (stdout, "calling urg_set_scanning_parameter (urg, %d, %d, %d)\n",
             min_step, max_step, skipbeams);
    if (urg_set_scanning_parameter (urg, min_step, max_step, skipbeams) != 0) {
        fprintf (stderr, "_connect: urg_set_scanning parameter returned error = %s\n",
                 urg_error (urg));
        urg_close (urg);
        return 0;
    }


    // Start data acquisition with desired mode and scans to skip
    urg_measurement_type_t mode;
    if (!multi_echo) {
        if (readIntensities)
            mode = URG_DISTANCE_INTENSITY;
        else
            mode = URG_DISTANCE;
    }
    else {
        if (readIntensities)
            mode = URG_MULTIECHO_INTENSITY;
        else
            mode = URG_MULTIECHO;
    }

    int status = urg_start_measurement (urg, mode, 0, skipscans-1);
    if (status < 0) {
        fprintf(stderr, "urg_start_measurement(): %s\n", urg_error(urg));
        urg_close (urg);
        return 0;
    }

    return 1;
}

int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    char *optstring = "hc:d:e:p:i:as:b:Imta";
    int c;
    struct option long_opts[] = {
        {"help", no_argument, 0, 'h'},
        {"channel", required_argument, 0, 'c'},
        {"device", required_argument, 0, 'd'},
        {"ethernet", required_argument, 0, 'e'},
        {"id", required_argument, 0, 'i'},
        {"lcmurl", required_argument, 0, 'l'},
        {"skipscans",required_argument,0,'s'},
        {"skipbeams",required_argument,0,'b'},
        {"intensity", no_argument, 0, 'I'},
        {"multi-echo", no_argument, 0, 'm'},
        {"timesync", no_argument,0,'t'},
        {"auto-lookup", no_argument,0,'a'},
        {0, 0, 0, 0}
    };

    int exit_code = 0;
    char *device = NULL;
    char *channel = g_strdup("HOKUYO_LIDAR");
    int channel_option = 0;
    char *lcm_url = NULL;
    char *ip_address = NULL;
    char *serialno = NULL;
    int skipscans = 1;
    int skipbeams = 1;
    int readIntensities = 0;
    int no_timesync = 1;
    int read_from_param = 0;
    int multi_echo = 0;

    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c) {
            case 'c':
                free(channel);
                channel = g_strdup(optarg);
                channel_option = 1;
                break;
            case 'd':
                free(device);
                device = g_strdup(optarg);
                break;
            case 'e':
                free(ip_address);
                ip_address = g_strdup (optarg);
                break;
            case 'p':
                free(lcm_url);
                lcm_url = g_strdup(optarg);
                break;
            case 'a':
                read_from_param = 1;
                break;
            case 'i':
            {
                free (serialno);
                serialno = g_strdup (optarg);
                break;
            }
            break;
            case 's':
            {
                skipscans = atoi(optarg);
                if (skipscans<1){
                    usage(argv[0]);
                    return 1;
                }
            }
            break;
            case 'b':
            {
                skipbeams = atoi(optarg);
                if (skipbeams<1){
                    usage(argv[0]);
                    return 1;
                }
            }
            break;
            case 'I':
                readIntensities =1;
                break;
            case 'm':
                multi_echo = 1;
                break;
            case 't':
                no_timesync=0;
                break;
            case 'h':
            default:
                usage(argv[0]);
                return 1;
        }
    }

    // Validate usage
    // Auto lookup requires either a channel number, a device name, or an ethernet address
    if (read_from_param && !channel_option && !device && !ip_address) {
        usage (argv[0]);
        return 1;
    }

    int data_max;
    long* data = NULL;
    unsigned short* intensity = NULL;

    // setup LCM
    lcm_t *lcm = lcm_create(lcm_url);
    if(!lcm) {
        fprintf(stderr, "Couldn't setup LCM\n");
        return 1;
    }


    BotParam *param = bot_param_new_from_server (lcm, 0);
    if (!param) {
        fprintf (stderr, "Error getting BotParam\n");
        exit_code = 1;
        goto done;
    }

    // Auto lookup: specified channel --> serial number and/or ethernet address
    if (read_from_param && channel_option) {

        BotParam *param = bot_param_new_from_server (lcm, 0);
        if (!param) {
            fprintf (stderr, "Error getting BotParam\n");
            exit_code = 1;
            goto done;
        }

        char *sensor_name =
            bot_param_get_planar_lidar_name_from_lcm_channel (param, channel);

        if (!sensor_name) {
            fprintf (stderr, "ERROR: Unable to find planar lidar with specified channel %s\n", channel);
            exit_code = 1;
            goto done;
        }

        char *device_string = NULL;
        char cfg_device[256];
        snprintf(cfg_device, 256, "planar_lidars.%s.device", sensor_name);
        if (bot_param_get_str (param, cfg_device, &device_string) == 0) {
            free (ip_address);
            ip_address = g_strdup (device_string);
        }
        else {
            char *serial_string = NULL;
            char cfg_serial[256];
            snprintf(cfg_serial, 256, "planar_lidars.%s.serial", sensor_name);
            if (bot_param_get_str (param, cfg_serial, &serial_string) == 0) {
                free (serialno);
                serialno = g_strdup (serial_string);
            }
            else {
                fprintf (stderr, "ERROR: Unable to find serial number or ethernet address for lidar with channel name %s\n",
                         channel);
                exit_code = 1;
                goto done;
            }
        }
    }

    bot_timestamp_sync_state_t *sync = NULL;

    urg_t urg;
    int max_initial_tries = 10;
    int connected = 0;
    char *unit_serialno = (char *) calloc (1, 256*sizeof(char));
    for(int i=0; i<max_initial_tries && !connected; i++) {

        fprintf (stdout, "Calling _connect (urg, serialno = %s, device = %s, ip_address = %s)\n",
                 serialno, device, ip_address);
        connected = _connect (&urg, serialno, device, ip_address,
                              &data_max, &sync, skipscans, skipbeams,
                              readIntensities, multi_echo, unit_serialno);

        if(!connected) {
            struct timespec ts = { 0, 500000000 };
            nanosleep(&ts, NULL);
        }
    }
    if(!connected) {
        fprintf(stderr, "Unable to connect to any device\n");
        lcm_destroy(lcm);
        exit_code = 1;
        goto done;
    }


    // Auto lookup with specified device --> look up serial number
    if (read_from_param && !channel_option) {
        char **planar_lidar_names = NULL;
        int no_channels = 0;
        int found_channel = 0;

        BotParam *param = bot_param_new_from_server (lcm, 0);
        if (!param) {
            fprintf (stderr, "Error getting BotParam\n");
            exit_code = 1;
            goto done;
        }

        planar_lidar_names = bot_param_get_all_planar_lidar_names (param);

        if (planar_lidar_names) {
            for (int pind = 0; planar_lidar_names[pind] != NULL; pind++) {

                char *serial_string;
                char cfg_var[256];
                snprintf(cfg_var, 256, "planar_lidars.%s.serial", planar_lidar_names[pind]);
                serial_string = bot_param_get_str_or_fail (param, cfg_var);

                if (!strcmp (unit_serialno, serial_string)) {
                    free (channel);
                    channel = bot_param_get_sensor_lcm_channel (param, "planar_lidars",
                                                                planar_lidar_names[pind]);
                    found_channel = 1;
                    break;
                }
                free (serial_string);
            }
            g_strfreev(planar_lidar_names);
        }
        if (!found_channel) {
            fprintf (stderr, "ERROR: Unable to find Hokuyo with serial %s in config.\n",
                     unit_serialno);
            exit_code = 1;
            goto done;
        }
    }

    // # of measurements per scan?
    data_max = urg_max_data_size (&urg);
    if (multi_echo)
        data_max = URG_MAX_ECHO * data_max;

    fprintf (stdout, "data_max = %d\n", data_max);

    data = (long *) malloc (sizeof(long) * data_max);
    long time_stamp;
    intensity = (unsigned short *) malloc(sizeof(unsigned short) * data_max);
    if (data == NULL || intensity ==NULL) {
        perror("data buffer");
        exit_code = 1;
        goto done;
    }

    bot_core_planar_lidar_t msg;
    int max_nranges = urg_max_data_size (&urg);

    bot_core_planar_lidar_t *m_msgs;

    //if (multi_echo)
    //  max_nranges = URG_MAX_ECHO * max_nranges;

    msg.ranges = (float*) malloc(sizeof(float) * max_nranges);
    msg.nintensities = 0;
    msg.intensities = (float*) malloc(sizeof(float) * max_nranges);

    int min_step, max_step;
    double min_rad, max_rad;
    urg_step_min_max (&urg, &min_step, &max_step);
    min_rad = urg_step2rad (&urg, min_step);
    max_rad = urg_step2rad (&urg, max_step);


    double area_total = max_rad - min_rad;

    msg.radstep = (max_rad - min_rad) / (max_nranges - 1);

    msg.rad0 = min_rad;

    if (multi_echo){
        //allocate multi msg space
        //multi_msg.no_returns = URG_MAX_ECHO;
        m_msgs = (bot_core_planar_lidar_t *)calloc(URG_MAX_ECHO, sizeof(bot_core_planar_lidar_t));
        for(int i=0; i < URG_MAX_ECHO; i++){
            //allocate each msg
            m_msgs[i].ranges = (float*) malloc(sizeof(float) * max_nranges);
            m_msgs[i].nintensities = 0;
            m_msgs[i].intensities = (float*) malloc(sizeof(float) * max_nranges);
            m_msgs[i].radstep = (max_rad - min_rad) / (max_nranges - 1);

            m_msgs[i].rad0 = min_rad;
        }
    }

    fprintf(stdout, "Angular resolution: %.2f deg\n", TO_DEGREES(msg.radstep));
    fprintf(stdout, "Starting angle:     %.2f deg\n", TO_DEGREES(msg.rad0));
    fprintf(stdout, "Ending angle:       %.2f deg\n", TO_DEGREES(max_rad));
    fprintf(stdout, "Scan RPM:           %.2f hz\n", 1E6/((double) urg_scan_usec (&urg)));

    char **m_channel_names = NULL;

    if(multi_echo){
        m_channel_names = (char **) calloc(URG_MAX_ECHO, sizeof(char *));
        for(int i=0; i < URG_MAX_ECHO; i++){
            m_channel_names[i] = (char *) calloc(1024, sizeof(char));
            if(i == 0){
                sprintf(m_channel_names[i], "%s", channel);
            }
            else{
                sprintf(m_channel_names[i], "%s_%d", channel, i);
            }
        }
    }

    //char multi_channel_name[1024];
    //sprintf(multi_channel_name, "%s_MULTI", channel);

    fprintf(stdout, "LCM channel:        %s\n", channel);
    fprintf(stdout, "\n");


    int64_t now = bot_timestamp_now();
    int64_t report_last_utime = now;
    int64_t report_interval_usec = 2000000;
    int64_t next_report_utime = now + report_interval_usec;

    int64_t scancount_since_last_report = 0;
    int failure_count = 0;
    int reconnect_thresh = 10;
    int epic_fail = 0;
    int max_reconn_attempts = 600;

    // loop forever, reading scans
    while(!epic_fail) {
        int nranges=-1;
        if (multi_echo) {
            if (!readIntensities)
                nranges = urg_get_multiecho (&urg, data, &time_stamp);
            else
                nranges = urg_get_multiecho_intensity (&urg, data, intensity, &time_stamp);
            //mutli-echo returns the no of groups
            nranges;// *= 3;
        }
        else {
            if (!readIntensities)
                nranges = urg_get_distance (&urg, data, &time_stamp);
            else
                nranges = urg_get_distance_intensity (&urg, data, intensity, &time_stamp);
        }


        if(nranges > max_nranges) {
            printf("WARNING:  received more range measurements than the maximum advertised!\n");
            printf("          Hokuyo reported max %d, but received %d\n", max_nranges, nranges);
            max_nranges = nranges;
            msg.ranges = (float*) realloc(msg.ranges, sizeof(float) * max_nranges);
            if (readIntensities)
                msg.intensities = (float*) realloc(msg.intensities, sizeof(float) * max_nranges);
        }
        now = bot_timestamp_now();
        //int64_t hokuyo_mtime = urg_recentTimestamp(&urg);

        if(nranges < 0) {
            // sometimes, the hokuyo can freak out a little.
            // Count how many times we've failed to get data from the hokuyo
            // If it's too many times, then reset the connection.  That
            // can help sometimes..
            fprintf(stderr, "urg_receiveData(): %s\n", urg_error(&urg));
            failure_count++;
            struct timespec ts = { 0, 300000000 };
            nanosleep(&ts, NULL);

            int reconn_failures = 0;
            while(failure_count > reconnect_thresh) {
                if(connected) {
                    urg_close (&urg);
                    connected = 0;
                    fprintf(stderr, "Comms failure.  Trying to reconnect...\n");
                }

                free (unit_serialno);
                unit_serialno = (char *) calloc (1, 256*sizeof(char));
                if (_connect (&urg, serialno, device, ip_address, &data_max, &sync,
                              skipscans, skipbeams, readIntensities, multi_echo, unit_serialno)) {
                    failure_count = 0;
                    connected = 1;
                }

                // Throttle reconnect attempts
                struct timespec ts = { 0, 500000000 };
                nanosleep(&ts, NULL);

                reconn_failures++;
                if(reconn_failures > max_reconn_attempts) {
                    fprintf(stderr, "Exceeded maximum reconnection attempts.\n");
                    exit_code = 1;
                    epic_fail = 1;
                    break;
                }
            }
            continue;
        }

        if(failure_count > 0)
            failure_count--;

        if (no_timesync)
            msg.utime = now;
        //else
        //  msg.utime = bot_timestamp_sync(sync, hokuyo_mtime, now);

        if(!multi_echo){
            msg.nranges = nranges/skipbeams;
            if (readIntensities){
                msg.nintensities = nranges/skipbeams;
                double min_intensity = 0;
                double max_intensity = 0;
                for(int i=0; i<nranges; i+=skipbeams) {
                    if (intensity[i] <= min_intensity)
                        min_intensity = intensity[i];
                    else if (intensity[i] > max_intensity)
                        max_intensity = intensity[i];
                }

                int c=0;
                for(int i=0; i<nranges; i+=skipbeams) {
                    msg.ranges[c] = data[i] * 1e-3;
                    msg.intensities[c] = (intensity[i]-min_intensity)/(max_intensity-min_intensity);
                    c++;
                }
            }
            else{
                int c=0;
                for(int i=0; i<nranges; i+=skipbeams)
                    msg.ranges[c++] = data[i] * 1e-3;
            }
            bot_core_planar_lidar_t_publish(lcm, channel, &msg);
        }
        else{ //multi-echo - need to do something funky
            //data is ordered min value first and
            //fprintf(stderr,"Multi Echo\n");
            //multi_msg.utime = now;
            for(int j=0; j< URG_MAX_ECHO; j++){
                bot_core_planar_lidar_t *c_msg = &m_msgs[j];

                c_msg->utime = now;
                c_msg->nranges = nranges/skipbeams;

                if (readIntensities){
                    c_msg->nintensities = nranges/skipbeams;
                    double min_intensity = 0;
                    double max_intensity = 0;
                    for(int i=j; i< nranges * URG_MAX_ECHO; i+= URG_MAX_ECHO + URG_MAX_ECHO *(skipbeams-1)) {
                        if (intensity[i] <= min_intensity)
                            min_intensity = intensity[i];
                        else if (intensity[i] > max_intensity)
                            max_intensity = intensity[i];
                    }

                    int c=0;
                    //for(int i=0; i<nranges; i+=skipbeams) {
                    for(int i=j; i< nranges * URG_MAX_ECHO; i+= URG_MAX_ECHO + URG_MAX_ECHO *(skipbeams-1)) {
                        //if(j==0)
                        //  fprintf(stderr,"\t Ind %d\n", i);
                        c_msg->ranges[c] = data[i] * 1e-3;
                        c_msg->intensities[c] = (intensity[i]-min_intensity)/(max_intensity-min_intensity);
                        c++;
                    }
                }
                else{
                    //fprintf(stderr, "%d, nranges : %d Max Echo : %d skip : %d\n",
                    //      j, nranges, URG_MAX_ECHO, skipbeams);

                    int c=0;
                    for(int i=j; i< nranges * URG_MAX_ECHO; i+= URG_MAX_ECHO + URG_MAX_ECHO *(skipbeams-1)) {
                        //    if(j==0)
                        //fprintf(stderr,"\t[%d] Ind %d\n", c, i);
                        //for(int i=0; i<nranges; i+=skipbeams)
                        if(j>0 && data[i]==0){
                            //check the one before
                            for(int k=1; k <=j; k--){
                                if(data[i-k] >0){
                                    c_msg->ranges[c++] = data[i-k] * 1e-3;
                                    break;
                                }
                            }
                        }
                        else{
                            c_msg->ranges[c++] = data[i] * 1e-3;
                        }
                    }
                }
                //publish the first msg as the normal one
                bot_core_planar_lidar_t_publish(lcm, m_channel_names[j], &m_msgs[j]);
            }

        }


        scancount_since_last_report++;

        if(now > next_report_utime) {
            double dt = (now - report_last_utime) * 1e-6;

            fprintf(stdout, "%4.1f Hz\n", scancount_since_last_report / dt);
            //this will add a dependency to envoy code base
            bot_core_sensor_status_t msg;
            msg.utime = bot_timestamp_now();
            msg.sensor_name = channel; //maybe use some indexing - to make it unique
            msg.rate = scancount_since_last_report / dt;
            msg.type = BOT_CORE_SENSOR_STATUS_T_HOKUYO_LASER;
            bot_core_sensor_status_t_publish(lcm, "SENSOR_STATUS_HOKUYO", &msg);

            scancount_since_last_report = 0;
            next_report_utime = now + report_interval_usec;
            report_last_utime = now;
        }
    }

done:
    if(connected) {
        urg_laser_off (&urg);
        urg_close (&urg);
    }

    if(sync)
        bot_timestamp_sync_free(sync);
    free(data);
    free(intensity);
    free(lcm_url);
    free(channel);
    free(device);

    return exit_code;
}
