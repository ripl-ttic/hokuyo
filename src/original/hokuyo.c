#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <sys/time.h>
#include <time.h>

#include <glib.h>
#include <lcm/lcm.h>

#include <bot_param/param_client.h>
#include <lcmtypes/bot2_param.h>
#include <bot_param/param_util.h>

#define MAX_ACM_DEVS 20

#include <lcmtypes/bot_core_planar_lidar_t.h>
#include <lcmtypes/bot_core_sensor_status_t.h>

int read_from_config = 0;

#define TO_DEGREES(rad) ((rad)*180/M_PI)

#include "liburg/urg_ctrl.h"

int flip = 0;

static void
usage(const char *progname)
{
    fprintf (stderr, "usage: %s [options]\n"
             "\n"
             "  -h, --help             shows this help text and exits\n"
             //"  -c, --channel CHAN     LCM channel name\n"
             "  -c, --config          Use the proper channel name based on config file (using the serial no)\n"
             "  -d, --device DEV       Device file to connect to\n"
             "  -i, --id ID            Search for Hokuyo with serial id ID\n"
             "  -l, --lcmurl URL       LCM URL\n"
             "  -a, --auto-lookup      Determine device file from LCM channel with BotConf\n"
             , g_path_get_basename(progname));
}

static inline int64_t
_timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
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
_connect_by_device(urg_t *urg, urg_parameter_t *params, const char *device)
{
    if(urg_connect(urg, device, 115200) < 0) {
        return 0;
    }
    urg_parameters(urg, params);
    return 1;
}

static int
_connect_any_device(urg_t *urg, urg_parameter_t *params)
{
    char **devnames = _get_acm_devnames();
    if(!devnames[0]) {
        printf("No Hokuyo detected\n");
    }
    for(int i=0; devnames[i]; i++) {
        printf("Trying %s...\n", devnames[i]);
        if(_connect_by_device(urg, params, devnames[i])) {
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
    int LinesMax = 5;
    char version_buffer[LinesMax][UrgLineWidth];
    char *version_lines[LinesMax];
    for (int i = 0; i < LinesMax; ++i) {
        version_lines[i] = version_buffer[i];
    }
    int status = urg_versionLines(urg, version_lines, LinesMax);
    if (status < 0) {
        fprintf(stderr, "urg_versionLines: %s\n", urg_error(urg));
        return 0;
    }
    const char *prefix = "SERI:H";
    int plen = strlen(prefix);

    for(int i = 0; i < LinesMax; ++i) {
        if(!strncmp(version_lines[i], prefix, plen)) {
            char *eptr = NULL;
            int sn = strtol(version_lines[i] + plen, &eptr, 10);
            if(eptr != version_lines[i] + plen) {
                *serialno = sn;
                return 1;
            }
        }
    }
    return 0;
}

static gboolean
_connect_by_id(urg_t *urg, urg_parameter_t *params, const int desired_serialno)
{
    char **devnames = _get_acm_devnames();
    for(int i=0; devnames[i]; i++) {
        printf("Trying %s...\n", devnames[i]);
        const char *devname = devnames[i];

        if(!_connect_by_device(urg, params, devname)) {
            continue;
        }

        int serialno = -1;
        if(!_read_serialno(urg, &serialno)) {
            printf("Couldn't read serial number on %s\n", devname);
            urg_laserOff(urg);
            urg_disconnect(urg);
            continue;
        }

        if(desired_serialno == serialno) {
            printf("Found %d on %s\n", serialno, devname);
            g_strfreev(devnames);
            return 1;
        } else {
            printf("Skipping %s (found serial #: %d, desired: %d)\n", devname, serialno, desired_serialno);
            urg_laserOff(urg);
            urg_disconnect(urg);
        }
    }
    g_strfreev(devnames);
    return 0;
}

static gboolean
_connect_by_device_and_serial(urg_t *urg, urg_parameter_t *params, int *serials, int no_channels, 
                              const char *device, int *data_max, int *laser_ind)
{
    if(no_channels > 0) {
        if(device) {
            if(!_connect_by_device(urg, params, device)) {
                return 0;
            }

            int serialno = -1;
            if(!_read_serialno(urg, &serialno)) {
                printf("Couldn't read serial number on %s\n", device);
                urg_laserOff(urg);
                urg_disconnect(urg);
                return 0;
            }
            
            int found_serial = 0;

            for(int i=0; i < no_channels; i++){
                if(serials[i] == serialno) {
                    printf("Found %d on %s\n", serials[i], device);
                    *laser_ind = i;
                    found_serial = 1;
                    break; 
                }                     
            }
            
            if(!found_serial){
                urg_laserOff(urg);
                urg_disconnect(urg);
                return 0;
            }
        }
        else{
            fprintf(stderr, "No device specified\n");
            return 0;
        }
    }
    else{
        return 0;
    }

    if(data_max)
        *data_max = urg_dataMax(urg);

    // read and print out version information
    int LinesMax = 5;
    char version_buffer[LinesMax][UrgLineWidth];
    char *version_lines[LinesMax];
    for (int i = 0; i < LinesMax; ++i) {
        version_lines[i] = version_buffer[i];
    }
    int status = urg_versionLines(urg, version_lines, LinesMax);
    if (status < 0) {
        fprintf(stderr, "urg_versionLines: %s\n", urg_error(urg));
        urg_disconnect(urg);
        return 0;
    }
    for(int i = 0; i < LinesMax; ++i) {
        printf("%s\n", version_lines[i]);
    }
    printf("\n");


    // configure Hokuyo to continuous capture mode.
    urg_setCaptureTimes(urg, UrgInfinityTimes);

    // start data transmission
    status = urg_requestData(urg, URG_MD, URG_FIRST, URG_LAST);
    if (status < 0) {
        fprintf(stderr, "urg_requestData(): %s\n", urg_error(urg));
        urg_disconnect(urg);
        return 0;
    }

    return 1;
}

static gboolean
_connect(urg_t *urg, urg_parameter_t *params, int serialno,
         const char *device, int *data_max)
{
    if(serialno) {
        if(!_connect_by_id(urg, params, serialno)) {
            return 0;
        }
    } else if(device) {
        if(!_connect_by_device(urg, params, device))
            return 0;
    } else {
        if(!_connect_any_device(urg, params)) {
            return 0;
        }
    }

    if(data_max)
        *data_max = urg_dataMax(urg);

    // read and print out version information
    int LinesMax = 5;
    char version_buffer[LinesMax][UrgLineWidth];
    char *version_lines[LinesMax];
    for (int i = 0; i < LinesMax; ++i) {
        version_lines[i] = version_buffer[i];
    }
    int status = urg_versionLines(urg, version_lines, LinesMax);
    if (status < 0) {
        fprintf(stderr, "urg_versionLines: %s\n", urg_error(urg));
        urg_disconnect(urg);
        return 0;
    }
    for(int i = 0; i < LinesMax; ++i) {
        printf("%s\n", version_lines[i]);
    }
    printf("\n");

    //    //compute the offset between this computer's clock and the hokuyo's
    //    int64_t beforeTime;
    //    int64_t urgTime;
    //    int64_t afterTime;
    //    urg_enableTimestampMode(urg);
    //    printf("detirmining the offset hokuyo's clock\n");
    //    FILE * f = fopen("timeCalib.txt","w");
    //    for (int i=0;i<10000;i++){
    //      beforeTime=_timestamp_now();
    //      urgTime = urg_currentTimestamp(urg)*1e3;//urg timestamp is milliseconds
    //      afterTime=_timestamp_now();
    //      if (afterTime-beforeTime>.1e6)
    //        continue; //somethin weird happened... discard this sample
    //    }
    //    urg_disableTimestampMode(urg);

    // configure Hokuyo to continuous capture mode.
    urg_setCaptureTimes(urg, UrgInfinityTimes);

    // start data transmission
    status = urg_requestData(urg, URG_MD, URG_FIRST, URG_LAST);
    if (status < 0) {
        fprintf(stderr, "urg_requestData(): %s\n", urg_error(urg));
        urg_disconnect(urg);
        return 0;
    }

    return 1;
}

int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    char *optstring = "hc:d:p:i:af";
    char c;
    struct option long_opts[] = {
        {"help", no_argument, 0, 'h'},
        {"channel", required_argument, 0, 'c'},
        //{"config", required_argument, 0, 'c'},
        {"device", required_argument, 0, 'd'},
        {"id", required_argument, 0, 'i'},
        {"lcmurl", required_argument, 0, 'l'},
        {"auto-lookup", no_argument, 0, 'a'},
        {"flip", no_argument, 0, 'f'},
        {0, 0, 0, 0}
    };

    int exit_code = 0;
    char *device = NULL;
    char *channel = g_strdup("HOKUYO_LIDAR");
    char *lcm_url = NULL;
    int serialno = 0;
    //int lookup_id_from_channel = 0;
    int read_from_config = 0;
    int no_channels = 0;

    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
        {
            switch (c)
                {
                case 'c':
                    free(channel);
                    channel = g_strdup(optarg);                
                    fprintf(stderr,"Channel : %s\n", channel);
                    break;
                case 'd':
                    free(device);
                    device = g_strdup(optarg);
                    break;
                case 'p':
                    free(lcm_url);
                    lcm_url = g_strdup(optarg);
                    break;
                case 'i':
                    {
                        char *eptr = NULL;
                        serialno = strtol(optarg, &eptr, 10);
                        if(*eptr != '\0') {
                            usage(argv[0]);
                            return 1;
                        }
                    }
                    break;
                case 'a':
                    read_from_config = 1;
                    break;
                case 'f':
                    flip = 1;
                    fprintf(stderr,"Flipping laser readings\n");
                    break;
                case 'h':
                default:
                    usage(argv[0]);
                    return 1;
                }
        }

    char **planar_lidar_names = NULL;
    
    int data_max;
    long* data = NULL;
    urg_parameter_t urg_param;

    // setup LCM
    lcm_t *lcm = lcm_create(lcm_url);
    if(!lcm) {
        fprintf(stderr, "Couldn't setup LCM\n");
        return 1;
    }

    int *serials = NULL;

    if(read_from_config) {
        BotParam * param = bot_param_new_from_server(lcm, 1);

        if (param == NULL) {
            fprintf(stderr, "could not get params!\n");
            exit(1);
        }

        fprintf(stderr,"Getting Information from BotConfig\n");

        planar_lidar_names = bot_param_get_all_planar_lidar_names(param);     
      
        if(planar_lidar_names) {
            for (int pind = 0; planar_lidar_names[pind] != NULL; pind++) {
                fprintf(stderr, "Channel : %s\n", planar_lidar_names[pind]);
                no_channels++;
            }
        }

        serials = (int *) calloc(no_channels, sizeof(int));

        char cfg_var[256];
        for(int pind=0; pind < no_channels; pind++){
            snprintf(cfg_var, 256, "planar_lidars.%s.serial", planar_lidar_names[pind]);             serials[pind] = bot_param_get_int_or_fail(param, cfg_var); 
            fprintf(stderr, "Channel : %s Serial : %d\n" , planar_lidar_names[pind], serials[pind]);                  
        }
    }    

    urg_t urg;
    int max_initial_tries = 10;
    int connected = 0;
    int laser_ind = -1;

    for(int i=0; i<max_initial_tries && !connected; i++) {
        if(read_from_config){
            connected = _connect_by_device_and_serial(&urg, &urg_param, serials, no_channels , device, &data_max, &laser_ind);

            if(connected ==1){
                fprintf(stderr, "Found Laser : %s\n", planar_lidar_names[laser_ind]);
                if(channel !=NULL){
                    free(channel);
                }
                channel = strdup(planar_lidar_names[laser_ind]);

                //free laser channels
                g_strfreev(planar_lidar_names);
            }
        }
        else{
            connected = _connect(&urg, &urg_param, serialno, device, &data_max);
        }
        if(!connected) {
            struct timespec ts = { 0, 500000000 };
            nanosleep(&ts, NULL);
        }
    }
    if(!connected) {
        fprintf(stderr, "Unable to connect to any device\n");
        lcm_destroy(lcm);
        return 1;
    }

    // # of measurements per scan?
    data = (long*)malloc(sizeof(long) * data_max);
    if (data == NULL) {
        perror("data buffer");
        exit_code = 1;
        goto done;
    }

    bot_core_planar_lidar_t msg;
    int max_nranges = urg_param.area_max_ - urg_param.area_min_ + 1;
    msg.ranges = (float*) malloc(sizeof(float) * max_nranges);
    msg.nintensities = 0;
    msg.intensities = NULL;
    msg.radstep = 2.0 * M_PI / urg_param.area_total_;
    msg.rad0 = (urg_param.area_min_ - urg_param.area_front_) * msg.radstep;
    printf("Angular resolution: %f deg\n", TO_DEGREES(msg.radstep));
    printf("Starting angle:     %f deg\n", TO_DEGREES(msg.rad0));
    printf("Scan RPM:           %d\n", urg_param.scan_rpm_);
    printf("\n");

    int64_t now = _timestamp_now();
    int64_t report_last_utime = now;
    int64_t report_interval_usec = 2000000;
    int64_t next_report_utime = now + report_interval_usec;

    int64_t scancount_since_last_report = 0;
    int failure_count = 0;
    int reconnect_thresh = 10;
    int epic_fail = 0;
    int max_reconn_attempts = 600;

    //    int64_t before_receive = _timestamp_now();
    //    FILE * f = fopen("tstampLog.txt","w");


    // loop forever, reading scans
    while(!epic_fail) {
        //        before_receive = _timestamp_now();
        int nranges = urg_receiveData(&urg, data, data_max);
        now = _timestamp_now();

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
                    urg_disconnect(&urg);
                    connected = 0;
                    fprintf(stderr, "Comms failure.  Trying to reconnect...\n");
                }

                if(_connect(&urg, &urg_param, serialno, device, NULL)) {
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

        //        long timestamp = urg_recentTimestamp(&urg);
        //        fprintf(f,"T %lld %lld %ld\n",before_receive,now,timestamp);
        //        // TODO incorporate timestamp reported by hokuyo

        msg.utime = now;
        msg.nranges = nranges;
        
        //fprintf(stderr, "No of ranges : %d channel : %s\n", nranges, channel);
        
        for(int i=0; i<nranges; i++) {
            if(!flip) {
                msg.ranges[i] = data[i] * 1e-3;
            }
            else{//flipping readings
                msg.ranges[i] = data[nranges-i] * 1e-3;
            }
        } 

        bot_core_planar_lidar_t_publish(lcm, channel, &msg);

        scancount_since_last_report++;

        if(now > next_report_utime) {
            double dt = (now - report_last_utime) * 1e-6;

            printf("%4.1f Hz\n", scancount_since_last_report / dt);
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
    lcm_destroy(lcm);
    if(connected) {
        urg_laserOff(&urg);
        urg_disconnect(&urg);
    }

    free(data);
    free(lcm_url);
    free(channel);
    free(device);

    return exit_code;
}
