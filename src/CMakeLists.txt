add_definitions(-std=gnu99)

set(progname hr-hokuyo)
add_executable(${progname}
    main.c
    urglib/urg_sensor.h
    urglib/urg_sensor.c
    urglib/urg_connection.h
    urglib/urg_connection.c
    urglib/urg_serial.h
    urglib/urg_serial.c
    urglib/urg_tcpclient.h
    urglib/urg_tcpclient.c
    urglib/urg_ring_buffer.h
    urglib/urg_ring_buffer.c
    urglib/urg_errno.h
    urglib/urg_utils.h
    urglib/urg_utils.c)

pods_use_pkg_config_packages(${progname}
    bot2-core bot2-param-client lcm)

pods_install_executables(${progname})
