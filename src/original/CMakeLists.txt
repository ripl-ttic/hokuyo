add_definitions(-std=gnu99)

set(progname hokuyo)
add_executable(${progname}
    hokuyo.c
    liburg/serial_ctrl.c
    liburg/serial_utils.c
    liburg/ring_buffer.c
    liburg/serial_errno.h
    liburg/delay.c
    liburg/ticks.c
    liburg/urg_ctrl.c
    liburg/scip_handler.c
    liburg/urg_errno.c
    liburg/scip_handler.h
    liburg/urg_errno.h)

pods_use_pkg_config_packages(${progname}
    bot2-core bot2-param-client lcm)

pods_install_executables(${progname})
