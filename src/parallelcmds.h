#ifndef __PARALLELCMDS_H
#define __PARALLELCMDS_H

#include <stdint.h> // uint8_t
#include "board/gpio.h"

typedef enum parallel_bus_mode {
    intel8080,
    motorola6800
} parallel_bus_mode_t;

typedef enum parallel_bus_flags {
    PARBUS_SOFTWARE_MODE = 0x01,
    PARBUS_HARDWARE_MODE = 0x02,
} parallel_bus_flags_t;

typedef enum parallel_dev_flags {
    PARDEV_BUS_MODE_MSK     = 0x000F,
    PARDEV_HAS_CS           = 0x0010,
    PARDEV_CS_ACTIVE_HIGH   = 0x0020,
    PARDEV_CS_HW_DRIVEN     = 0x0040,
} parallel_dev_flags_t;

struct parallelbus_s {
    union {
        struct parallel_hw_config* hw_config;
        struct parallel_sw_config* sw_config;
    };
    uint16_t flags;
};

struct paralleldev_s {
    union {
        struct gpio_out cs_pin;
        uint32_t cs_pin_def;
    };
    struct parallelbus_s* bus;
    void* bus_addr;
    uint16_t flags;
    uint8_t data_width;
};

struct parallelbus_s *config_parallelbus(uint8_t oid);
struct parallelbus_s *parallelbus_oid_lookup(uint8_t oid);
struct paralleldev_s *paralleldev_oid_lookup(uint8_t oid);

#endif // parallelcmds.h