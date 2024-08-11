#ifndef __PARALLELCMDS_H
#define __PARALLELCMDS_H

#include <stdint.h> // uint8_t

typedef enum parallel_bus_mode {
    i80,
    m68
} parallel_bus_mode_t;

typedef enum parallel_bus_flags {
    PARBUS_SOFTWARE_MODE = 0x01,
    PARBUS_HARDWARE_MODE = 0x02,
} parallel_bus_flags_t;

typedef enum parallel_dev_flags {
    PARDEV_BUS_MODE         = 0x0F,
    PARDEV_CS_ACTIVE_HIGH   = 0x10,
    PARDEV_CS_HW_DRIVEN     = 0x20,
} parallel_dev_flags_t;

struct parallelbus_s {
    union {
        struct parallel_config* parallel_config;
        struct parallel_software* parallel_software;
    };
    uint8_t flags;
};

struct paralleldev_s {
    struct parallelbus_s* bus;
    void* bus_addr;
    uint32_t cs_pin;
    uint8_t flags;
    uint8_t data_width;
};

struct parallelbus_s *config_parallelbus(uint8_t oid);
struct parallelbus_s *parallelbus_oid_lookup(uint8_t oid);
struct paralleldev_s *paralleldev_oid_lookup(uint8_t oid);
void paralleldev_write(struct paralleldev_s *p_dev, uint32_t addr, uint8_t addr_inc, uint16_t data_len, const uint8_t *data);
void paralleldev_read(struct paralleldev_s *p_dev, uint32_t addr, uint8_t addr_inc, uint16_t data_len, uint8_t *data);

#endif // parallelcmds.h
