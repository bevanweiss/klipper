#ifndef __PARALLEL_SOFTWARE_H
#define __PARALLEL_SOFTWARE_H

#include <stdint.h> // uint8_t

void paralleldev_software_setup(struct paralleldev_s *dev);
void paralleldev_software_write(struct paralleldev_s *dev, uint32_t addr_start, uint8_t addr_inc, uint16_t data_len, uint8_t *data);
void paralleldev_software_read(struct paralleldev_s *dev, uint32_t addr_start, uint8_t addr_inc, uint16_t data_len, uint8_t *data);

#endif // parallel_software.h
