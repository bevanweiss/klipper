#ifndef __PARALLEL_SOFTWARE_H
#define __PARALLEL_SOFTWARE_H

#include <stdint.h> // uint8_t

void paralleldev_software_setup(struct paralleldev_s *dev);
void paralleldev_software_write(const struct paralleldev_s *dev, uint32_t dest_start, uint8_t dest_inc, uint8_t src_inc, uint16_t data_len, const uint8_t *src_data);
void paralleldev_software_read(const struct paralleldev_s *dev, uint32_t src_start, uint8_t src_inc, uint8_t dest_inc, uint16_t data_len, uint8_t *dest_data);

#endif // parallel_software.h
