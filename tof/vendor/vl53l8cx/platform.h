/*
 * Linux i2c-dev platform layer for the ST VL53L8CX ULD — microduck.
 *
 * Replaces ST's platform.h template (BSD-3-Clause, see LICENSE.txt).
 * Talks to /dev/i2c-N via I2C_RDWR ioctls (../platform.c, shared with the
 * VL53L5CX generation); used by shim.c, which exposes the flat API
 * `tof::Sensor` wraps.
 *
 * The six hook names below are rewritten per generation by the build, so both
 * ULDs can link into one binary — see ../platform.c.
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

typedef struct
{
    /* 8-bit I2C address (7-bit << 1), the format the ULD expects. */
    uint16_t address;
    /* Open file descriptor on /dev/i2c-N, or on /dev/spidevB.C when `spi` is set. */
    int fd;
    /* 1: the descriptor is a spidev and `address` is unused. Only the VL53L8CX has
     * an SPI interface; the field is in both structs because ../platform.c is one
     * implementation compiled against either. */
    uint8_t spi;
} VL53L8CX_Platform;

/*
 * Four targets per zone, ST's maximum (UM2884/UM3109 §4.10): a zone straddling
 * an edge reports the near surface and the one behind it when they are at
 * least 600 mm apart, instead of whichever returned more light. Must match
 * `tof::TARGETS`.
 */
#define VL53L8CX_NB_TARGET_PER_ZONE 4U

/*
 * Output trim: `tof.frame` carries distance + target_status per target, and
 * the shim needs nb_target_detected to tell a target from an empty slot, so
 * every other output block is disabled here — the sensor firmware then does
 * not even emit them. Four targets make each 8x8 readout ~850 B of bus traffic
 * (~250 B with one): at 15 Hz ~17% of a 400 kHz bus, against ~5%, which the
 * codec sharing the bus on a zero3 has to live with. Re-enabling a block means
 * widening the wire format to match.
 */
#define VL53L8CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L8CX_DISABLE_NB_SPADS_ENABLED
#define VL53L8CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L8CX_DISABLE_RANGE_SIGMA_MM
/* VL53L8CX_DISABLE_DISTANCE_MM      — kept */
#define VL53L8CX_DISABLE_REFLECTANCE_PERCENT
/* VL53L8CX_DISABLE_TARGET_STATUS   — kept */
#define VL53L8CX_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
               uint8_t *p_value);
uint8_t WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
               uint8_t value);
uint8_t RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                uint8_t *p_values, uint32_t size);
uint8_t WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                uint8_t *p_values, uint32_t size);
void SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs);

#endif /* _PLATFORM_H_ */
