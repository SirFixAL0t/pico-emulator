/*
 * TAP Interface for CYW43 WiFi Bridge
 *
 * Creates a Linux TAP virtual network interface and bridges
 * Ethernet frames between the emulated CYW43 and the host network.
 *
 * Usage: ./picoemu firmware.uf2 -wifi -tap <ifname>
 *
 * Linux only (requires /dev/net/tun).
 */

#ifndef TAPIF_H
#define TAPIF_H

#include <stdint.h>

/* Open a TAP interface. Returns fd or -1 on error. */
int tapif_open(const char *name);

/* Close TAP interface */
void tapif_close(int fd);

/* Read an Ethernet frame (non-blocking). Returns bytes read, 0 if none, -1 on error. */
int tapif_read(int fd, uint8_t *buf, int maxlen);

/* Write an Ethernet frame. Returns bytes written or -1 on error. */
int tapif_write(int fd, const uint8_t *buf, int len);

#endif /* TAPIF_H */
