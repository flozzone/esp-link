/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include "stm32serial.h"
#include "stm32port.h"

static port_err_t serial_esp_open(struct port_interface *port,
				    struct port_options *ops)
{
    // TODO: open

	return PORT_ERR_OK;
}

static port_err_t serial_esp_close(struct port_interface *port)
{
	// TODO: close

	return PORT_ERR_OK;
}

static port_err_t serial_esp_read(struct port_interface *port, void *buf,
				     size_t nbyte)
{
	// TODO: read n bytes

	return PORT_ERR_OK; // PORT_ERR_TIMEDOUT, PORT_ERR_UNKNOWN
}

static port_err_t serial_esp_write(struct port_interface *port, void *buf,
				      size_t nbyte)
{
    // TODO: write n bytes

	return PORT_ERR_OK; // PORT_ERR_UNKNOWN
}

static port_err_t serial_esp_gpio(struct port_interface *port,
				    serial_gpio_t n, int level)
{
	serial_t *h;

	h = (serial_t *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	switch (n) {
	case GPIO_RTS:
		//bit = TIOCM_RTS;
		break;

	case GPIO_DTR:
		//bit = TIOCM_DTR;
		break;

	case GPIO_BRK:
		if (level == 0)
			return PORT_ERR_OK;
		//if (tcsendbreak(h->fd, 1))
		//	return PORT_ERR_UNKNOWN;
		return PORT_ERR_OK;

	default:
		return PORT_ERR_UNKNOWN;
	}

	/* TODO: handle RTS/DTR */
	/*
	if (ioctl(h->fd, TIOCMGET, &lines))
		return PORT_ERR_UNKNOWN;
	lines = level ? lines | bit : lines & ~bit;
	if (ioctl(h->fd, TIOCMSET, &lines))
		return PORT_ERR_UNKNOWN;*/

	return PORT_ERR_OK;
}

static const char *serial_esp_get_cfg_str(struct port_interface *port)
{
    /*
	serial_t *h;
	h = (serial_t *)port->private;
     */
	return "serial_esp_get_cfg_str";
}

static port_err_t serial_esp_flush(struct port_interface *port)
{
	// TODO: flush uart

	return PORT_ERR_OK;
}

struct port_interface port_serial = {
	.name	= "serial_esp",
	.flags	= PORT_BYTE | PORT_GVR_ETX | PORT_CMD_INIT | PORT_RETRY,
	.open	= serial_esp_open,
	.close	= serial_esp_close,
	.flush  = serial_esp_flush,
	.read	= serial_esp_read,
	.write	= serial_esp_write,
	.gpio	= serial_esp_gpio,
	.get_cfg_str	= serial_esp_get_cfg_str,
};
