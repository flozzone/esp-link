// Copyright (c) 2015 by Thorsten von Eicken, see LICENSE.txt in the esp-link repo

#ifndef CGISTM32_H
#define CGISTM32_H

#include <httpd.h>

int ICACHE_FLASH_ATTR cgiSTM32Sync(HttpdConnData *connData);
int ICACHE_FLASH_ATTR cgiSTM32Data(HttpdConnData *connData);

#endif /* CGISTM32_H */
