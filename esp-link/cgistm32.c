// Copyright (c) 2015 by Thorsten von Eicken, see LICENSE.txt in the esp-link repo

// Some code moved to esp-link/pgmshared.c to avoid code duplication.
// Those changes are Copyright (c) 2017 by Danny Backx.

// Protocol used : https://github.com/Optiboot/optiboot/wiki/HowOptibootWorks

#include <esp8266.h>
#include "cgi.h"
#include "cgistm32.h"
#include "config.h"
#include "uart.h"
#include "stk500.h"
#include "serbridge.h"
#include "mqtt_cmd.h"
#include "serled.h"

#include "pgmshared.h"

#define INIT_DELAY     150   // wait this many millisecs before sending anything
#define BAUD_INTERVAL  600   // interval after which we change baud rate
#define PGM_TIMEOUT  20000   // timeout after sync is achieved, in milliseconds
#define PGM_INTERVAL   200   // send sync at this interval in ms when in programming mode
#define ATTEMPTS         8   // number of attempts total to make

#ifdef OPTIBOOT_DBG
#define DBG(format, ...) do { os_printf(format, ## __VA_ARGS__); } while(0)
#else
#define DBG(format, ...) do { } while(0)
#endif

#define DBG_GPIO5 1 // define to 1 to use GPIO5 to trigger scope


static void ICACHE_FLASH_ATTR optibootInit() {

  DBG("STM32 init\n");
}


//===== Cgi to reset AVR and get Optiboot into sync
int ICACHE_FLASH_ATTR cgiSTM32Sync(HttpdConnData *connData) {
  if (connData->conn==NULL) return HTTPD_CGI_DONE; // Connection aborted. Clean up.

  // check that we know the reset pin, else error out with that
  if (flashConfig.reset_pin < 0) {
    errorResponse(connData, 400, "No reset pin defined");

  } else if (connData->requestType == HTTPD_METHOD_POST) {
    // issue reset
    //optibootInit();
    mqtt_block(); // prevent MQTT from interfering
    //baudRate = flashConfig.baud_rate;


    // start sync timer
    //os_timer_disarm(&optibootTimer);
    //os_timer_setfn(&optibootTimer, optibootTimerCB, NULL);
    //os_timer_arm(&optibootTimer, INIT_DELAY, 0);

    // respond with optimistic OK
    noCacheHeaders(connData, 204);
    httpdEndHeaders(connData);
    httpdSend(connData, "", 0);

  } else if (connData->requestType == HTTPD_METHOD_GET) {
    noCacheHeaders(connData, 200);
    //httpdEndHeaders(connDoptibootInit();ata);
    /*
    if (!errMessage[0] && progState >= stateProg) {
      char buf[64];
      DBG("OB got sync\n");
      os_sprintf(buf, "SYNC at %d baud: bootloader v%d.%d",
          baudRate, optibootVers>>8, optibootVers&0xff);
      httpdSend(connData, buf, -1);
    } else if (errMessage[0] && progState == stateSync) {
      DBG("OB cannot sync\n");
      char buf[512];
      os_sprintf(buf, "FAILED to SYNC: %s, got: %d chars\r\n", errMessage, responseLen);
      appendPretty(buf, 512, responseBuf, responseLen);
      httpdSend(connData, buf, -1);
    } else {
      httpdSend(connData, errMessage[0] ? errMessage : "NOT READY", -1);
    }*/

  } else {
    errorResponse(connData, 404, "Only GET and POST supported");
  }

  return HTTPD_CGI_DONE;
}

//===== Cgi to write firmware to Optiboot, requires prior sync call
int ICACHE_FLASH_ATTR cgiSTM32Data(HttpdConnData *connData) {
  if (connData->conn==NULL) return HTTPD_CGI_DONE; // Connection aborted. Clean up.
  //if (!optibootData)
  //  DBG("OB pgm: state=%d postLen=%d\n", progState, connData->post->len);

  // check that we have sync
  /*
  if (errMessage[0] || progState < stateProg) {
    DBG("OB not in sync, state=%d, err=%s\n", progState, errMessage);
    errorResponse(connData, 400, errMessage[0] ? errMessage : "Optiboot not in sync");
    return HTTPD_CGI_DONE;
  }*/

  // check that we don't have two concurrent programming requests going on
  if (connData->cgiPrivData == (void *)-1) {
    DBG("OB aborted\n");
    errorResponse(connData, 400, "Request got aborted by a concurrent sync request");
    return HTTPD_CGI_DONE;
  }
/*
  // allocate data structure to track programming
  if (!optibootData) {
    //optibootData = os_zalloc(sizeof(struct optibootData));
    //char *saved = os_zalloc(MAX_SAVED+1); // need space for string terminator
    //char *pageBuf = os_zalloc(MAX_PAGE_SZ+MAX_SAVED/2);
    if (!optibootData || !pageBuf || !saved) {
      errorResponse(connData, 400, "Out of memory");
      return HTTPD_CGI_DONE;
    }
    optibootData->mega = false;
    optibootData->pageBuf = pageBuf;
    optibootData->saved = saved;
    optibootData->startTime = system_get_time();
    optibootData->pgmSz = 128; // hard coded for 328p for now, should be query string param
    DBG("OB data alloc\n");
  }*/

  // iterate through the data received and program the AVR one block at a time
  HttpdPostData *post = connData->post;
  char *saved = optibootData->saved;
  while (post->buffLen > 0) {
    // first fill-up the saved buffer
    short saveLen = strlen(saved);
    /*
    if (saveLen < MAX_SAVED) {
      short cpy = MAX_SAVED-saveLen;
      if (cpy > post->buffLen) cpy = post->buffLen;
      os_memcpy(saved+saveLen, post->buff, cpy);
      saveLen += cpy;
      saved[saveLen] = 0; // string terminator
      os_memmove(post->buff, post->buff+cpy, post->buffLen-cpy);
      post->buffLen -= cpy;
      //DBG("OB cp %d buff->saved\n", cpy);
    }*/

    // process HEX records
    while (saveLen >= 11) { // 11 is minimal record length
      // skip any CR/LF
      short skip = 0;
      while (skip < saveLen && (saved[skip] == '\n' || saved[skip] == '\r'))
        skip++;
      if (skip > 0) {
        // shift out cr/lf (keep terminating \0)
        os_memmove(saved, saved+skip, saveLen+1-skip);
        saveLen -= skip;
        if (saveLen < 11) break;
        DBG("OB skip %d cr/lf\n", skip);
      }

      // inspect whether we have a proper record start
      if (saved[0] != ':') {
        DBG("OB found non-: start\n");
        os_sprintf(errMessage, "Expected start of record in POST data, got %s", saved);
        errorResponse(connData, 400, errMessage);
        optibootInit();
        return HTTPD_CGI_DONE;
      }

      if (!checkHex(saved+1, 2)) {
        errorResponse(connData, 400, errMessage);
        optibootInit();
        return HTTPD_CGI_DONE;
      }
      uint8_t recLen = getHexValue(saved+1, 2);
      //DBG("OB record %d\n", recLen);

      // process record
      if (saveLen >= 11+recLen*2) {
        if (!processRecord(saved, 11+recLen*2)) {
          DBG("OB process err %s\n", errMessage);
          errorResponse(connData, 400, errMessage);
          optibootInit();
          return HTTPD_CGI_DONE;
        }
        short shift = 11+recLen*2;
        os_memmove(saved, saved+shift, saveLen+1-shift);
        saveLen -= shift;
        //DBG("OB %d byte record\n", shift);
      } else {
        break;
      }
    }
  }

  short code;
  if (post->received < post->len) {
    //DBG("OB pgm need more\n");
    return HTTPD_CGI_MORE;
  }

  if (optibootData->eof) {
    // tell optiboot to reboot into the sketch
    uart0_write_char(STK_LEAVE_PROGMODE);
    uart0_write_char(CRC_EOP);
    code = 200;
    // calculate some stats
    /*
    float dt = ((system_get_time() - optibootData->startTime)/1000)/1000.0; // in seconds
    uint16_t pgmDone = optibootData->pgmDone;
    optibootInit();*/
    /*
    os_sprintf(errMessage, "Success. %d bytes at %d baud in %d.%ds, %dB/s %d%% efficient",
        pgmDone, baudRate, (int)dt, (int)(dt*10)%10, (int)(pgmDone/dt),
        (int)(100.0*(10.0*pgmDone/baudRate)/dt));*/
  } else {
    code = 400;
    optibootInit();
    os_strcpy(errMessage, "Improperly terminated POST data");
  }
  DBG("OB pgm done: %d -- %s\n", code, errMessage);
  noCacheHeaders(connData, code);
  httpdEndHeaders(connData);
  httpdSend(connData, errMessage, -1);
  errMessage[0] = 0;
  return HTTPD_CGI_DONE;
}
