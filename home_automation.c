// Copyright (c) 2015 Primiano Tucci -- www.primianotucci.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The name of Primiano Tucci may not be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "mongoose.h"


#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define ARRAY_SIZE(x) ((sizeof x) / (sizeof *x))

// Prototypes.
static bool REST_dimmer_get(const char* path, struct mg_connection *conn);
static bool REST_dimmer_post(const char* path, struct mg_connection *conn);
static bool REST_gpio_get(const char* path, struct mg_connection *conn);
static bool REST_gpio_post(const char* path, struct mg_connection *conn);

// Globals.
struct rest_endpoint_t {
  const char* uri;
  const char* method;
  bool (*handler)(const char*, struct mg_connection*);
};

static uint8_t dimmer_values[4];
static uint8_t io_values[3];
static const uint8_t gpio_pins[3] = {18, 16, 17};
static const char* tty;

const struct rest_endpoint_t endpoints[] = {
  {"/dimmer/", "GET",  REST_dimmer_get},
  {"/dimmer/", "POST", REST_dimmer_post},
  {"/io/",  "GET",  REST_gpio_get},
  {"/io/",  "POST", REST_gpio_post},
};


static void refresh_gpios() {
  uint8_t i;
  char path[64];
  for (i = 0; i < sizeof(io_values); ++i) {
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction",
          gpio_pins[i]);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
      perror("open");
      continue;
    }
    write(fd, "out\n", 4);
    close(fd);

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_pins[i]);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
      perror("open");
      continue;
    }
    write(fd, (io_values[i] ? "1\n" : "0\n"), 2);
    close(fd);
  }
}


static int refresh_dimmers(void) {
  uint8_t i;
  int fd = open(tty, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) {
    perror("Cannot open tty");
    return 1;
  }

  struct termios tio;
  if(tcgetattr(fd, &tio) < 0) {
    perror("tcgetattr");
    return 2;
  }

  tio.c_cflag &= ~(IGNBRK | BRKINT | ICRNL |
                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
  tio.c_oflag = 0;
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  tio.c_cflag &= ~(CSIZE | PARENB);
  tio.c_cflag |= CS8;

  if(cfsetispeed(&tio, B2400) < 0 || cfsetospeed(&tio, B2400) < 0) {
    perror("cfsetispeed");
    return 3;
  }

  if(tcsetattr(fd, TCSAFLUSH, &tio) < 0) {
    perror("tcsetattr");
    return 4;
  }

  uint8_t buf[sizeof(dimmer_values)];
  for (i = 0; i < sizeof(dimmer_values); ++i) {
    const uint8_t value = 0x3F - ((dimmer_values[i] >> 2) & 0x3F);
    buf[i] = value | (i << 6);
  }
  write(fd, buf, sizeof(buf));
  close(fd);
  return 0;
}


static bool REST_dimmer_get(const char* path, struct mg_connection *conn) {
  const uint8_t channel = atoi(path);
  if (channel >= sizeof(dimmer_values))
    return false;
  mg_printf_data(conn, "%d", dimmer_values[channel]);
  return true;
}


static bool REST_dimmer_post(const char* path, struct mg_connection *conn) {
  const uint8_t channel = atoi(path);
  if (channel >= sizeof(dimmer_values))
    return false;
  char val[4] = {0,0,0,0};
  strncpy(val, conn->content, MIN(conn->content_len, sizeof(val) - 1));
  dimmer_values[channel] = atoi(val);
  refresh_dimmers();
  mg_printf_data(conn, "OK\n");
  return true;
}


static bool REST_gpio_get(const char* path, struct mg_connection *conn) {
  const uint8_t channel = atoi(path);
  if (channel >= sizeof(io_values))
    return false;
  mg_printf_data(conn, "%d", io_values[channel]);
  return true;
}


static bool REST_gpio_post(const char* path, struct mg_connection *conn) {
  const uint8_t channel = atoi(path);
  if (channel >= sizeof(io_values))
    return false;
  io_values[channel] = (conn->content_len > 0 && conn->content[0] == '1') ? 1
                                                                          : 0;
  refresh_gpios();
  mg_printf_data(conn, "OK\n");
  return true;
}


static int http_handler(struct mg_connection *conn, enum mg_event ev) {
  uint32_t i;
  bool handled = false;
  switch (ev) {
    case MG_AUTH: return MG_TRUE;
    case MG_REQUEST:
      for (i = 0; i < ARRAY_SIZE(endpoints) && !handled; ++i) {
        const struct rest_endpoint_t* ep = &endpoints[i];
        const size_t uri_len = strlen(ep->uri);
        if (strcmp(conn->request_method, ep->method) == 0 &&
            strncmp(conn->uri, ep->uri, uri_len) == 0) {
          mg_send_header(conn, "Access-Control-Allow-Origin", "*");
          if (ep->handler(&conn->uri[uri_len], conn))
            return MG_TRUE;
        }
      }
      return MG_FALSE;
    default:
      return MG_FALSE;
  }
}


static void pulse_heartbeat_leds() {
  static int prev_value = 0;
  static uint8_t idx = 0;
  static char pattern[] = "11100000111111000000000000000000";

  int fd = open("/sys/class/leds/pwm3/brightness", O_WRONLY);
  if (fd < 0)
    return;
  int value = pattern[idx] == '1' ? 4096 : 0;
  value = value / 10 + prev_value * 9 / 10;
  prev_value = value;
  idx = (idx + 1) % sizeof(pattern);
  char value_str[32];
  snprintf(value_str, sizeof(value_str), "%d", value);
  write(fd, value_str, strlen(value_str));
  close(fd);
}


int main(int argc, char** argv) {
  if (argc < 3) {
    printf("Usage: %s /dev/ttymxc3 /data/www/root\n", argv[0]);
    return -1;
  }

  tty = argv[1];

  struct mg_server *server;
  server = mg_create_server(NULL, http_handler);
  mg_set_option(server, "listening_port", "8080");
  mg_set_option(server, "document_root", argv[2]);

  printf("Starting on port %s\n", mg_get_option(server, "listening_port"));
  int i;
  for (;;) {
    mg_poll_server(server, 60);
    pulse_heartbeat_leds();
    // Refresh values every ~sec to re-trigger the dimmer's watchdog.
    if (++i == 15) {
      refresh_dimmers();
      refresh_gpios();
      i = 0;
    }
  }
  mg_destroy_server(&server);
  return 0;
}
