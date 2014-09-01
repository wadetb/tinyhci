#include <Arduino.h>
#include <SPI.h>
#include "tinyhci.h"

#define WLAN_SSID      ""
#define WLAN_PW        ""
#define WLAN_SECURITY  WLAN_SEC_WPA2
#define WLAN_TIMEOUT   30000

#define WEB_PORT       80

uint8_t recv_buf[256];
uint8_t recv_buf_idx = 0;
int recv_buf_avail = 0;

void wifi_connect(void)
{
  wlan_ioctl_set_connection_policy(false, false, false);
  wlan_connect(WLAN_SECURITY, WLAN_SSID, strlen(WLAN_SSID), 0, (unsigned char*)WLAN_PW, strlen(WLAN_PW));
    
  int time = millis();
  while (!wifi_dhcp) 
  {
    if ((millis() - time) > WLAN_TIMEOUT)
    {
      SERIAL_PRINTLN("TIMED OUT.");
      return;
    }
  }
}

void wifi_listen(void)
{
  // Set the CC3000 inactivity timeout to 0 (never timeout).  This will ensure 
  // the CC3000 does not close the listening socket when it's idle for more than 
  // 60 seconds (the default timeout).  See more information from:
  // http://e2e.ti.com/support/low_power_rf/f/851/t/292664.aspx
  unsigned long aucDHCP       = 14400;
  unsigned long aucARP        = 3600;
  unsigned long aucKeepalive  = 30;
  unsigned long aucInactivity = 0;
  if (netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity) != 0) 
    return;

  listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_socket < 0)
    return;

  char arg = SOCK_ON;
  if (setsockopt(listen_socket, SOL_SOCKET, SOCKOPT_ACCEPT_NONBLOCK, &arg, sizeof(arg)) < 0) 
    return;

  sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = 0;
  address.sin_port = htons(WEB_PORT);
  if (bind(listen_socket, (sockaddr *)&address, sizeof(address)) < 0) 
    return;
  
  if (listen(listen_socket, 0) < 0) 
    return;
}

void send_mem_with_length(char *text, int len)
{
  if (client_socket < 0)
    return;

  send(client_socket, (uint8_t *)text, len, 0);
}

void send_mem(char *text)
{
  send_mem_with_length(text, strlen(text));
}

uint8_t tx_buf[256];
uint8_t tx_buf_zero;

void send_flash(const __FlashStringHelper *ifsh)
{
  int idx = 0;

  if (client_socket < 0)
    return;
  
  const char PROGMEM *p = (const char PROGMEM *)ifsh;
  for (;;) 
  {
    unsigned char c = pgm_read_byte(p++);
    
    if (c == 0) 
      break;
      
    tx_buf[idx] = c;
    idx++;
    if (idx >= sizeof(tx_buf)) 
    {
      if (client_socket < 0)
        return;
      send(client_socket, tx_buf, sizeof(tx_buf), 0);
      idx = 0;
    }
  }
  
  if (idx > 0) 
  {
    if (client_socket < 0)
      return;
    send(client_socket, tx_buf, idx, 0);
  }
}

uint8_t read_byte(void) 
{
  if (client_socket < 0)
    return 0;

  while (recv_buf_idx >= recv_buf_avail) 
  {
    recv_buf_avail = recv(client_socket, recv_buf, sizeof(recv_buf), 0);
    DEBUG_LV2(SERIAL_PRINTVAR(recv_buf_avail));
    if (recv_buf_avail < 0)
    {
      closesocket(client_socket);
      client_socket = -1;
      return 0;
    }
    recv_buf_idx = 0;
  }

  uint8_t result = recv_buf[recv_buf_idx];
  recv_buf_idx++;
  
  return result;
}

void wifi_accept(void)
{
  recv_buf_idx = 0;
  recv_buf_avail = 0;

  client_socket = accept(listen_socket, NULL, NULL);
  if (client_socket < 0) 
    return;

  char request[128];
  int request_len = 0;
  request[request_len] = 0;
  while (client_socket >= 0 && strchr(request, '\r') == 0 && request_len < sizeof(request))
  {
    request[request_len++] = read_byte();
    request[request_len] = 0;
  }

  if (client_socket < 0) 
    return;

  if (strncmp(request, "GET /", 5) != 0)
    goto fail;

  if (char *p = strchr(request, '\r')) *p = 0;
  if (char *p = strchr(request, '\n')) *p = 0;
  if (char *p = strstr(request, " HTTP")) *p = 0;

  send_flash(F(
    "HTTP/1.1 200 OK\n"
    "Content-Type: text/html\n"
    "Connection: close\n"
    "\n"
    "<!doctype html>\n"
    "<html>\n"
    "<head>\n"
    "<title>tinyhci server test</title>\n"
    "</head>\n"
    "<body>\n"
    "Hello from tinyhci!\n"
    "</body>\n"
    "</html>\n"));

fail:;
  if (client_socket >= 0)
    closesocket(client_socket);

#if 1
  // Workaround for second accept returning -1.
  closesocket(listen_socket);
  wifi_listen();
#endif
}

void setup()
{
  wifi_init();
  wifi_connect();
  wifi_listen();  
}

void loop()
{
  wifi_accept();
}

