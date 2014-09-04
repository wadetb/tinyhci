/*
All of tinyhci is licensed under the MIT license.

Copyright (c) 2014 by Wade Brainerd <wadeb@wadeb.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#ifndef __TINYHCI_H__
#define __TINYHCI_H__

//
// Debugging macros
//
// Define any of these as 'x' instead of empty and recompile tinyhci to enable debug serial output.
// 
// DEBUG_LV1 - Reserved for the application.
// DEBUG_LV2 - Logs API functions and their arguments.
// DEBUG_LV3 - Logs low level HCI actions, including interrupts.
// DEBUG_LV4 - Logs every byte transferred via SPI to/from the CC3000.
//
#define DEBUG_LV1(x) 
#define DEBUG_LV2(x) 
#define DEBUG_LV3(x) 
#define DEBUG_LV4(x) 

//
// Serial port helper macros.
//
#define SERIAL_PRINT(x)            Serial.print(x); Serial.flush()
#define SERIAL_PRINTLN(x)          Serial.println(x); Serial.flush()
#define SERIAL_PRINTFUNCTION()     Serial.print("==> "); Serial.print(__FUNCTION__); Serial.println(" <=="); Serial.flush()
#define SERIAL_PRINTVAR(x)         Serial.print(#x ": "); Serial.println(x); Serial.flush()
#define SERIAL_PRINTVAR_HEX(x)     Serial.print(#x ": "); Serial.println(x, HEX); Serial.flush()

typedef struct _in_addr_t
{
    uint32_t         s_addr;                // load with inet_aton()
} in_addr;

typedef struct _sockaddr_t
{
    uint16_t         sa_family;
    uint8_t          sa_data[14];
} sockaddr;

typedef struct _sockaddr_in_t
{
    int16_t          sin_family;            // e.g. AF_INET
    uint16_t         sin_port;              // e.g. htons(3490)
    in_addr          sin_addr;              // see struct in_addr, below
    uint8_t          sin_zero[8];           // zero this if you want to
} sockaddr_in;

#define htons(a) ((((uint16_t)(a) & 0xff00) >> 8) | (((uint16_t)(a) & 0x00ff) << 8))

#define MDNS_DEVICE_SERVICE_MAX_LENGTH  32
#define MAXIMAL_SSID_LENGTH             32

#define WLAN_SEC_UNSEC             0
#define WLAN_SEC_WEP               1
#define WLAN_SEC_WPA               2
#define WLAN_SEC_WPA2              3

#define AF_INET                		2

#define SOCK_STREAM            		1
#define SOCK_DGRAM             		2
#define SOCK_RAW               		3

#define IPPROTO_TCP            		6
#define IPPROTO_UDP            		17
#define IPPROTO_RAW            		255

#define SOL_SOCKET             		0xFFFF 	// socket level

#define SOCKOPT_RECV_NONBLOCK       0 		// recv non block mode, set SOCK_ON or SOCK_OFF (default block mode)
#define SOCKOPT_RECV_TIMEOUT     	1 		// optname to configure recv and recvfromtimeout
#define SOCKOPT_ACCEPT_NONBLOCK     2 		// accept non block mode, set SOCK_ON or SOCK_OFF (default block mode)

#define SOCK_ON                   	0     	// socket non-blocking mode is enabled    
#define SOCK_OFF                  	1     	// socket blocking mode is enabled

extern volatile uint8_t wifi_connected;
extern volatile uint8_t wifi_dhcp;
extern volatile uint8_t ip_addr[4];

extern volatile int16_t client_socket;

void wlan_init(void);
long netapp_timeout_values(unsigned long *aucDHCP, unsigned long *aucARP, unsigned long *aucKeepalive, unsigned long *aucInactivity);
int32_t wlan_ioctl_set_connection_policy(bool should_connect_to_open_ap, bool should_use_fast_connect, bool use_profiles);
int32_t wlan_connect(unsigned long sec_type, const char *ssid, long ssid_len, unsigned char *bssid, unsigned char *key, long key_len);
int setsockopt(long sd, long level, long optname, const void *optval, unsigned long optlen);
int socket(long domain, long type, long protocol);
int listen(int sd, int backlog);
int bind(int sd, struct _sockaddr_t *addr, int addrlen);
int accept(int sd, struct sockaddr_t *addr, unsigned long *addrlen);
int recv(int sd, uint8_t *buffer, int size, int flags);
int send(int sd, uint8_t *buffer, int size, int flags);
int closesocket(int sd);
int mdnsAdvertiser(unsigned short mdnsEnabled, char *deviceServiceName, unsigned short deviceServiceNameLength);

#endif
