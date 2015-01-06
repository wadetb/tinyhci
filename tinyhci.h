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
#define SERIAL_PORT                Serial
#define SERIAL_PRINT(x)            SERIAL_PORT.print(x); SERIAL_PORT.flush()
#define SERIAL_PRINTLN(x)          SERIAL_PORT.println(x); SERIAL_PORT.flush()
#define SERIAL_PRINTFUNCTION()     SERIAL_PORT.print("==> "); SERIAL_PORT.print(__FUNCTION__); SERIAL_PORT.println(" <=="); SERIAL_PORT.flush()
#define SERIAL_PRINTVAR(x)         SERIAL_PORT.print(#x ": "); SERIAL_PORT.println(x); SERIAL_PORT.flush()
#define SERIAL_PRINTVAR_HEX(x)     SERIAL_PORT.print(#x ": "); SERIAL_PORT.println(x, HEX); SERIAL_PORT.flush()

//
// HCI interface constants
//
#define HCI_STATE_IDLE                          0
#define HCI_STATE_WAIT_ASSERT                   1

#define HCI_READ                                0x3
#define HCI_WRITE                               0x1

#define HCI_TYPE_CMND                           0x1
#define HCI_TYPE_DATA                           0x2
#define HCI_TYPE_PATCH                          0x3
#define HCI_TYPE_EVNT                           0x4

//
// HCI Command IDs
//
#define HCI_CMND_WLAN_CONNECT                   0x0001
#define HCI_CMND_WLAN_DISCONNECT                0x0002
#define HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY 0x0004
#define HCI_CMD_WLAN_IOCTL_DEL_PROFILE          0x0006
#define HCI_CMND_EVENT_MASK                     0x0008

#define HCI_CMND_SEND                           0x0081
#define HCI_CMND_SENDTO                         0x0083
#define HCI_DATA_BSD_RECVFROM                   0x0084
#define HCI_DATA_BSD_RECV                       0x0085

#define HCI_CMND_SOCKET                         0x1001
#define HCI_CMND_BIND                           0x1002
#define HCI_CMND_RECV                           0x1004
#define HCI_CMND_RECVFROM                       0x100D
#define HCI_CMND_ACCEPT                         0x1005
#define HCI_CMND_LISTEN                         0x1006
#define HCI_CMND_CONNECT                        0x1007
#define HCI_CMND_SELECT                         0x1008
#define HCI_CMND_SETSOCKOPT                     0x1009
#define HCI_CMND_CLOSE_SOCKET                   0x100B
#define HCI_CMND_GETHOSTNAME                    0x1010
#define HCI_CMND_MDNS_ADVERTISE                 0x1011

#define HCI_NETAPP_SET_TIMERS                   0x2009
#define HCI_NETAPP_IPCONFIG                     0x2005
#define HCI_CMND_READ_SP_VERSION                0x0207

#define HCI_CMND_SIMPLE_LINK_START              0x4000
#define HCI_CMND_READ_BUFFER_SIZE               0x400B

//
// HCI Data commands
//
#define HCI_DATA_RECVFROM                       0x84
#define HCI_DATA_RECV                           0x85
#define HCI_DATA_NVMEM                          0x91
//
// HCI Event IDs
//
#define HCI_EVNT_DATA_SEND                      0x1003
#define HCI_EVNT_DATA_SENDTO                    0x100F
#define HCI_EVNT_DATA_UNSOL_FREE_BUFF           0x4100
#define HCI_EVNT_WLAN_UNSOL_CONNECT             0x8001
#define HCI_EVNT_WLAN_UNSOL_DISCONNECT          0x8002
#define HCI_EVNT_WLAN_UNSOL_INIT                0x8004
#define HCI_EVNT_WLAN_UNSOL_DHCP                0x8010
#define HCI_EVNT_WLAN_KEEPALIVE                 0x8200
#define HCI_EVNT_WLAN_UNSOL_TCP_CLOSE_WAIT      0x8800

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

typedef unsigned long socklen_t;

// ipconfig
typedef struct _netapp_ipconfig_t
{
  uint32_t ipAddr;
  uint32_t subnet;
  uint32_t gateway;
  uint32_t DHCPServer;
  uint32_t DNSServer;
  uint8_t macAddr[6];
  uint8_t ssid[32];
} netapp_ipconfig_t;

#ifdef __AVR__
typedef unsigned long time_t;  /* KTown: Updated to be compatible with Arduino Time.h */
#else
typedef long time_t;
#endif

typedef long suseconds_t;

typedef struct _timeval_t
{
  time_t         tv_sec;                  /* seconds */
  suseconds_t    tv_usec;                 /* microseconds */
} timeval;

// The fd_set member is required to be an array of longs.
typedef long int __fd_mask;

#define __FD_SETSIZE            32

// It's easier to assume 8-bit bytes than to get CHAR_BIT.
#define __NFDBITS               (8 * sizeof (__fd_mask))
#define __FDELT(d)              ((d) / __NFDBITS)
#define __FDMASK(d)             ((__fd_mask) 1 << ((d) % __NFDBITS))

#ifdef fd_set
#undef fd_set  // for compatibility with newlib, which defines fd_set
#endif

// fd_set for select and pselect.
typedef struct
{
  __fd_mask fds_bits[__FD_SETSIZE / __NFDBITS];
#define __FDS_BITS(set)        ((set)->fds_bits)
} fd_set;

// We don't use `memset' because this would require a prototype and
//   the array isn't too big.
#define __FD_ZERO(set)                               \
  do {                                                \
    unsigned int __i;                                 \
    fd_set *__arr = (set);                            \
    for (__i = 0; __i < sizeof (fd_set) / sizeof (__fd_mask); ++__i) \
      __FDS_BITS (__arr)[__i] = 0;                    \
  } while (0)
#define __FD_SET(d, set)       (__FDS_BITS (set)[__FDELT (d)] |= __FDMASK (d))
#define __FD_CLR(d, set)       (__FDS_BITS (set)[__FDELT (d)] &= ~__FDMASK (d))
#define __FD_ISSET(d, set)     (__FDS_BITS (set)[__FDELT (d)] & __FDMASK (d))

// Access macros for 'fd_set'.
#ifdef FD_SET
#undef FD_SET
#endif
#ifdef FD_CLR
#undef FD_CLR
#endif
#ifdef FD_ISSET
#undef FD_ISSET
#endif
#ifdef FD_ZERO
#undef FD_ZERO
#endif
#define FD_SET(fd, fdsetp)      __FD_SET (fd, fdsetp)
#define FD_CLR(fd, fdsetp)      __FD_CLR (fd, fdsetp)
#define FD_ISSET(fd, fdsetp)    __FD_ISSET (fd, fdsetp)
#define FD_ZERO(fdsetp)         __FD_ZERO (fdsetp)

#define htons(a) ((((uint16_t)(a) & 0xff00) >> 8) | (((uint16_t)(a) & 0x00ff) << 8))

#define ntohs                   htons

#define htonl(A)    ((((unsigned long)(A) & 0xff000000) >> 24) | \
                     (((unsigned long)(A) & 0x00ff0000) >> 8) | \
                     (((unsigned long)(A) & 0x0000ff00) << 8) | \
                     (((unsigned long)(A) & 0x000000ff) << 24))

#define ntohl                   htonl

#define MDNS_DEVICE_SERVICE_MAX_LENGTH  32
#define MAXIMAL_SSID_LENGTH             32

#define WLAN_SEC_UNSEC             0
#define WLAN_SEC_WEP               1
#define WLAN_SEC_WPA               2
#define WLAN_SEC_WPA2              3

#define INADDR_ANY                  0

#define AF_INET                		2

#define SOCK_STREAM            		1
#define SOCK_DGRAM             		2
#define SOCK_RAW               		3

#define IPPROTO_TCP            		6
#define IPPROTO_UDP            		17
#define IPPROTO_RAW            		255

#define SOL_SOCKET             		0xFFFF  // socket level

#define SOCKOPT_RECV_NONBLOCK       0 		// recv non block mode, set SOCK_ON or SOCK_OFF (default block mode)
#define SOCKOPT_RECV_TIMEOUT     	1 		// optname to configure recv and recvfromtimeout
#define SOCKOPT_ACCEPT_NONBLOCK     2 		// accept non block mode, set SOCK_ON or SOCK_OFF (default block mode)

#define SOCK_ON                   	0     // socket non-blocking mode is enabled
#define SOCK_OFF                  	1     // socket blocking mode is enabled

#define HOSTNAME_MAX_LENGTH        230   // 230 bytes + header shouldn't exceed 8 bit value

extern volatile uint8_t wifi_connected;
extern volatile uint8_t wifi_dhcp;
extern volatile uint32_t ip_addr;


//*****************************************************************************
//                  ERROR CODES
//*****************************************************************************
#define ESUCCESS        0
#define EFAIL          -1
#define EERROR          EFAIL

void wlan_init(void);
long netapp_timeout_values(unsigned long *aucDHCP, unsigned long *aucARP, unsigned long *aucKeepalive, unsigned long *aucInactivity);
int32_t wlan_ioctl_set_connection_policy(bool should_connect_to_open_ap, bool should_use_fast_connect, bool use_profiles);
int32_t wlan_ioctl_del_profile(uint32_t profile_id);
int32_t wlan_connect(unsigned long sec_type, const char *ssid, long ssid_len, unsigned char *bssid, unsigned char *key, long key_len);
int32_t wlan_disconnect();
int connect(long sd, const sockaddr *addr, long addrlen);
int setsockopt(long sd, long level, long optname, const void *optval, unsigned long optlen);
int socket(long domain, long type, long protocol);
int listen(int sd, int backlog);
int bind(int sd, struct _sockaddr_t *addr, int addrlen);
int accept(int sd, struct sockaddr_t *addr, unsigned long *addrlen);
int recv(int sd, void *buffer, int size, int flags);
int recvfrom(int fd, void *buffer, int size, int flags, sockaddr *remaddr, socklen_t *len);
int send(int sd, const void *buffer, int size, int flags);
int sendto(int sd, const void *buffer, int size, int flags, const sockaddr *to, socklen_t tolen);
int select(long nfds, fd_set *readsds, fd_set *writesds, fd_set *exceptsds, timeval *timeout);
int closesocket(int sd);
int mdnsAdvertiser(unsigned short mdnsEnabled, char *deviceServiceName, unsigned short deviceServiceNameLength);
int gethostbyname(char *url, unsigned short len, unsigned long *ip);

int netappipconfig(netapp_ipconfig_t *ipconfig);
uint16_t getFirmwareVersion();

#endif

