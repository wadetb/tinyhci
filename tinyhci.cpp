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
#include <Arduino.h>
#include <SPI.h>
#include "tinyhci.h"

#define CC3K_CS_PIN    6
#define CC3K_IRQ_PIN   7
#define CC3K_EN_PIN    8
#define CC3K_IRQ_NUM   4

uint8_t wifi_connected = 0;
uint8_t wifi_dhcp = 0;
uint8_t ip_addr[4];

int16_t listen_socket = -1;
int16_t client_socket = -1;

volatile uint8_t hci_data_available;
volatile uint8_t hci_pending_event_available;
volatile uint16_t hci_pending_event = 0xffff;

int buffer_size;
int buffer_count;
volatile int avail_buffer_count;

uint16_t hci_payload_size;

#define HCI_STATE_IDLE          0
#define HCI_STATE_WAIT_ASSERT   1

volatile uint8_t hci_state;

uint8_t hci_pad;
uint16_t rx_payload_size;

void hci_begin_receive();
void hci_end_receive();
void hci_dispatch();

#define READ                    3
#define WRITE                   1

#define HCI_TYPE_CMND          0x1
#define HCI_TYPE_DATA          0x2
#define HCI_TYPE_PATCH         0x3
#define HCI_TYPE_EVNT          0x4

#define HCI_EVENT_PATCHES_DRV_REQ     (1)
#define HCI_EVENT_PATCHES_FW_REQ      (2)
#define HCI_EVENT_PATCHES_BOOTLOAD_REQ    (3)

#define HCI_CMND_WLAN_BASE  (0x0000)
#define HCI_CMND_WLAN_CONNECT  0x0001
#define HCI_CMND_WLAN_DISCONNECT   0x0002
#define HCI_CMND_WLAN_IOCTL_SET_SCANPARAM    0x0003
#define HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY  0x0004
#define HCI_CMND_WLAN_IOCTL_ADD_PROFILE  0x0005
#define HCI_CMND_WLAN_IOCTL_DEL_PROFILE  0x0006
#define HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS  0x0007
#define HCI_CMND_EVENT_MASK    0x0008
#define HCI_CMND_WLAN_IOCTL_STATUSGET 0x0009
#define HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START        0x000A
#define HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP         0x000B
#define HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX   0x000C
#define HCI_CMND_WLAN_CONFIGURE_PATCH         0x000D

#define HCI_CMND_SOCKET_BASE   0x1000
#define HCI_CMND_SOCKET        0x1001
#define HCI_CMND_BIND          0x1002
#define HCI_CMND_RECV          0x1004
#define HCI_CMND_ACCEPT        0x1005
#define HCI_CMND_LISTEN        0x1006
#define HCI_CMND_CONNECT       0x1007
#define HCI_CMND_BSD_SELECT   0x1008
#define HCI_CMND_SETSOCKOPT    0x1009
#define HCI_CMND_GETSOCKOPT    0x100A
#define HCI_CMND_CLOSE_SOCKET  0x100B
#define HCI_CMND_RECVFROM      0x100D
#define HCI_CMND_GETHOSTNAME   0x1010
#define HCI_CMND_MDNS_ADVERTISE    0x1011


#define HCI_DATA_BASE               0x80

#define HCI_CMND_SEND                     (0x01 + HCI_DATA_BASE)
#define HCI_CMND_SENDTO                   (0x03 + HCI_DATA_BASE)
#define HCI_DATA_BSD_RECVFROM           (0x04 + HCI_DATA_BASE)
#define HCI_DATA_BSD_RECV             (0x05 + HCI_DATA_BASE)

#define HCI_CMND_NVMEM_CBASE    (0x0200)


#define HCI_CMND_NVMEM_CREATE_ENTRY (0x0203)
#define HCI_CMND_NVMEM_SWAP_ENTRY   (0x0205)
#define HCI_CMND_NVMEM_READ       (0x0201)
#define HCI_CMND_NVMEM_WRITE      (0x0090)
#define HCI_CMND_NVMEM_WRITE_PATCH  (0x0204)
#define HCI_CMND_READ_SP_VERSION    (0x0207)

#define  HCI_CMND_READ_BUFFER_SIZE  0x400B
#define  HCI_CMND_SIMPLE_LINK_START 0x4000

#define HCI_CMND_NETAPP_BASE    0x2000

#define HCI_NETAPP_DHCP       (0x0001 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_PING_SEND        (0x0002 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_PING_REPORT      (0x0003 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_PING_STOP        (0x0004 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_IPCONFIG         (0x0005 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_ARP_FLUSH    (0x0006 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_SET_DEBUG_LEVEL  (0x0008 + HCI_CMND_NETAPP_BASE)
#define HCI_NETAPP_SET_TIMERS   (0x0009 + HCI_CMND_NETAPP_BASE)

#define HCI_EVNT_WLAN_BASE     0x0000
#define HCI_EVNT_WLAN_CONNECT  0x0001
#define HCI_EVNT_WLAN_DISCONNECT 0x0002
#define HCI_EVNT_WLAN_IOCTL_ADD_PROFILE 0x0005

#define HCI_EVNT_SEND          0x1003
#define HCI_EVNT_WRITE         0x100E
#define HCI_EVNT_SENDTO        0x100F

#define HCI_EVNT_PATCHES_REQ    0x1000

#define HCI_EVNT_UNSOL_BASE    0x4000

#define HCI_EVNT_WLAN_UNSOL_BASE     (0x8000)

#define HCI_EVNT_WLAN_UNSOL_CONNECT    (0x0001 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_UNSOL_DISCONNECT   (0x0002 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_UNSOL_INIT         (0x0004 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_TX_COMPLETE         (0x0008 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_UNSOL_DHCP         (0x0010 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_ASYNC_PING_REPORT  (0x0040 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE  (0x0080 + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_KEEPALIVE      (0x0200  + HCI_EVNT_WLAN_UNSOL_BASE)
#define HCI_EVNT_WLAN_UNSOL_TCP_CLOSE_WAIT      (0x0800 + HCI_EVNT_WLAN_UNSOL_BASE)

#define HCI_EVNT_DATA_UNSOL_FREE_BUFF 0x4100

#define HCI_EVNT_NVMEM_WRITE    (0x0202)

#define HCI_EVNT_INPROGRESS     0xFFFF

#define HCI_DATA_RECVFROM       0x84
#define HCI_DATA_RECV           0x85
#define HCI_DATA_NVMEM          0x91

#define HCI_EVENT_CC3000_CAN_SHUT_DOWN 0x99

#define PATCHES_HOST_TYPE_WLAN_DRIVER   0x01
#define PATCHES_HOST_TYPE_WLAN_FW       0x02
#define PATCHES_HOST_TYPE_BOOTLOADER    0x03

#define SL_SET_SCAN_PARAMS_INTERVAL_LIST_SIZE (16)
#define SL_SIMPLE_CONFIG_PREFIX_LENGTH  (3)
#define ETH_ALEN                    (6)
#define MAXIMAL_SSID_LENGTH             (32)

#define SL_PATCHES_REQUEST_DEFAULT    (0)
#define SL_PATCHES_REQUEST_FORCE_HOST (1)
#define SL_PATCHES_REQUEST_FORCE_NONE (2)

#define HCI_ATTR __attribute__((noinline))

HCI_ATTR 
unsigned char hci_transfer(unsigned char out) 
{ 
  unsigned char in = SPI.transfer(out); 
  DEBUG_LV4(
    Serial.print("SPI: ");
    Serial.print(out, HEX);
    Serial.print(" -> ");
    Serial.print(in, HEX);
    Serial.println();
    Serial.flush());
  return in;
}

HCI_ATTR 
uint8_t hci_read_u8()
{
  if (hci_payload_size > 0)
  {
    hci_payload_size--;
    return hci_transfer(0);
  }
  else
  {
    return 0;
  }
}

HCI_ATTR 
uint16_t hci_read_u16_le()
{
  uint8_t b0 = hci_read_u8();
  uint8_t b1 = hci_read_u8();
  return b0 | (b1 << 8);
}

HCI_ATTR 
uint32_t hci_read_u32_le()
{
  uint8_t b0 = hci_read_u8();
  uint8_t b1 = hci_read_u8();
  uint8_t b2 = hci_read_u8();
  uint8_t b3 = hci_read_u8();
  return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

HCI_ATTR 
void hci_read_array(uint8_t *data, uint16_t length)
{
  while (length)
  {
    *data = hci_read_u8();
    data++;
    length--;
  }
}

HCI_ATTR 
uint8_t hci_write_u8(uint8_t v)
{
  if (hci_payload_size > 0)
  {
    hci_payload_size--;
    hci_transfer(v);
  }
}

HCI_ATTR 
void hci_write_u16_le(uint16_t v)
{
  hci_write_u8((v >>  0) & 0xff);
  hci_write_u8((v >>  8) & 0xff);
}

HCI_ATTR 
void hci_write_u32_le(uint32_t v)
{
  hci_write_u8((v >>  0) & 0xff);
  hci_write_u8((v >>  8) & 0xff);
  hci_write_u8((v >> 16) & 0xff);
  hci_write_u8((v >> 24) & 0xff);
}

HCI_ATTR 
void hci_write_array(uint8_t *data, uint16_t length)
{
  while (length)
  {
    hci_write_u8(*data);
    data++;
    length--;
  }
}

HCI_ATTR 
void hci_irq()
{
  DEBUG_LV3(SERIAL_PRINTFUNCTION());
  if (hci_state == HCI_STATE_WAIT_ASSERT)
  {
    hci_state = HCI_STATE_IDLE;
  }
  else 
  {
    hci_begin_receive();
    hci_dispatch();
  }
}

HCI_ATTR 
void hci_begin_first_command(uint16_t opcode, uint16_t argsSize)
{ 
  DEBUG_LV3(SERIAL_PRINTFUNCTION());

  // First Host Write Operation

  // 1. The master detects the IRQ line low: in this case the detection of IRQ low does not 
  //    indicate the intention of the CC3000 device to communicate with the master but rather 
  //    CC3000 readiness after power up.
  uint16_t start = millis();
  while (digitalRead(CC3K_IRQ_PIN) != LOW)
  {
    if (millis() - start >= 5000)
    {
      SERIAL_PRINTLN("Failed to detect CC3000.  Check wiring?");
      for (;;);
    }
  }

  // 2. The master asserts nCS.
  digitalWrite(CC3K_CS_PIN, LOW);

  // 3. The master introduces a delay of at least 50 μs before starting actual transmission of data.
  delay(50);

  // 4. The master transmits the first 4 bytes of the SPI header.
  uint16_t pad = !(argsSize & 1);
  uint16_t payloadSize = 4 + argsSize + pad;
  hci_transfer(WRITE);
  hci_transfer(payloadSize >> 8);
  hci_transfer(payloadSize & 0xff);
  hci_transfer(0);

  // 5. The master introduces a delay of at least an additional 50 μs.
  delay(50);

  // 6. The master transmits the rest of the packet.
  hci_transfer(0);
  hci_transfer(HCI_TYPE_CMND);
  hci_transfer(opcode & 0xff);
  hci_transfer(opcode >> 8);
  hci_transfer(argsSize);
}

HCI_ATTR 
void hci_begin_command(uint16_t opcode, uint16_t argsSize)
{ 
  DEBUG_LV3(SERIAL_PRINTFUNCTION());

  // Generic Host Write Operation
  hci_state = HCI_STATE_WAIT_ASSERT;

  // 1. The master asserts nCS (that is, drives the signal low) and waits for IRQ assertion.
  digitalWrite(CC3K_CS_PIN, LOW);

  // 2. The CC3000 device asserts IRQ when ready to receive the data.
  while (hci_state != HCI_STATE_IDLE)
    ;

  // 3. The master starts the write transaction. The write transaction consists of a 5-byte header
  //    followed by the payload and a padding byte (if required: remember, the total packet length
  //    must be 16-bit aligned).
  hci_pad = (argsSize & 1) == 0;
  hci_payload_size = 4 + argsSize + hci_pad;
  hci_transfer(WRITE);
  hci_transfer(hci_payload_size >> 8);
  hci_transfer(hci_payload_size & 0xff);
  hci_transfer(0);
  hci_transfer(0);

  hci_transfer(HCI_TYPE_CMND);
  hci_transfer(opcode & 0xff);
  hci_transfer(opcode >> 8);
  hci_transfer(argsSize);
}

HCI_ATTR 
void hci_end_command_begin_receive(uint16_t event)
{
  hci_pending_event = event;
  hci_pending_event_available = 0;
  hci_data_available = 0;

  if (hci_pad)
    hci_transfer(0);

  // 4. After the last byte of data, the nCS is deasserted by the master.
  digitalWrite(CC3K_CS_PIN, HIGH);

  // 5. The CC3000 device deasserts the IRQ line.

  while (!hci_pending_event_available)
    ;
}

HCI_ATTR 
void hci_begin_data(uint16_t opcode, uint8_t argsSize, uint16_t bufferSize)
{ 
  DEBUG_LV3(SERIAL_PRINTFUNCTION());

  // Generic Host Write Operation

  // 1. The master asserts nCS (that is, drives the signal low) and waits for IRQ assertion.
  hci_state = HCI_STATE_WAIT_ASSERT;

  digitalWrite(CC3K_CS_PIN, LOW);

  // 2. The CC3000 device asserts IRQ when ready to receive the data.
  while (hci_state != HCI_STATE_IDLE)
    ;

  // 3. The master starts the write transaction. The write transaction consists of a 5-byte header
  //    followed by the payload and a padding byte (if required: remember, the total packet length
  //    must be 16-bit aligned).
  int totalSize = argsSize + bufferSize;
  hci_pad = (totalSize & 1) != 0;
  hci_payload_size = 4 + totalSize + hci_pad;
  hci_transfer(WRITE);
  hci_transfer(hci_payload_size >> 8);
  hci_transfer(hci_payload_size);
  hci_transfer(0);
  hci_transfer(0);

  hci_transfer(HCI_TYPE_DATA);
  hci_transfer(opcode);
  hci_transfer(argsSize);
  hci_transfer(totalSize);
  hci_transfer(totalSize >> 8);
}

#define hci_end_data_begin_receive hci_end_command_begin_receive

HCI_ATTR 
void hci_dispatch_event(void)
{
  uint16_t rx_event_type = hci_read_u16_le();
  DEBUG_LV3(SERIAL_PRINTVAR_HEX(rx_event_type));

  uint8_t rx_args_size = hci_read_u8();
  DEBUG_LV3(SERIAL_PRINTVAR(rx_args_size));

  if (rx_event_type == hci_pending_event)
  {
    hci_pending_event_available = 1;
  }
  else
  {
    switch (rx_event_type)
    {
    case HCI_EVNT_WLAN_UNSOL_CONNECT:
      wifi_connected = 1;
      DEBUG_LV3(SERIAL_PRINTVAR(wifi_connected));
      break;
    
    case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
      wifi_connected = 0;
      wifi_dhcp = 0;
      DEBUG_LV3(SERIAL_PRINTVAR(wifi_connected));
      DEBUG_LV3(SERIAL_PRINTVAR(wifi_dhcp));
      break;
    
    case HCI_EVNT_WLAN_UNSOL_DHCP:
      wifi_dhcp = 1;
      DEBUG_LV3(SERIAL_PRINTVAR(wifi_dhcp));
      hci_read_u8(); // status
      ip_addr[3] = hci_read_u8();
      ip_addr[2] = hci_read_u8();
      ip_addr[1] = hci_read_u8();
      ip_addr[0] = hci_read_u8();
      break;

    case HCI_EVNT_WLAN_UNSOL_TCP_CLOSE_WAIT:
      client_socket = -1;
      DEBUG_LV3(SERIAL_PRINTVAR(client_socket));
      break;
    
    case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
      {
        hci_read_u8(); // status
        int fce_count = hci_read_u16_le();
        for (int i = 0; i < fce_count; i++)
        {
          hci_read_u16_le(); // ??
          avail_buffer_count += hci_read_u16_le();
        }
        DEBUG_LV3(SERIAL_PRINTVAR(avail_buffer_count));
      }
      break;
    
    default:
      break;
    }

    hci_end_receive();
  }
}

HCI_ATTR 
void hci_dispatch_data(void)
{
  uint8_t rx_data_type = hci_read_u8();
  DEBUG_LV3(SERIAL_PRINTVAR_HEX(rx_data_type));

  uint8_t rx_args_size = hci_read_u8();
  DEBUG_LV3(SERIAL_PRINTVAR(rx_args_size));

  rx_payload_size = hci_read_u16_le();
  DEBUG_LV3(SERIAL_PRINTVAR(rx_payload_size));

  for (int i = 0; i < rx_args_size; i++)
    hci_read_u8();

  hci_data_available = 1;
}

HCI_ATTR 
void hci_dispatch(void)
{
  DEBUG_LV3(SERIAL_PRINTFUNCTION());

  uint8_t rx_type = hci_read_u8();
  DEBUG_LV3(SERIAL_PRINTVAR(rx_type));

  if (rx_type == HCI_TYPE_EVNT)
    hci_dispatch_event();
  else if (rx_type == HCI_TYPE_DATA)
    hci_dispatch_data();
}

HCI_ATTR 
void hci_begin_receive()
{
  DEBUG_LV3(SERIAL_PRINTFUNCTION());

  // Master Read Transaction

  // 1. The IRQ line is asserted by the CC3000 device.

  // 2. The master asserts the nCS line.
  digitalWrite(CC3K_CS_PIN, LOW);

  // 3. The master transmits the following 3 bytes: read opcode followed by two busy bytes
  hci_transfer(READ);
  hci_transfer(0);
  hci_transfer(0);

  // 4. The CC3000 sends back the following data: the first two bytes indicate the payload length 
  //    and the data payload bytes follow, immediately after.
  uint8_t p0 = hci_transfer(0);
  uint8_t p1 = hci_transfer(0);

  hci_payload_size = (p0 << 8) | p1;
  DEBUG_LV3(SERIAL_PRINTVAR(hci_payload_size));
}

HCI_ATTR 
void hci_end_receive()
{
  DEBUG_LV3(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(hci_payload_size);
    )

  // Read and discard any unread portion of the message.
  while (hci_payload_size)
    hci_read_u8();

  // 5. At the end of read transaction, the master drives nCS inactive.
  digitalWrite(CC3K_CS_PIN, HIGH);

  // 6. The CC3000 device deasserts an IRQ line.
  while (digitalRead(CC3K_IRQ_PIN) == LOW)
    ;
}

HCI_ATTR 
void hci_wait_data()
{
  DEBUG_LV3(SERIAL_PRINTFUNCTION());

  while (!hci_data_available)
    ;
}

HCI_ATTR 
void hci_read_status()
{
  int status = hci_read_u8();
  DEBUG_LV2(SERIAL_PRINTVAR(status));
}

HCI_ATTR 
uint32_t hci_end_command_receive_u32_result(uint16_t event)
{
  hci_end_command_begin_receive(event);
  
  hci_read_status();

  uint32_t result = hci_read_u32_le();
  DEBUG_LV3(SERIAL_PRINTVAR(result));

  hci_end_receive();

  return result;
}

void wifi_init(void)
{ 
  DEBUG_LV1(SERIAL_PRINTFUNCTION());
  delay(1000);

  pinMode(CC3K_EN_PIN, OUTPUT);
  digitalWrite(CC3K_EN_PIN, LOW);
  delay(500);

  pinMode(CC3K_CS_PIN, OUTPUT);

  pinMode(CC3K_IRQ_PIN, INPUT_PULLUP);
  
  digitalWrite(CC3K_CS_PIN, HIGH);
  digitalWrite(CC3K_EN_PIN, HIGH);
  delay(100);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  delay(100);

  hci_begin_first_command(HCI_CMND_SIMPLE_LINK_START, 1);
  hci_write_u8(SL_PATCHES_REQUEST_DEFAULT);
  attachInterrupt(CC3K_IRQ_NUM, hci_irq, FALLING);
  hci_end_command_begin_receive(HCI_CMND_SIMPLE_LINK_START);
  hci_end_receive();

  hci_begin_command(HCI_CMND_READ_BUFFER_SIZE, 0);
  hci_end_command_begin_receive(HCI_CMND_READ_BUFFER_SIZE);
  hci_read_status();
  buffer_count = hci_read_u8();
  avail_buffer_count = buffer_count;
  DEBUG_LV2(SERIAL_PRINTVAR(buffer_count));
  buffer_size = hci_read_u16_le();
  DEBUG_LV2(SERIAL_PRINTVAR(buffer_size));
  hci_end_receive();

  hci_begin_command(HCI_CMND_EVENT_MASK, 4);
  hci_write_u32_le(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT);
  hci_end_command_begin_receive(HCI_CMND_EVENT_MASK);
  hci_end_receive();
}

#define MIN_TIMER_VAL_SECONDS      20
#define MIN_TIMER_SET(t)    if ((0 != t) && (t < MIN_TIMER_VAL_SECONDS)) \
                            { \
                                t = MIN_TIMER_VAL_SECONDS; \
                            }

long netapp_timeout_values(unsigned long *aucDHCP, unsigned long *aucARP, unsigned long *aucKeepalive, unsigned long *aucInactivity)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(*aucDHCP);
    SERIAL_PRINTVAR(*aucARP);
    SERIAL_PRINTVAR(*aucKeepalive);
    SERIAL_PRINTVAR(*aucInactivity);
    )

  MIN_TIMER_SET(*aucDHCP)
  MIN_TIMER_SET(*aucARP)
  MIN_TIMER_SET(*aucKeepalive)
  MIN_TIMER_SET(*aucInactivity)
  
  hci_begin_command(HCI_NETAPP_SET_TIMERS, 16);
  hci_write_u32_le(*aucDHCP);
  hci_write_u32_le(*aucARP);
  hci_write_u32_le(*aucKeepalive);
  hci_write_u32_le(*aucInactivity);

  return hci_end_command_receive_u32_result(HCI_NETAPP_SET_TIMERS);
}

int32_t wlan_ioctl_set_connection_policy(bool should_connect_to_open_ap, bool should_use_fast_connect, bool use_profiles)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(should_connect_to_open_ap);
    SERIAL_PRINTVAR(should_use_fast_connect);
    SERIAL_PRINTVAR(use_profiles);
    )

  hci_begin_command(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY, 12);
  hci_write_u32_le(should_connect_to_open_ap);
  hci_write_u32_le(should_use_fast_connect);
  hci_write_u32_le(use_profiles);

  return hci_end_command_receive_u32_result(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY);
}

int32_t wlan_connect(unsigned long sec_type, const char *ssid, long ssid_len, unsigned char *bssid, unsigned char *key, long key_len)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sec_type);
    SERIAL_PRINTVAR(ssid);
    SERIAL_PRINTVAR(ssid_len);
    SERIAL_PRINTVAR((char*)bssid);
    SERIAL_PRINTVAR((char*)key);
    SERIAL_PRINTVAR(key_len);
    )

  static unsigned char bssid_zero[6] = {0, 0, 0, 0, 0, 0};

  hci_begin_command(HCI_CMND_WLAN_CONNECT, 28 + ssid_len + key_len);
  hci_write_u32_le(0x1c);
  hci_write_u32_le(ssid_len);
  hci_write_u32_le(sec_type);
  hci_write_u32_le(16 + ssid_len);
  hci_write_u32_le(key_len);
  hci_write_u16_le(0);
  if (bssid)
    hci_write_array(bssid, 6);
  else
    hci_write_array(bssid_zero, 6);
  hci_write_array((uint8_t*)ssid, ssid_len);
  if (key_len && key)
    hci_write_array(key, key_len);

  return hci_end_command_receive_u32_result(HCI_CMND_WLAN_CONNECT);
}

int setsockopt(long sd, long level, long optname, const void *optval, unsigned long optlen)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    SERIAL_PRINTVAR(level);
    SERIAL_PRINTVAR(optname);
    SERIAL_PRINTVAR_HEX((int)optval);
    SERIAL_PRINTVAR(optlen);
    )
  
  hci_begin_command(HCI_CMND_SETSOCKOPT, 20 + optlen);
  hci_write_u32_le(sd);
  hci_write_u32_le(level);
  hci_write_u32_le(optname);
  hci_write_u32_le(8);
  hci_write_u32_le(optlen);
  hci_write_array(((uint8_t *)optval), optlen);

  return hci_end_command_receive_u32_result(HCI_CMND_SETSOCKOPT);
}

int socket(long domain, long type, long protocol)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(domain);
    SERIAL_PRINTVAR(type);
    SERIAL_PRINTVAR(protocol);
    )

  hci_begin_command(HCI_CMND_SOCKET, 12);
  hci_write_u32_le(domain);
  hci_write_u32_le(type);
  hci_write_u32_le(protocol);

  return hci_end_command_receive_u32_result(HCI_CMND_SOCKET);
}

int listen(int sd, int backlog)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    SERIAL_PRINTVAR(backlog);
    )
  
  hci_begin_command(HCI_CMND_LISTEN, 8);
  hci_write_u32_le(sd);
  hci_write_u32_le(backlog);

  return hci_end_command_receive_u32_result(HCI_CMND_LISTEN);
}

int bind(int sd, struct _sockaddr_t *addr, int addrlen)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    SERIAL_PRINTVAR(addrlen);
    SERIAL_PRINTVAR(((_sockaddr_in_t*)addr)->sin_family);
    SERIAL_PRINTVAR(((_sockaddr_in_t*)addr)->sin_port);
    SERIAL_PRINTVAR_HEX(((_sockaddr_in_t*)addr)->sin_addr.s_addr);
    )
  
  hci_begin_command(HCI_CMND_BIND, 20);
  hci_write_u32_le(sd);
  hci_write_u32_le(0x8);
  hci_write_u32_le(addrlen);
  hci_write_array(((uint8_t *)addr), 8);

  return hci_end_command_receive_u32_result(HCI_CMND_BIND);
}

int accept(int sd, struct sockaddr_t *addr, unsigned long *addrlen)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    )
    
  hci_begin_command(HCI_CMND_ACCEPT, 4);
  hci_write_u32_le(sd);

  hci_end_command_begin_receive(HCI_CMND_ACCEPT);

  hci_read_status();

  uint32_t return_sd = hci_read_u32_le();
  DEBUG_LV2(SERIAL_PRINTVAR(return_sd));

  int32_t return_status = hci_read_u32_le();
  DEBUG_LV2(SERIAL_PRINTVAR(return_status));

  if (addr)
    hci_read_array((uint8_t*)addr, 8);
  if (addrlen) 
    *addrlen = 8;

  hci_end_receive();

  // Return status is actually the socket descriptor.
  if (return_status < 0 || return_status >= 8)
    return -1;

  return return_status;
}

int recv(int sd, uint8_t *buffer, int size, int flags)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    SERIAL_PRINTVAR(size);
    SERIAL_PRINTVAR(flags);
    )
  
  hci_begin_command(HCI_CMND_RECV, 12);
  hci_write_u32_le(sd);
  hci_write_u32_le(size);
  hci_write_u32_le(flags);

  hci_end_command_begin_receive(HCI_CMND_RECV);
  
  hci_read_status();

  long return_sd = hci_read_u32_le();
  DEBUG_LV2(SERIAL_PRINTVAR(return_sd));

  long return_length = hci_read_u32_le();
  DEBUG_LV2(SERIAL_PRINTVAR(return_length));

  long return_flags = hci_read_u32_le();
  DEBUG_LV2(SERIAL_PRINTVAR_HEX(return_flags));

  hci_end_receive();

  if (return_length > 0)
  {
    // TODO: while waiting for data, we need to handle unsolicited client drops
    //  and stop waiting.
    // Also for safety, the interrupt handler should be able to ignore 
    //  unsolicited data when we're not waiting.
    // Thus, the intention to wait for data must be indicated to hci_end_receive, 
    //  e.g. hci_end_receive_with_data.
    hci_wait_data();

    if (return_length > size)
      return_length = size;

    for (int i = 0; i < return_length; i++)
      buffer[i] = hci_read_u8();

    hci_end_receive();

    if (return_length < size)
    {
      buffer[return_length] = 0;
      DEBUG_LV2(SERIAL_PRINTVAR((char*)buffer));
    }
  }

  return return_length;
}

int send(int sd, uint8_t *buffer, int size, int flags)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    SERIAL_PRINTVAR(size);
    SERIAL_PRINTVAR(flags);
    )

  DEBUG_LV3(SERIAL_PRINTVAR(avail_buffer_count));
  while (avail_buffer_count == 0)
    ;
  avail_buffer_count--;
  
  hci_begin_data(HCI_CMND_SEND, 16, size);
  hci_write_u32_le(sd);
  hci_write_u32_le(12);
  hci_write_u32_le(size);
  hci_write_u32_le(flags);
  hci_write_array(buffer, size);
  hci_end_data_begin_receive(HCI_EVNT_SEND);
  hci_end_receive();

  return size;
}

int closesocket(int sd)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(sd);
    )
  
  while (avail_buffer_count != buffer_count)
    ;
    
  hci_begin_command(HCI_CMND_CLOSE_SOCKET, 4);
  hci_write_u32_le(sd);

  return hci_end_command_receive_u32_result(HCI_CMND_CLOSE_SOCKET);
}

int mdnsAdvertiser(unsigned short mdnsEnabled, char *deviceServiceName, unsigned short deviceServiceNameLength)
{
  DEBUG_LV2(
    SERIAL_PRINTFUNCTION();
    SERIAL_PRINTVAR(mdnsEnabled);
    SERIAL_PRINTVAR(deviceServiceName);
    SERIAL_PRINTVAR(deviceServiceNameLength);
    )

  if (deviceServiceNameLength > MDNS_DEVICE_SERVICE_MAX_LENGTH)
    return -1;
    
  hci_begin_command(HCI_CMND_MDNS_ADVERTISE, 12 + deviceServiceNameLength);
  hci_write_u32_le(mdnsEnabled);
  hci_write_u32_le(8);
  hci_write_u32_le(deviceServiceNameLength);
  hci_write_array((uint8_t*)deviceServiceName, deviceServiceNameLength);

  return hci_end_command_receive_u32_result(HCI_CMND_MDNS_ADVERTISE);
}
