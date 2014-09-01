#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_CC3000.h>
#include <Adafruit_CC3000_Server.h>

#define WLAN_SSID      ""
#define WLAN_PW        ""
#define WLAN_SECURITY  WLAN_SEC_WPA2

#define WEB_PORT       80

#define ADAFRUIT_CC3000_CS    6
#define ADAFRUIT_CC3000_IRQ   7
#define ADAFRUIT_CC3000_VBAT  8

Adafruit_CC3000 cc3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);

Adafruit_CC3000_Server webServer(WEB_PORT);

void wifi_init(void)
{ 
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
}
 
void wifi_connect()
{
  Serial.println(F("\nAttempting to connect to ")); 
  Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PW, WLAN_SECURITY)) 
  {
    Serial.println(F("Failed!"));
    while(1);
  }
  
  while (!cc3000.checkDHCP())
    delay(100);
}

void wifi_listen(void)
{
  webServer.begin();
}

void wifi_accept(void)
{
  Adafruit_CC3000_ClientRef client = webServer.available();
  if (!client) 
    return;

  char request[128];
  int request_len = 0;
  request[request_len] = 0;
  while (client && strchr(request, '\r') == 0 && request_len < sizeof(request))
  {
    if (client.available())
    {
      request[request_len++] = client.read();
      request[request_len] = 0;
    }
  }

  if (!client)
  {
    client.close();
    return;
  }
    
  if (strncmp(request, "GET /", 5) != 0)
  {
    client.close();
    return;
  }
  
  if (char *p = strchr(request, '\r')) *p = 0;
  if (char *p = strchr(request, '\n')) *p = 0;
  if (char *p = strstr(request, " HTTP")) *p = 0;

  client.fastrprint(F(
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

  // Workaround for CC3000 driver not sending the last few things.
  for (int i = 0; i < 100; i++)
    client.fastrprint(" ");
  client.fastrprint("\n");
  
  client.close();
}

void setup()
{
  Serial.begin(9600);

  wifi_init();
  wifi_connect();
  wifi_listen();  
}

void loop()
{
  wifi_accept();
}

