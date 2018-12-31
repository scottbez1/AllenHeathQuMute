#include <Bounce2.h>

#include <DNSServer.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>


#define MUTE_BUTTON D1
#define UNMUTE_BUTTON D2
#define MUTE_LED D3
#define UNMUTE_LED D4
#define WIFI_LED D6
#define MIXER_LED D5

 
IPAddress serverAddress(192,168,1,140);
//IPAddress serverAddress(192,168,0,101);
unsigned long serverPort = 51325;
const byte QU_MIDI_CHANNEL = 1;
const byte MUTE_GROUP_NOTE = 0x53; // Mute Group 4


/* Set these to your desired softAP credentials. They are not configurable at runtime */
const char *softAP_ssid = "QuMuteConfig";
const char *softAP_password = "";

/* Don't set this wifi credentials. They are configurated at runtime and stored on EEPROM */
char ssid[32] = "linksys";
char password[32] = "";

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

// Web server
ESP8266WebServer server(80);

/* Soft AP network parameters */
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);


/** Should I connect to WLAN asap? */
boolean connect = true;

/** Last time I tried to connect to WLAN */
long lastWifiConnectTry = 0;

long lastTcpConnectTry = 0;

long lastStatusUpdate = 0;

/** Current WLAN status */
int status = WL_IDLE_STATUS;

Bounce muteButton = Bounce();
Bounce unmuteButton = Bounce();

WiFiClient client;

#define MIDI_BUF_SIZE 4
short midiBuf[] = {-1, -1, -1, -1};
int midiBufNext = 0;

bool muted = false;
bool unmuted = false;

bool waitingForStatus = false;

void setup() {
  pinMode(MUTE_BUTTON, INPUT_PULLUP);
  pinMode(UNMUTE_BUTTON, INPUT_PULLUP);
  pinMode(MUTE_LED, OUTPUT);
  pinMode(UNMUTE_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(MIXER_LED, OUTPUT);

  digitalWrite(MUTE_LED, LOW);
  digitalWrite(UNMUTE_LED, LOW);
  digitalWrite(WIFI_LED, LOW);
  digitalWrite(MIXER_LED, LOW);
  
  delay(1000);

  digitalWrite(WIFI_LED, HIGH);
  delay(100);
  digitalWrite(MIXER_LED, HIGH);
  delay(100);
  digitalWrite(MUTE_LED, HIGH);
  delay(100);
  digitalWrite(UNMUTE_LED, HIGH);
  delay(500);
  
  digitalWrite(WIFI_LED, LOW);
  delay(100);
  digitalWrite(MIXER_LED, LOW);
  delay(100);
  digitalWrite(MUTE_LED, LOW);
  delay(100);
  digitalWrite(UNMUTE_LED, LOW);
  delay(500);
  
  muteButton.attach(MUTE_BUTTON);
  muteButton.interval(20);
  unmuteButton.attach(UNMUTE_BUTTON);
  unmuteButton.interval(20);
  
  Serial.begin(115200);
  
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(softAP_ssid, softAP_password);
  delay(500); // Without delay I've seen the IP address blank
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  /* Setup the DNS server redirecting all the domains to the apIP */  
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", apIP);

  /* Setup web pages: root, wifi config pages, SO captive portal detectors and not found. */
  server.on("/", handleRoot);
  server.on("/wifi", handleWifi);
  server.on("/scan", handleScan);
  server.on("/wifisave", handleWifiSave);
  server.on("/generate_204", handleRoot);  //Android captive portal. Maybe not needed. Might be handled by notFound handler.
  server.on("/fwlink", handleRoot);  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  server.onNotFound ( handleNotFound );
  server.begin(); // Web server start
  Serial.println("HTTP server started");
  loadCredentials(); // Load WLAN credentials from network
}

void connectWifi() {
  Serial.println("Connecting as wifi client...");
  Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
  digitalWrite(WIFI_LED, HIGH);
  delay(100);
  digitalWrite(WIFI_LED, LOW);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  int connRes = WiFi.waitForConnectResult();
  Serial.print ( "connRes: " );
  Serial.println ( connRes );
}

void loop() {
  unsigned long t = millis();
  if (connect) {
    Serial.println ( "Connect requested" );
    connect = false;
    connectWifi();
    lastWifiConnectTry = t;
  }
  {
    int s = WiFi.status();
    if (s == 0 && t - lastWifiConnectTry > 30000) {
      /* If WLAN disconnected and idle try to connect */
      /* Don't set retry time too low as retry interfere the softAP operation */
      connect = true;
    }
//    digitalWrite(D4, !(
//      (s == WL_CONNECTED) ||
//      ((s == WL_CONNECT_FAILED || s == WL_CONNECTION_LOST || s == WL_DISCONNECTED) && ((t % 1000) < 8 || (t % 1000 > 100 && t % 1000 < 108)))
//      ));
    if (status != s) { // WLAN status change
      Serial.print ( "Status: " );
      Serial.println ( s );
      status = s;
      if (s == WL_CONNECTED) {
        /* Just connected to WLAN */
        Serial.println ( "" );
        Serial.print ( "Connected to " );
        Serial.println ( WiFi.SSID() );
        Serial.print ( "IP address: " );
        Serial.println ( WiFi.localIP() );
      } else if (s == WL_NO_SSID_AVAIL) {
        WiFi.disconnect();
      }
    }

    if (!client.connected()) {
        waitingForStatus = false;
    }

    if (s == WL_CONNECTED) {
      if (!client.connected() && (t - lastTcpConnectTry > 10000 || lastTcpConnectTry == 0)) {
        lastTcpConnectTry = t;
        Serial.println("connecting to server...");
        Serial.println(serverAddress);
        digitalWrite(MIXER_LED, HIGH);
        delay(100);
        digitalWrite(MIXER_LED, LOW);
        if (client.connect(serverAddress, serverPort)) {
          client.setNoDelay(true);
          Serial.println("connected to server");
        } else {
          Serial.println("failed to connect to server");
        }
      }
    }

    if (client.connected()) {
      if (muteButton.fell()) {
        sendMute(true);
      }
      if (unmuteButton.fell()) {
        sendMute(false);
      }
      
      if (!waitingForStatus && (t - lastStatusUpdate > 15000 || lastStatusUpdate == 0)) {
        lastStatusUpdate = t;
        waitingForStatus = true;
        Serial.println("request status");
        muted=false;
        unmuted=false;

        char req[] = {0xF0,0x00,0x00,0x1A,0x50,0x11,0x01,0x00,0x7F,0x10,0x01,0xF7};

        client.write(req, 12);
        
        Serial.println("wrote request status");
        client.flush();
        Serial.println("flushed request status");
      }
      while (client.available()) {
        yield();
        midiBuf[midiBufNext] = client.read();
        midiBufNext = (midiBufNext + 1) % MIDI_BUF_SIZE;

        if ((midiBuf[(midiBufNext + MIDI_BUF_SIZE - 3)%MIDI_BUF_SIZE] & 0xF0) == 0x90 &&
             midiBuf[(midiBufNext + MIDI_BUF_SIZE - 2)%MIDI_BUF_SIZE] >= 0 && 
             midiBuf[(midiBufNext + MIDI_BUF_SIZE - 1)%MIDI_BUF_SIZE] >= 0) {
          // Note ON message
          uint8_t note = midiBuf[(midiBufNext + MIDI_BUF_SIZE - 2)%MIDI_BUF_SIZE];
          uint8_t velocity = midiBuf[(midiBufNext + MIDI_BUF_SIZE - 1)%MIDI_BUF_SIZE];
          // Ignore velocity 0x00
          if (velocity != 0 && note == MUTE_GROUP_NOTE) {
            lastStatusUpdate = t;
            waitingForStatus = false;
            Serial.printf("Note on: 0x%02x, velocity: 0x%02x\n", note, velocity);
            if (velocity >= 0x01 && velocity <= 0x3F) {
              unmuted = true;
              muted = false;
            } else if (velocity >= 0x40 && velocity <= 0x7F) {
              unmuted = false;
              muted = true;
            }
          }
        } else if (midiBuf[(midiBufNext + MIDI_BUF_SIZE - 1)%MIDI_BUF_SIZE] == 0xFE) {
          // Active Sense
          Serial.println("active sense");
          client.write(0xFE);
          client.flush();
        }
      }
    }
  }

  digitalWrite(MUTE_LED, client.connected() && muted);
  digitalWrite(UNMUTE_LED, client.connected() && unmuted);
  digitalWrite(WIFI_LED, WiFi.status() == WL_CONNECTED);
  digitalWrite(MIXER_LED, client.connected());
  
  muteButton.update();
  unmuteButton.update();
  dnsServer.processNextRequest();
  server.handleClient();
}

void sendMute(boolean mute) {
  digitalWrite(MUTE_LED, LOW);
  digitalWrite(UNMUTE_LED, LOW);
    Serial.println("sending mute update");

    char req[] = {0x90 + QU_MIDI_CHANNEL - 1, MUTE_GROUP_NOTE, mute ? 0x7F : 0x3F, 0x80 + QU_MIDI_CHANNEL - 1, MUTE_GROUP_NOTE, 0x00};

    client.write(req, 6);

    client.flush();

    // Clear mute status (will be reset by the next status refresh)
    muted = false;
    unmuted = false;

    // Trigger a status refresh
    lastStatusUpdate = 0;
    waitingForStatus = false;
}
