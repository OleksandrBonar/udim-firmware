#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "";
char pass[] = "";

#define MODE_LEADING_EDGE 0
#define MODE_TRAILING_EDGE 1
#define NUM_CHANNELS 3
#define ZERO_CROSSING_INT_PIN 12
#define DELTA 4 // (t_zero_crossing - t_interrupt) / STEP_TIME
#define STEP_TIME 78 // for 128 lvls (in uS) (65 for 50 Hz) 78

int mode = MODE_TRAILING_EDGE;

// 128 lvl brightness control 
int dimming_lvl[NUM_CHANNELS] = {0,0,0}; // (0-127) 

int drive_pin[NUM_CHANNELS] = {5,4,0};
int state[NUM_CHANNELS] = {0,0,0};

volatile boolean is_handled[NUM_CHANNELS] = {0,0,0};
volatile int lvl_counter[NUM_CHANNELS] = {0,0,0};

int num_active_channels = 0;
volatile boolean zero_cross = 0;
volatile int num_handled = 0;

WiFiClient wifi;
PubSubClient mqtt(wifi);

void update_state(int on_off, int channel_number) { 
  if (state[channel_number] == 0 && on_off == 1) {
    num_active_channels++;
    EEPROM.write(1, num_active_channels);
  } else if (state[channel_number] == 1 && on_off == 0) {
    num_active_channels--;
    EEPROM.write(1, num_active_channels);
    digitalWrite(drive_pin[channel_number], LOW);
  }

  state[channel_number] = on_off;
  EEPROM.write(2 + channel_number, on_off);
}

ICACHE_RAM_ATTR void zero_crossing_int() {
  if (num_active_channels > 0) {
    num_handled = 0;

    for (int i = 0; i < NUM_CHANNELS; i++) {
      is_handled[i] = 0;       

	    if (state[i] == 1) {
        digitalWrite(drive_pin[i], mode == MODE_LEADING_EDGE ? LOW : HIGH);
      }
    }

    zero_cross = 1; 
  }
}

ICACHE_RAM_ATTR void on_timer_isr() {
  if (zero_cross == 1) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (state[i] == 1) {
        if (lvl_counter[i] > dimming_lvl[i] + DELTA) {
          digitalWrite(drive_pin[i], mode == MODE_LEADING_EDGE ? HIGH : LOW);
          lvl_counter[i] = 0;
          is_handled[i] = 1; 

          num_handled++;
          if (num_handled == num_active_channels) {
            zero_cross = 0;
          }
        } else if (is_handled[i] == 0) {
          lvl_counter[i]++;
        }
      }
    }
  }
  
  timer1_write(STEP_TIME * 5);
}

void callback(char* t, byte* p, unsigned int l) {
  Serial.print("Message arrived [");
  Serial.print(t);
  Serial.print("] ");
  for (int i = 0; i < l; i++) {
    Serial.print((char)p[i]);
  }
  Serial.println();

  String topic(t);

  if (topic.equals("udim/channel1/seton")) {
    if (0 == memcmp(p, "true", l)) {
      Serial.println("Channel1 On");
      update_state(1, 0);
    } else {
      Serial.println("Channel1 Off");
      update_state(0, 0);
    }
  } else if (topic.equals("udim/channel2/seton")) {
    if (0 == memcmp(p, "true", l)) {
      Serial.println("Channel2 On");
      update_state(1, 1);
    } else {
      Serial.println("Channel2 Off");
      update_state(0, 1);
    }
  } else if (topic.equals("udim/channel3/seton")) {
    if (0 == memcmp(p, "true", l)) {
      Serial.println("Channel3 On");
      update_state(1, 2);
    } else {
      Serial.println("Channel3 Off");
      update_state(0, 2);
    }
  }

  if (topic.equals("udim/channel1/setbrightness")) {
    EEPROM.write(5, p);
    dimming_lvl[0] = atoi((char*)p);
    Serial.print("Channel1 Level: ");
    Serial.println(dimming_lvl[0]);
  } else if (topic.equals("udim/channel2/setbrightness")) {
    EEPROM.write(6, p);
    dimming_lvl[1] = atoi((char*)p);
    Serial.print("Channel2 Level: ");
    Serial.println(dimming_lvl[1]);
  } else if (topic.equals("udim/channel3/setbrightness")) {
    EEPROM.write(7, p);
    dimming_lvl[2] = atoi((char*)p);
    Serial.print("Channel3 Level: ");
    Serial.println(dimming_lvl[2]);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ZERO_CROSSING_INT_PIN, INPUT_PULLUP);
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(drive_pin[i], OUTPUT);
    digitalWrite(drive_pin[i], LOW);
  }
  
  EEPROM.begin(512);
  mode = atoi((char)EEPROM.read(0));
  num_active_channels = atoi((char)EEPROM.read(1));
  state[0] = atoi(char)EEPROM.read(2));
  state[1] = atoi(char)EEPROM.read(3));
  state[2] = atoi(char)EEPROM.read(4));
  dimming_lvl[0] = atoi((char)EEPROM.read(5));
  dimming_lvl[1] = atoi((char)EEPROM.read(6));
  dimming_lvl[2] = atoi((char)EEPROM.read(7));
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  mqtt.setServer("192.168.0.111", 1883);
  mqtt.setCallback(callback);
  
  noInterrupts();

  // Initialize Ticker
  timer1_attachInterrupt(on_timer_isr);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(STEP_TIME * 5);

  attachInterrupt(ZERO_CROSSING_INT_PIN, zero_crossing_int, RISING);   
  interrupts();
}

void loop() {
  if (!mqtt.connected()) {
    // Loop until we're reconnected
    while (!mqtt.connected()) {
      Serial.print("Connecting to MQTT: ");
      // Attempt to connect
      if (mqtt.connect("udim", "mosquitto", "password")) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        mqtt.publish("udim/system/getmode", mode);
        mqtt.publish("udim/system/getonline", "online");
        mqtt.publish("udim/channel1/geton", state[0]);
        mqtt.publish("udim/channel1/getbrightness", dimming_lvl[0]);
        mqtt.publish("udim/channel2/geton", state[1]);
        mqtt.publish("udim/channel2/getbrightness", dimming_lvl[1]);
        mqtt.publish("udim/channel3/geton", state[2]);
        mqtt.publish("udim/channel3/getbrightness", dimming_lvl[2]);
        // ... and resubscribe
        mqtt.subscribe("udim/system/setmode");
        mqtt.subscribe("udim/channel1/seton");
        mqtt.subscribe("udim/channel1/setbrightness");
        mqtt.subscribe("udim/channel2/seton");
        mqtt.subscribe("udim/channel2/setbrightness");
        mqtt.subscribe("udim/channel3/seton");
        mqtt.subscribe("udim/channel3/setbrightness");
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqtt.state());
        Serial.println(" try again in 2 seconds");
        // Wait 5 seconds before retrying
        delay(2000);
      }
    }
  }

  mqtt.loop();
}
