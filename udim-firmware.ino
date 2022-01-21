#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "";
char pass[] = "";

#define NUM_CHANNELS 3
#define ZERO_CROSSING_INT_PIN 12
#define DELTA 0               //(t_zero_crossing - t_interrupt) / STEP_TIME 4
// There might be some fixed mismatch between the actual zero-crossing and the interrupt,which is removed by this constant.
#define STEP_TIME 78         //for 128 lvls (in uS) (65 for 50 Hz) 78(128) 100(100)
#define MIN_LEVEL 25.0f
#define MAX_LEVEL 128.0f
// defines the interval over which the timer ISR is called repeatedly.It is calculated as follows:
// STEP_TIME: 1/(2*frequency*NumLvls)* 10^6
// Default: frequency = 50 Hz NumLvls = 128

int mode = 0;

// brightness control 
float dimming_usr[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f}; // user setting
float dimming_set[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f}; // setting
float dimming_inc[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f}; // increment
float dimming_cur[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f}; // current

// D1 (GPIO5), D2 (GPIO4), D3 (GPIO0), D5 (GPIO14), D8 (GPIO15)
int drive_pin[NUM_CHANNELS] = {5,4,14};
int state[NUM_CHANNELS] = {0,0,0};

volatile boolean is_handled[NUM_CHANNELS] = {0,0,0};
volatile int lvl_counter[NUM_CHANNELS] = {0,0,0};

int num_active_channels = 0;
volatile boolean zero_cross = 0;
volatile int num_handled = 0;

WiFiClient wifi;
PubSubClient mqtt(wifi);

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_state(int on_off, int channel_number) { 
  if (state[channel_number] == 0 && on_off == 1) {
    num_active_channels++;
    dimming_cur[channel_number] = MIN_LEVEL;
    update_dimming(dimming_usr[channel_number], channel_number);
    
    Serial.print("channel_");
    Serial.print(channel_number);
    Serial.println(" status: on");
  } else if (state[channel_number] == 1 && on_off == 0) {
    num_active_channels--;
    digitalWrite(drive_pin[channel_number], LOW);
    
    update_dimming(0.0f, channel_number);
    
    Serial.print("channel_");
    Serial.print(channel_number);
    Serial.println(" status: off");
  }

  state[channel_number] = on_off;
}

void update_dimming(float dimming_value, int channel_number) {
  dimming_set[channel_number] = dimming_value;
  dimming_inc[channel_number] = abs(dimming_set[channel_number] - dimming_cur[channel_number]) / 100.0f;
  
  Serial.print("channel_1 brightness = ");
  Serial.println(dimming_set[channel_number]);
  Serial.print("channel_1 increment = ");
  Serial.println(dimming_inc[channel_number]);
}

ICACHE_RAM_ATTR void zero_crossing_int() {
  if (num_active_channels > 0) {
    num_handled = 0;

    for (int i = 0; i < NUM_CHANNELS; i++) {
      is_handled[i] = 0;

      if (state[i] == 1) {
        digitalWrite(drive_pin[i], mode ? LOW : HIGH); // change mode
      }
      
      if (dimming_cur[i] < dimming_set[i]) {
        dimming_cur[i] += dimming_inc[i];
      } else if (dimming_cur[i] > dimming_set[i] + dimming_inc[i]) {
        dimming_cur[i] -= dimming_inc[i];
      }
    }

    zero_cross = 1;
    
    timer1_write(STEP_TIME * 5);
  }
}

ICACHE_RAM_ATTR void on_timer_isr() {
  if (zero_cross == 1) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (state[i] == 1 && is_handled[i] == 0) {
        if (lvl_counter[i] >= (int) (mode ? MAX_LEVEL - dimming_cur[i] : dimming_cur[i])) { // dimming_cur[i] + DELTA
          digitalWrite(drive_pin[i], mode ? HIGH : LOW); // change mode
          lvl_counter[i] = 0;
          is_handled[i] = 1; 

          num_handled++;
          if (num_handled == num_active_channels) {
            zero_cross = 0;
          }
        } else {
          lvl_counter[i]++;
        }
      }
    }

    if (zero_cross == 1) {
      timer1_write(STEP_TIME * 5);
    }
  }
}

void callback(char* t, byte* p, unsigned int l) {
  char b[l];
  for (int i = 0; i < l; i++) {
    b[i] = p[i];
  }
  b[l] = '\0';

  String topic(t);
  String param(b);
  
  if (topic == "udimmer/system/setmode") {
    mode = param == "true" ? 1 : 0;
    
    Serial.print("system mode: ");
    Serial.println(mode);
  }

  if (topic == "udimmer/channel1/seton") {
      update_state(param == "true" ? 1 : 0, 0);
  } else if (topic == "udimmer/channel2/seton") {
      update_state(param == "true" ? 1 : 0, 1);
  } else if (topic == "udimmer/channel3/seton") {
      update_state(param == "true" ? 1 : 0, 2);
  }

  if (topic == "udimmer/channel1/setbrightness") {
    dimming_usr[0] = mapf(param.toFloat(), 0.0f, 100.0f, MIN_LEVEL, MAX_LEVEL);
    update_dimming(dimming_usr[0], 0);
  }
  
  if (topic == "udimmer/channel2/setbrightness") {
    dimming_usr[1] = mapf(param.toFloat(), 0.0f, 100.0f, MIN_LEVEL, MAX_LEVEL);
    update_dimming(dimming_usr[1], 1);
  }
  
  if (topic == "udimmer/channel3/setbrightness") {
    dimming_usr[2] = mapf(param.toFloat(), 0.0f, 100.0f, MIN_LEVEL, MAX_LEVEL);
    update_dimming(dimming_usr[1], 1);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ZERO_CROSSING_INT_PIN, INPUT_PULLUP);
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(drive_pin[i], OUTPUT);
    digitalWrite(drive_pin[i], LOW);
  }
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("wifi connected");
  Serial.println("ip address: ");
  Serial.println(WiFi.localIP());
  
  mqtt.setServer("192.168.0.111", 1883);
  mqtt.setCallback(callback);
  
  noInterrupts();

  // Initialize Ticker
  timer1_attachInterrupt(on_timer_isr);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE); // 5MHz (5 ticks/us - 1677721.4 us max)
  // timer1_write(STEP_TIME * 5); // Ticks 5 = 1 us, 1ms = 1000us, 10ms = 10000us

  attachInterrupt(ZERO_CROSSING_INT_PIN, zero_crossing_int, RISING);   
  interrupts();
}

void loop() {
  if (!mqtt.connected()) {
    // Loop until we're reconnected
    while (!mqtt.connected()) {
      // Attempt to connect
      if (mqtt.connect("udimmer", "mosquitto", "password")) {
        Serial.println("mqtt connected");
        // Once connected, publish an announcement...
        mqtt.publish("udimmer/system/getmode", mode ? "true" : "false");
        mqtt.publish("udimmer/system/getonline", "online");
        mqtt.publish("udimmer/channel1/geton", state[0] ? "true" : "false");
        mqtt.publish("udimmer/channel1/getbrightness", String(dimming_cur[0]).c_str());
        mqtt.publish("udimmer/channel2/geton", state[1] ? "true" : "false");
        mqtt.publish("udimmer/channel2/getbrightness", String(dimming_cur[1]).c_str());
        mqtt.publish("udimmer/channel3/geton", state[2] ? "true" : "false");
        mqtt.publish("udimmer/channel3/getbrightness", String(dimming_cur[2]).c_str());
        // ... and resubscribe
        mqtt.subscribe("udimmer/system/setmode");
        mqtt.subscribe("udimmer/channel1/seton");
        mqtt.subscribe("udimmer/channel1/setbrightness");
        mqtt.subscribe("udimmer/channel2/seton");
        mqtt.subscribe("udimmer/channel2/setbrightness");
        mqtt.subscribe("udimmer/channel3/seton");
        mqtt.subscribe("udimmer/channel3/setbrightness");
      } else {
        Serial.print("mqtt connection failed, rc=");
        Serial.print(mqtt.state());
        Serial.println(" try again in 2 seconds");
        // Wait 5 seconds before retrying
        delay(2000);
      }
    }
  }

  mqtt.loop();
}
