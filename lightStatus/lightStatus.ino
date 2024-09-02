#include <FastLED.h>


#define NUM_LEDS 300
#define DATA_PIN 6

#define INTERNAL_LED_PIN 13
#define KILL_DETECT_PIN A0

CRGB leds[NUM_LEDS];

CRGB RED = CRGB(255,0,0);
CRGB GREEN = CRGB(0,255,0);
CRGB YELLOW = CRGB(255,150,0);
CRGB BLACK = CRGB(0,0,0);

String incoming_data = "";

//IF DATA IS EQUAL TO:
// "00" : Telemode
// "11" : Automode

// DEFAULT IS TELEMODE
// KILL DETECTION OVERRIDES CURRENT MODE

bool KILLED = false;
bool TELE = true; // If FALSE means in Automode

void setup() {
  // put your setup code here, to run once:
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(KILL_DETECT_PIN, INPUT);
  Serial.begin(115200);

  digitalWrite(INTERNAL_LED_PIN, LOW);
}

void check_kill() {
  if(digitalRead(KILL_DETECT_PIN) == HIGH) {
    KILLED = false;
  }
  else {
    KILLED = true;
  }
}


void change_mode() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incoming_data = Serial.readString();
    incoming_data.trim();
    Serial.print("Received: ");
    Serial.println(incoming_data);

    if(incoming_data=="00") {
      TELE = true;
      Serial.println("SWITCHED TO TELEOP STATUS");
    }
    else if(incoming_data=="11") {
      TELE = false;
      Serial.println("SWITCHED TO AUTO STATUS");
    }
  }
  check_kill();
}

void show_color(CRGB color) {
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
}


void show_killed() {
  digitalWrite(INTERNAL_LED_PIN, HIGH);
  show_color(RED);
  delay(200);
  digitalWrite(INTERNAL_LED_PIN, LOW);
  show_color(BLACK);
  delay(200);
}

void show_tele() {
  digitalWrite(INTERNAL_LED_PIN, LOW);
  show_color(GREEN);
  delay(100);
}

void show_auto() {
  digitalWrite(INTERNAL_LED_PIN, HIGH);
  show_color(YELLOW);
  delay(100);
}

void change_status() {
  change_mode();

  if(KILLED) {
    show_killed();
    return;
  }
  if(TELE) {
    show_tele();
  }
  else {
    show_auto();
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  change_status();
}
