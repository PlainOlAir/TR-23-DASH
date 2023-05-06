/*XCZVbb
    0x50 - Kill Sw[01]  N/O[234567]
    0x51 - Temp[01]     Volt[23]      Clutch[45]    N/O[67]
    0x52 - Oil P[01]    Fuel P[23]    APP[45]       N/O[67]
    0x53 - RPM[01]      N/O[34]       TC[45]        MIL[67]
*/

//CANbus setup
#include <same51_can.h>
SAME51_CAN can;

//LED Strip setup
#include <Adafruit_NeoPixel.h>
const int PIN = 4;
const int NUMPIXELS = 13;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Data values
int rpm = 0;    //#
int t = 0;      //F
int fp = 0;     //dpsi
int op = 0;     //dpsi
int batt = 0;   //dV
int mil = 0;    //#
int spd = 0;    //hm/h
int tc = 0;     //#
float tclvl = 0;//#
float lclvl = 0;//#
float emlvl = 0;//#
int clutch = 0; //#
int killsw = 0; //#
int pedpos = 0; //0.1%
int brakep = 0; //psi

//RPM
const int rpm_max = 9000;
const int rpm_min = 1000;
const int rpm_rng = rpm_max - rpm_min;
const int rpm_step = (rpm_rng) / NUMPIXELS;

//Brake Pressure
const float brakep_max = 1500;
const float brakep_min = 550;

//Warning logic
int warning_sleep = 30000;
int warntime = 0;
bool warning = false;

//Button logic
int button_time = 0;
int hold_time = 0;
int button_depress = 200;

//Switch logid
int switchThresh[10] = {0, 80, 160, 230, 300, 390, 470, 530, 650, 800};
int switchDiff[2];
int switchValues[3];
int switchLevels[3];

//General setup
String endChar = String(char(0xff)) + String(char(0xff)) + String(char(0xff));
String page = "boot";
String lastpage = "";
String diag0 = "BATTERY VOLTAGE";
String diag1 = "GPS SIGNAL";
String diag2 = "ENGINE TEMPERATURE";
int diag = 0;
int i = 0;
int j = 0;
int reftime = 0;
const int diagnostictime = 3000;
const int logotime = 4000;

static void off_LED() {
  if (millis() % 2000 > 1000) {
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
  } else {
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 0));
    }
  }
  pixels.show();
}

static void tc_LED() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 0, 255));
  }
  pixels.show();
}

static void clutch_LED() {
  int current_step = floor( (millis() % 600) / (600 / 7));

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  }
  pixels.setPixelColor(current_step, pixels.Color(255, 0, 0));
  pixels.setPixelColor(NUMPIXELS - (current_step + 1), pixels.Color(255, 0, 0));
  pixels.show();
}

static void tach_LED(int rev) {
  int current_step = (max(rpm_min, rev) - rpm_min) / rpm_step;

  //Above limit

  if (rev > rpm_max) {
    if (millis() % 100 > 50) {
      for (int i = 0; i < 3; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));
        pixels.setPixelColor(NUMPIXELS - 1 - i, pixels.Color(0, 255, 0));
      }
      for (int i = 3; i < 5; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 0));
        pixels.setPixelColor(NUMPIXELS - 1 - i, pixels.Color(255, 255, 0));
      }
      for (int i = 5; i < 7; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 255));
        pixels.setPixelColor(NUMPIXELS - 1 - i, pixels.Color(0, 0, 255));
      }
    } else {
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
    }

    //Below limit
  } else {
    for (i = 0; i < 7; i++) {
      if (i < current_step) {
        if (i < 3) {
          pixels.setPixelColor(i, pixels.Color(0, 255, 0));
          pixels.setPixelColor(NUMPIXELS - i - 1, pixels.Color(0, 255, 0));
        } else if (i >= 3 && i < 5) {
          pixels.setPixelColor(i, pixels.Color(255, 255, 0));
          pixels.setPixelColor(NUMPIXELS - i - 1, pixels.Color(255, 255, 0));
        } else if (i >= 5) {
          pixels.setPixelColor(i, pixels.Color(0, 0, 255));
          pixels.setPixelColor(NUMPIXELS - i - 1, pixels.Color(0, 0, 255));
        }
      } else if (i >= current_step) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        pixels.setPixelColor(NUMPIXELS - i - 1, pixels.Color(0, 0, 0));
      }
    }
  }
  pixels.show();
}

void setup() {
  pixels.begin();

  Serial1.begin(115200);
  Serial.begin(115200);
  while (!Serial1);

  uint8_t ret;
  ret = can.begin(MCP_ANY, CAN_500KBPS, MCAN_MODE_CAN);

  delay(1000);
  Serial1.print("page logo" + endChar);
  page = "logo";
  delay(logotime);
  Serial1.print("page important" + endChar);
  page = "important";

}

void loop() {
  uint8_t ret;
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
  uint8_t i;

  ret = can.readMsgBuf(&id, &len, buf);

  //Get CAN data
  if (ret == CAN_OK) {
    switch (id) {
      case 0x50:
        killsw = (buf[1] * 256 + buf[0]);
        if (killsw < 6000) {
          killsw = 0;
        } else if (killsw > 10000) {
          killsw = 1;
        }
        break;
      case 0x51:
        t = (buf[1] * 256 + buf[0]);
        batt = (buf[3] * 256 + buf[2]);
        clutch = (buf[5] * 256 + buf[4]);
        break;
      case 0x52:
        op = (buf[1] * 256 + buf[0]);
        fp = (buf[3] * 256 + buf[2]);
        pedpos = (buf[5] * 256 + buf[4]);
        brakep = (buf[7] * 256 + buf[6]);
        pedpos = round(pedpos / 100.0);
        brakep = ((brakep - brakep_min) / (brakep_max - brakep_min)) * 100;
        break;
      case 0x53:
        rpm = (buf[1] * 256 + buf[0]);
        tc = (buf[5] * 256 + buf[4]);
        mil = (buf[7] * 256 + buf[6]);
        break;
    }
  }

  switchValues[0] = analogRead(A1);
  switchValues[1] = analogRead(A0);
  switchValues[2] = analogRead(A3);
  
  for (i = 0; i < 3; i++) {
    switchDiff[1] = 3000;
    for (j = 0; j < 10; j++) {
      switchDiff[0] = abs(switchThresh[j] - switchValues[i]);
      if (switchDiff[0] < switchDiff[1]) {
        switchLevels[i] = j;
        switchDiff[1] = switchDiff[0];
      }
    }
  }
  
  lclvl = 7 - switchLevels[0];
  tclvl = 7 - switchLevels[1];
  emlvl = 7 - switchLevels[2];

  
  

  //Update values in each page
  if (page == "important") {
    Serial1.print("ecut.val=" + String(t) + endChar);
    Serial1.print("ecubatt.val=" + String(batt) + endChar);
    Serial1.print("ecuop.val=" + String(op) + endChar);
    Serial1.print("ecufp.val=" + String(fp) + endChar);
    Serial1.print("rpm.val=" + String(rpm) + endChar);
    Serial1.print("pedpos.val=" + String(pedpos) + endChar);
    Serial1.print("brakepos.val=" + String(brakep) + endChar);
    Serial1.print("tclvl.val=" + String(int(tclvl)) + endChar);
    Serial1.print("lclvl.val=" + String(int(lclvl)) + endChar);
    Serial1.print("emlvl.val=" + String(int(emlvl)) + endChar);

    if (mil >= 1) {
      Serial1.print("mil.bco=65535" + endChar + "mil.pco=63488" + endChar);
    } else {
      Serial1.print("mil.bco=0" + endChar + "mil.pco=0" + endChar);
    }

    if (tc != 0) {
      Serial1.print("tc.bco=8160" + endChar + "tc.pco=63515" + endChar);
    } else {
      Serial1.print("tc.bco=0" + endChar + "tc.pco=65535" + endChar);
    }

    if (clutch != 0) {
      Serial1.print("launch.bco=8160" + endChar + "launch.pco=63515" + endChar);
    } else {
      Serial1.print("launch.bco=0" + endChar + "launch.pco=65535" + endChar);
    }

  } else if (page == "race") {

    Serial1.print("ecut.val=" + String(t) + endChar);
    Serial1.print("ecubatt.val=" + String(batt) + endChar);
    Serial1.print("ecuop.val=" + String(op) + endChar);
    Serial1.print("ecufp.val=" + String(fp) + endChar);
    Serial1.print("rpm.val=" + String(rpm) + endChar);
    Serial1.print("tclvl.val=" + String(int(tclvl)) + endChar);
    Serial1.print("lclvl.val=" + String(int(lclvl)) + endChar);
    Serial1.print("emlvl.val=" + String(int(emlvl)) + endChar);

    if (mil >= 1) {
      Serial1.print("mil.bco=65535" + endChar + "mil.pco=63488" + endChar);
    } else {
      Serial1.print("mil.bco=0" + endChar + "mil.pco=0" + endChar);
    }

    if (tc != 0) {
      Serial1.print("tc.bco=63488" + endChar);
      Serial1.print("tclvl.bco=63488" + endChar);
    } else {
      Serial1.print("tc.bco=0" + endChar);
      Serial1.print("tclvl.bco=0" + endChar);
    }

    if (clutch != 0) {
      Serial1.print("lc.bco=63488" + endChar);
      Serial1.print("lclvl.bco=63488" + endChar);
    } else {
      Serial1.print("lc.bco=0" + endChar);
      Serial1.print("lclvl.bco=0" + endChar);
    }
  }

  //Warnings
  if (page == "race" && mil > 0 && ((millis() - warntime) > warning_sleep) && rpm > 1000) {
    Serial1.print("page ecuwarn" + endChar);
    warning = true;
    lastpage = page;
    page = "ecuwarn";
  } else if (page == "ecuwarn" && mil == 0) {
    page = lastpage;
    Serial1.print("page " + lastpage);
  }

  if (page == "ecuwarn" && rpm <= 1000) {
    page = lastpage;
    Serial1.print("page " + lastpage);
  }

  //Screen switch/Ack

  if (analogRead(A2) >= 500 && (millis() - button_time > button_depress)) {

    button_time = millis();
    hold_time = millis();

    if (page == "important") {
      page = "race";
      Serial1.print("page race");

    } else if (page == "race") {
      page = "important";
      Serial1.print("page important");

    } else if (page == "ecuwarn") {
      page = lastpage;
      warntime = millis();
      Serial1.print("page " + lastpage);
      
    } else if (page == "munch") {
      page = "important";
      Serial1.print("page important");
    }

    Serial1.print(endChar);
    delay(100);

  } else if (analogRead(A2) >= 500 && millis() - hold_time > 2000) {
    hold_time = millis();
    button_time = millis();
    
    page = "munch";
    Serial1.print("page munch" + endChar);
    delay(100);
    
  } else if (analogRead(A2) >= 500 && millis() - button_time < button_depress) {
    button_time = millis();
  }

  //Car state
  if (page == "important") {
    if (killsw != 0) {
      Serial1.print("carstate.txt=\"CAR ON\"" + endChar + "carstate.pco=2016" + endChar);
    } else {
      Serial1.print("carstate.txt=\"CAR OFF\"" + endChar + "carstate.pco=63488" + endChar);
    }
  }

  //LED
  if (killsw == 0) {
    off_LED();
  } else if (killsw != 0) {
    if (tc != 0) {
      tc_LED();
    } else if (clutch != 0) {
      clutch_LED();
    } else {
      tach_LED(rpm);
    }
  }
}

// END FILE
