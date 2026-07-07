
 //Projet 1 - Instrumentation d'une cuve de fermentation de vin

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <math.h>


#define PIN_SERVO          9
#define PIN_THERMISTOR    A2
#define PIN_TRIG          10
#define PIN_ECHO           7
#define PIN_WATER_SIGNAL  A0
#define PIN_WATER_POWER   A3
#define PIN_PRESSURE      A1    // potentiometre = pression simulee
#define PIN_URGENCE        2
#define PIN_LED_SAFE       6    // VERT  = systeme sain
#define PIN_LED_FAULT      5    // ROUGE = defaut / urgence
#define PIN_BUZZER         8

LiquidCrystal_I2C lcd(0x27, 16, 2); 

//thermistance
#define R_SERIES        9860.0   // ohms 
#define R_REF            8700.0   // ohms
#define T_REF_K          301.15   // K  (28 C, ambiant confirme)
#define B_COEF           3950.0   // Beta suppose
//Seuils
#define TEMP_MIN          15.0    // C
#define TEMP_MAX          30.0    // C
#define DIST_FULL_CM       5.0    // HC-SR04 : surface < 5 cm => cuve pleine
#define WATER_HIGH_RAW     370    // module A0 : > seuil => debordement

#define PRESSURE_FULL_BAR  3.0    // pression a fond d'echelle (raw 1023)
#define PRESSURE_MAX_BAR   2.5    // seuil d'alarme surpression

// Vanne
#define ANGLE_FERME        0
#define ANGLE_OUVERT      90      // quart de tour
#define DUREE_CYCLE_MS  10000UL   // maquette ; mettre 300000UL pour 5 min

// Cadences
#define LECTURE_MS        250UL
#define AFFICHE_MS        500UL


Servo vanne;

enum Etat { IDLE, OPEN, CLOSE, EMERGENCY };
Etat etatCourant = IDLE;

float temperature = 25.0;
float pression    = 0.0;  // bar
float distanceCM  = 999.0;
int   niveauEau   = 0;
bool  vanneOuverte = false;

bool  emergencyLatched = false;
bool  causeBouton      = false;

volatile bool          boutonPresse = false;
volatile unsigned long dernierISR   = 0;

unsigned long tempsDebutEtat = 0, lastLecture = 0, lastAffiche = 0, buzzerLast = 0;
bool          buzzerOn = false;


void ISR_urgence() {
  unsigned long t = millis();
  if (t - dernierISR > 200) { boutonPresse = true; dernierISR = t; }
}


void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initialisation..");

  vanne.attach(PIN_SERVO);
  fermerVanne();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_WATER_POWER, OUTPUT);
  digitalWrite(PIN_WATER_POWER, LOW);

  pinMode(PIN_LED_SAFE,  OUTPUT);
  pinMode(PIN_LED_FAULT, OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);

  pinMode(PIN_URGENCE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_URGENCE), ISR_urgence, FALLING);

  delay(1500);
  lcd.clear();

  lireCapteurs();
  etatCourant = IDLE;
  tempsDebutEtat = lastLecture = lastAffiche = millis();
  Serial.println(F("Systeme pret."));
  boutonPresse = false;
}


void loop() {

  if (millis() - lastLecture >= LECTURE_MS) {
    lastLecture = millis();
    lireCapteurs();
  }

  bool alarme = conditionAlarme();

  if (!emergencyLatched) {
    if (boutonPresse || alarme) {   // entree en defaut
      emergencyLatched = true;
      causeBouton  = boutonPresse;
      boutonPresse = false;
      etatCourant  = EMERGENCY;
      fermerVanne();                         // securite positive (commande)
    }
  } else {
    if (boutonPresse && !alarme) {           // acquittement si cause levee
      emergencyLatched = false;
      causeBouton = false;
      etatCourant = IDLE;
      tempsDebutEtat = millis();
    }
    boutonPresse = false;
  }

  if (!emergencyLatched) {
    switch (etatCourant) {
      case IDLE:
        etatCourant = OPEN; tempsDebutEtat = millis(); ouvrirVanne();
        break;
      case OPEN:
        if (millis() - tempsDebutEtat >= DUREE_CYCLE_MS) {
          etatCourant = CLOSE; tempsDebutEtat = millis(); fermerVanne();
        }
        break;
      case CLOSE:
        if (millis() - tempsDebutEtat >= DUREE_CYCLE_MS) {
          etatCourant = OPEN; tempsDebutEtat = millis(); ouvrirVanne();
        }
        break;
      default: break;
    }
  } else {
    etatCourant = EMERGENCY;
  }

  gererIndicateurs();

  if (millis() - lastAffiche >= AFFICHE_MS) {
    lastAffiche = millis();
    gererAffichage();
    afficherSerial();
  }
}

void lireCapteurs() {
  temperature = lireTemperature();
  pression    = lirePression();
  distanceCM  = lireUltrason();
  niveauEau   = lireNiveauEau();
}

float lireTemperature() {
  int adc = analogRead(PIN_THERMISTOR);
  if (adc <= 0)    return -99.0;   // court-circuit
  if (adc >= 1023) return 199.0;   // sonde debranchee (=> alarme fail-safe)
  float r    = R_SERIES * (float)adc / (1023.0 - (float)adc);
  float invT = 1.0 / T_REF_K + (1.0 / B_COEF) * log(r / R_REF);
  return (1.0 / invT) - 273.15;
}

float lirePression() {
  int adc = analogRead(PIN_PRESSURE);
  return adc * (PRESSURE_FULL_BAR / 1023.0);   // 0 .. 3.0 bar
}

float lireUltrason() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duree = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duree == 0) return 999.0;
  return duree * 0.034 / 2.0;
}

int lireNiveauEau() {
  digitalWrite(PIN_WATER_POWER, HIGH);
  delay(10);
  int v = analogRead(PIN_WATER_SIGNAL);
  digitalWrite(PIN_WATER_POWER, LOW);
  return v;
}

bool conditionAlarme() {
  bool pressionHaute = (pression > PRESSURE_MAX_BAR);
  bool tempHors      = (temperature < TEMP_MIN || temperature > TEMP_MAX);
  bool cuvePleine    = (distanceCM > 0 && distanceCM < DIST_FULL_CM);
  bool debordement   = (niveauEau > WATER_HIGH_RAW);
  return pressionHaute || tempHors || cuvePleine || debordement;
}

const char* obtenirCauseAlarme() {
  if (causeBouton)                 return "Bouton urgence";
  if (pression > PRESSURE_MAX_BAR) return "Pression haute";
  if (temperature > TEMP_MAX)      return "Temp trop haute";
  if (temperature < TEMP_MIN)      return "Temp trop basse";
  if (niveauEau > WATER_HIGH_RAW)  return "Debordement eau";
  if (distanceCM < DIST_FULL_CM)   return "Cuve pleine";
  return "Defaut capteur";
}


void ouvrirVanne() { vanne.write(ANGLE_OUVERT); vanneOuverte = true;  }
void fermerVanne() { vanne.write(ANGLE_FERME);  vanneOuverte = false; }


void gererIndicateurs() {
  if (emergencyLatched) {
    digitalWrite(PIN_LED_SAFE,  LOW);
    digitalWrite(PIN_LED_FAULT, HIGH);             // rouge fixe
    if (millis() - buzzerLast >= 400) {            // buzzer pulse
      buzzerLast = millis();
      buzzerOn = !buzzerOn;
      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_LED_SAFE,  HIGH);             // vert : tout va bien
    digitalWrite(PIN_LED_FAULT, LOW);
    digitalWrite(PIN_BUZZER,    LOW);
    buzzerOn = false;
  }
}


void afficherLCD(const char* l1, const char* l2) {
  char b[17];
  snprintf(b, sizeof(b), "%-16s", l1); lcd.setCursor(0, 0); lcd.print(b);
  snprintf(b, sizeof(b), "%-16s", l2); lcd.setCursor(0, 1); lcd.print(b);
}


void gererAffichage() {
  char l1[17], l2[17];
  const char* vanneTxt = vanneOuverte ? "OUVERT" : "FERMEE";

  if (emergencyLatched) {
    snprintf(l1, sizeof(l1), "ALARME  V:%s", vanneTxt);
    afficherLCD(l1, obtenirCauseAlarme());        // L2 = OU est le defaut
  } else {
    char tbuf[8], pbuf[8];
    dtostrf(temperature, 4, 1, tbuf);
    dtostrf(pression,    3, 1, pbuf);
    snprintf(l1, sizeof(l1), "V:%s  E%d", vanneTxt, niveauEau);
    snprintf(l2, sizeof(l2), "T%s P%s D%d", tbuf, pbuf, (int)distanceCM);
    afficherLCD(l1, l2);
  }
}

void afficherSerial() {
  char tbuf[8], pbuf[8];
  dtostrf(temperature, 4, 1, tbuf);
  dtostrf(pression,    3, 1, pbuf);
  Serial.print(F("T=")); Serial.print(tbuf);
  Serial.print(F("C P=")); Serial.print(pbuf);
  Serial.print(F("bar D=")); Serial.print(distanceCM);
  Serial.print(F("cm Eau=")); Serial.print(niveauEau);
  Serial.print(F(" Vanne=")); Serial.print(vanneOuverte ? F("OUV") : F("FER"));
  Serial.print(F(" Etat="));
  Serial.println(emergencyLatched ? F("ALARME") : F("OK"));
}
