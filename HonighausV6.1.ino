// Metzens Honighaus Version 6.1
// Copyright by Rüdiger Metzen, Viernheim
// September März 2025
// Wifi Funktionen wurden herausprogrammiert
// Fachöffnung: Relais werden kurz zweimal hintereinander angesteuert, falls die Magnetschlösser festsitzen

// HINWEISE:
// Dieses Progamm ist voll lauffähig
// Der Münzprüfer ist tückisch. Er produziert im abgeschalteten Zustand Fehlimpulse, ebenfall bei Betätigung der Türöffner
// Daher muss der Interupt zur Abfrage eines Münzeinwurfes jeweils unterbrochen werden
// Nach Einstecken des Netzsteckers oder Stromausfall beginnt der Automat zunächst in der Betriebsart
// "Ausser Betrieb". Es muß dann einmal der Betriebsartenschalter Ein/Aus betätigt werden.
// Um Fehlimpulse des Münzprüfers durch Elektrostatik auszuschliessen wurde eine Überwachung der Zählimpulslänge ergänzt
// Ein korrekter Zählimpuls ist 170ms lang. Alles zwischen 165 und 175ms wird akzeptiert.
// Eine 1 Euro Münze erzeugt 2 Zählimpulse
// Eine 2 Euro Münze erzeugt 3 Zählimpulse

// Digitalausgang Pin 32 = Relaiskontakt 1 (Fach 1)
// Digitalausgang Pin 33 = Relaiskontakt 2 (Fach 2)
// Digitalausgang Pin 25 = Relaiskontakt 3 (Fach 3)
// Digitalausgang Pin 26 = Relaiskontakt 4 (Fach 4)
// Digitalausgang Pin 27 = Relaiskontakt 5 (Fach 5)
// Digitalausgang Pin 14 = Relaiskontakt 6 (Fach 6)
// Digitalausgang Pin 23 = Relaiskontakt 7 (Münzprüfer)
// Digitalausgang Pin 13 = Relaiskontakt 8 (Beleuchtung)

// Analogeingang Pin 34 = Fotowiderstand über Spannungsteiler (ADC1Ch6)
// Digitaleingang Pin 15 = Taste zur Bedienung/Türöffnung

// Pin 21 SDA = zum LCD Display
// Pin 22 SDL = zum LCD Display

// Pin 5 = Münzprüfer Zähleingang
// Pin 2 = Wahlschalter Betrieb/Ausser Betrieb

#include "DFRobotDFPlayerMini.h"
DFRobotDFPlayerMini myDFPlayer; // Achtung Program läuft nur mit DFPlayer 1.0.5
void printDetail(uint8_t type, int value);

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

#include <HardwareSerial.h>
HardwareSerial MySerial(1); // Serielle Schnittstelle für MP3 Modul
      
const int Relais1 = 32;         // Relais 1 von 8er Platine
const int Relais2 = 33;         // Relais 2 von 8er Platine
const int Relais3 = 25;         // Relais 3 von 8er Platine
const int Relais4 = 26;         // Relais 4 von 8er Platine
const int Relais5 = 27;         // Relais 5 von 8er Platine
const int Relais6 = 14;         // Relais 6 von 8er Platine
const int Relais7 = 23;         // Relais 7 von 8er Platine
const int Relais8 = 13;         // Relais 8 von 8er Platine

unsigned long currentMillis = 0;
unsigned long oldMillis = 0;
int coinsChange = 0; // a coin has been inserted flag

int Lautstaerke;                // aktuelle Lautstärke für die Biene Maja
const int LautstaerkeTag = 15;  // Lautstärkeeinstellung am Tag
const int LautstaerkeNacht = 13; // Lautstärkeeinstellung in der Nacht

const int coinpin = 5;          // Pin für Münzprüfer
volatile int EuroBezahlt = 0;

int Honigpreis = 8;       // Preis pro Glas bei Start des Automaten
int credits = 0;
int Fach = 1;
int SchwellwertBeleuchtung = 20; // ab hier Beleuchtung einschalten, je höher der Wert, desto heller
int Hysterese = 10;               // verhindert ein-und ausschalten bei Helligkeitswechsel

bool Netzwiederkehr = true;   // Nach einem Stromausfall beginnt der Automat zunächst in der Betriebsart "Ausser Betrieb"

// *******************************************************************************
// *****                                SETUP                                *****
// *******************************************************************************

void setup() {
  
Serial.begin(9600);                    // Debugging auf PC
MySerial.begin(9600,SERIAL_8N1,18,19); // Serielle Schnittstelle an Pin 19 TX, Ansteuerung Sound-Modul

myDFPlayer.begin(MySerial);
myDFPlayer.EQ(DFPLAYER_EQ_BASS);
Lautstaerke = LautstaerkeTag; //beginne mit der Lautstärkeeinstellung für den Tag

// Benötigte Digitalpins als Ausgang definieren
  pinMode(Relais1, OUTPUT);
  pinMode(Relais2, OUTPUT);
  pinMode(Relais3, OUTPUT);
  pinMode(Relais4, OUTPUT);
  pinMode(Relais5, OUTPUT);
  pinMode(Relais6, OUTPUT);
  pinMode(Relais7, OUTPUT);
  pinMode(Relais8, OUTPUT);
  
// Benötigte Digitalpins als Eingang definieren
  pinMode(15, INPUT_PULLUP);           // Pin zum Öffnen aller Fächer per Tastendruck
  pinMode(2, INPUT_PULLUP);            // Pin zur Betriebsartwahl Betrieb/Ausser Betrieb
  pinMode(coinpin, INPUT_PULLUP);      // Pin zur Münzzählung

// Benötigte Digitalpins erstmal ausschalten
  digitalWrite(Relais1, HIGH);
  digitalWrite(Relais2, HIGH);
  digitalWrite(Relais3, HIGH);
  digitalWrite(Relais4, HIGH);
  digitalWrite(Relais5, HIGH);
  digitalWrite(Relais6, HIGH);
  digitalWrite(Relais7, HIGH);
  digitalWrite(Relais8, HIGH);

// LCD initialisieren
 lcd.init();                      // initialize the lcd
 lcd.backlight();

  Serial.print("Ausser Betrieb   ");
//  AutomatAus(); // Beginn in der Betriebsart "Automat Aus"
}



// *******************************************************************************
// *****                           HAUPTPROGRAMM                             *****
// *******************************************************************************


void loop() {

if (Netzwiederkehr == true) { ;// Bei einem Neustart wird erstmal die Betriebsart "Ausser Betrieb" aufgerufen
  Serial.println("Netzwiederkehr   ");
  NetzWiederkehr();
  }

if (digitalRead (2)==LOW){    // Wahl der Betriebsart
  Serial.print("Ausser Betrieb   ");
  AutomatAus();  
}

if (digitalRead (15)==LOW){    // Wenn interner Taster gedrückt wird, alle Fächer öffnen
TuerenOeffnen();
credits = 0; // evtl. Restbetrag zurücksetzen
Fach = 1;    // Alle Fächer wurden befüllt, von vorne beginnen
}
   
Aussenhelligkeit(); // Wenn zu dunkel, dann Beleuchtung einschalten

// Es wird so lange auf einen Münzeinwurf gewartet, bis der Verkaufspreis erreicht ist, danach wird ein Glas gutgeschrieben und ausgegeben

  if (EuroBezahlt >= Honigpreis) {
//    delay(500); // Eventuellen zweiten Impuls bei Überzahlung abwarten
    credits = credits + 1;                    // Ein Glas wurde erworben
    lcd.setCursor(0, 3);
    lcd.print("  Erhalten: ");
    lcd.print(EuroBezahlt);
    lcd.print(" Euro ");
  EuroBezahlt = EuroBezahlt - Honigpreis;   // Eventuellen Überzahlungs-Betrag ermitteln
  }

  if (EuroBezahlt > 0 && coinsChange == 1) {
  Serial.print(EuroBezahlt);
  Serial.print(" Euro bezahlt ");
  Serial.print(" Aktuelles Fach: ");
  Serial.println(Fach);
  lcd.setCursor(0, 3);
  lcd.print("  Erhalten: ");
  lcd.print(EuroBezahlt);
  lcd.print(" Euro ");
    coinsChange = 0;
  }



    if ((credits > 0 ) && (Fach == 1)) {
      InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse
      Honigausgabe() ;
      digitalWrite(Relais1, LOW);       // Magnetschloss ansteuern
      delay(500);                       // wait for a second
      digitalWrite(Relais1, HIGH);      // Magnetschloss aus
      delay(500);                       // wait for a second
      digitalWrite(Relais1, LOW);       // Magnetschloss ansteuern
      delay(1000);                       // wait for a second
      digitalWrite(Relais1, HIGH);      // Magnetschloss aus
       InterruptEin(); // Interrupt nach der Türbetätigung wieder einschalten
      Fach = 2; // nächstes Fach wählen
      VielenDank ();
    }

    if ((credits > 0 ) && (Fach == 2)) {
      delay(500); // eventuellen zweiten Münzimpuls bei Überzahlung abwarten
      InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse
      Honigausgabe() ;
      digitalWrite(Relais2, LOW);       // Magnetschloss ansteuern
      delay(500);                       // wait for a second
      digitalWrite(Relais2, HIGH);      // Magnetschloss aus
      delay(500);                       // wait for a second
      digitalWrite(Relais2, LOW);       // Magnetschloss ansteuern
      delay(1000);                       // wait for a second
      digitalWrite(Relais2, HIGH);      // Magnetschloss aus
      InterruptEin(); // Interrupt nach der Türbetätigung wieder einschalten
      Fach = 3; // nächstes Fach wählen
      VielenDank ();
    }

    if ((credits > 0 ) && (Fach == 3)) {
      delay(500); // eventuellen zweiten Münzimpuls bei Überzahlung abwarten
      InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse
      Honigausgabe() ;
      digitalWrite(Relais3, LOW);       // Magnetschloss ansteuern
      delay(500);                       // wait for a second
      digitalWrite(Relais3, HIGH);      // Magnetschloss aus
      delay(500);                       // wait for a second
      digitalWrite(Relais3, LOW);       // Magnetschloss ansteuern
      delay(1000);                       // wait for a second
      digitalWrite(Relais3, HIGH);      // Magnetschloss aus      
      InterruptEin(); // Interrupt nach der Türbetätigung wieder einschalten
      Fach = 4; // nächstes Fach wählen
      VielenDank ();
    }


    if ((credits > 0 ) && (Fach == 4)) {
      delay(500); // eventuellen zweiten Münzimpuls bei Überzahlung abwarten
      InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse
      Honigausgabe() ;
      digitalWrite(Relais4, LOW);       // Magnetschloss ansteuern
      delay(500);                       // wait for a second
      digitalWrite(Relais4, HIGH);      // Magnetschloss aus
      delay(500);                       // wait for a second
      digitalWrite(Relais4, LOW);       // Magnetschloss ansteuern
      delay(1000);                       // wait for a second
      digitalWrite(Relais4, HIGH);      // Magnetschloss aus
      InterruptEin(); // Interrupt nach der Türbetätigung wieder einschalten
      Fach = 5; // nächstes Fach wählen   Fach 5 vorübergehend herausprogrammiert --------------------------------------------------------------------------------------------
      VielenDank ();
    }

    if ((credits > 0 ) && (Fach == 5)) {
      delay(500); // eventuellen zweiten Münzimpuls bei Überzahlung abwarten
      InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse
      Honigausgabe() ;
      digitalWrite(Relais5, LOW);       // Magnetschloss ansteuern
      delay(500);                       // wait for a second
      digitalWrite(Relais5, HIGH);      // Magnetschloss aus
      delay(500);                       // wait for a second
      digitalWrite(Relais5, LOW);       // Magnetschloss ansteuern
      delay(1000);                       // wait for a second
      digitalWrite(Relais5, HIGH);      // Magnetschloss aus
      InterruptEin(); // Interrupt nach der Türbetätigung wieder einschalten
      Fach = 6; // nächstes Fach wählen
      VielenDank ();
    }

    if ((credits > 0 ) && (Fach == 6)) {
      delay(500); // eventuellen zweiten Münzimpuls bei Überzahlung abwarten
      InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse
      Honigausgabe() ;
      digitalWrite(Relais6, LOW);       // Magnetschloss ansteuern
      delay(500);                       // wait for a second
      digitalWrite(Relais6, HIGH);      // Magnetschloss aus
      delay(500);                       // wait for a second
      digitalWrite(Relais6, LOW);       // Magnetschloss ansteuern
      delay(1000);                       // wait for a second
      digitalWrite(Relais6, HIGH);      // Magnetschloss aus
      Fach = 7; // Automat ist leer
      VielenDank ();
    }

    if (Fach == 7) {
      AutomatLeer ();
    }
  
}


// *******************************************************************************
// *****                           UNTERPROGRAMME                            *****
// *******************************************************************************


// Startanzeige
void Startanzeige () {
  lcd.setCursor(0, 0);
  lcd.print("     Honighaus      ");
  lcd.setCursor(0, 1);
  lcd.print(" - betriebsbereit - ");
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(1, 3);
  lcd.print(Honigpreis);
  lcd.setCursor(3, 3);
  lcd.print(" ,- Euro je Glas  ");
}


// Alle Türchen bei Tastendruck öffnen, zum Befüllen des Automaten
void TuerenOeffnen () {

InterruptAus(); // Interrupt während der Türbetätigung ausschalten, da sonst fehlerhafte Geldimpulse

  lcd.setCursor(0, 0);
  lcd.print(" Metzens Honighaus  ");
  lcd.setCursor(0, 1);
  lcd.print("   Fach 1-6 offen   ");
  lcd.setCursor(0, 2);
  lcd.print("  Automat komplett  ");
  lcd.setCursor(0, 3);
  lcd.print("     auffuellen     ");

  digitalWrite(Relais1, LOW);
  delay(1000);                       // 0,5 Sekunden warten
  digitalWrite(Relais1, HIGH);
  delay(10);                     // 0,5 Sekunden warten

  digitalWrite(Relais2, LOW);
  delay(1000);                       // 0,5 Sekunden warten
  digitalWrite(Relais2, HIGH);
  delay(10);                     // 0,5 Sekunden warten

  digitalWrite(Relais3, LOW);
  delay(1000);                       // 0,5 Sekunden warten
  digitalWrite(Relais3, HIGH);
  delay(10);                     // 0,5 Sekunden warten

  digitalWrite(Relais4, LOW);
  delay(1000);                       // 0,5 Sekunden warten
  digitalWrite(Relais4, HIGH);
  delay(10);                     // 0,5 Sekunden warten

  digitalWrite(Relais5, LOW);
  delay(1000);                       // 0,5 Sekunden warten
  digitalWrite(Relais5, HIGH);
  delay(10);                     // 0,5 Sekunden warten

  digitalWrite(Relais6, LOW);
  delay(1000);                       // 0,5 Sekunden warten
  digitalWrite(Relais6, HIGH);
  delay(1000);                     // 0,5 Sekunden warten

delay(200);

InterruptEin(); // Interrupt wieder einschalten
Startanzeige();
EuroBezahlt=0; // Eventuellen Überzahlungsbetrag auf Null setzen
Fach=1; // Fachzähler zurücksetzen

}

// Anzeige aktualisiern nach Münzeinwurf/Honigausgabe
void Honigausgabe() {
  credits=0;
   myDFPlayer.volume(Lautstaerke);        // Lautstärke des Sound-Moduls von 0 bis 30
   myDFPlayer.play(1);                    //Spiele die Biene Maja
  delay(1000);
  lcd.setCursor(0, 2);
  lcd.print("Honigausgabe, siehe ");
  lcd.setCursor(0, 3);
  lcd.print("   Fach Nummer      ");
  lcd.setCursor(15, 3);
  lcd.print (Fach);
}

// Vielen Dank für Ihren Einkauf
void VielenDank () {

  lcd.setCursor(0, 0);
  lcd.print("    Vielen  Dank    ");
  lcd.setCursor(0, 1);
  lcd.print(" fuer Ihren Einkauf ");
  lcd.setCursor(0, 2);
  lcd.print(" Bitte das Tuerchen ");
  lcd.setCursor(0, 3);
  lcd.print("  fest zudruecken ");
  delay(4000);                       // 4 Sekunden warten
  Startanzeige();                    // Startanzeige wiederherstellen
}


// Netzwiederkehr
void NetzWiederkehr() {

  lcd.setCursor(0, 0);
  lcd.print(" - ausser Betrieb - ");
  lcd.setCursor(0, 1);
  lcd.print("   Netzwiederkehr   ");
  lcd.setCursor(0, 2);
  lcd.print("  Bitte kein Geld   ");
  lcd.setCursor(0, 3);
  lcd.print("     einwerfen      ");

while (digitalRead (2)==HIGH){ ; // Solange der Betriebsartenschalter nicht betätigt wurde, abwarten
  //abwarten
}
  
    Netzwiederkehr = false;
    InterruptEin(); // Interrupt wieder einschalten

}

// Automat ausser Betrieb
void AutomatAus () {
  InterruptAus(); // Interrupt ausschalten, wenn Münzprüfer abgeschaltet
  lcd.setCursor(0, 0);
  lcd.print(" - ausser Betrieb - ");
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print("  Bitte kein Geld   ");
  lcd.setCursor(0, 3);
  lcd.print("     einwerfen      ");
 digitalWrite(Relais7, HIGH); // Münzprüfer ausschalten
 digitalWrite(Relais8, HIGH); // Beleuchtung ausschalten

while (digitalRead (2)==LOW){    // solange die Betriebsart auf "ausser Betrieb steht"
                                 //warten und ggf. auf Tastendruck zur Preisänderung reagieren
if (digitalRead (15)==LOW){     // Wenn interner Taster gedrückt wird, Honigpreis ändern
  lcd.setCursor(0, 0);
  lcd.print("Honigpreis geaendert");
  
  lcd.setCursor(0, 2);
  lcd.print("Alter Preis:        ");
  lcd.setCursor(13, 2);
  lcd.print (Honigpreis);
  lcd.setCursor(16, 2);
  lcd.print("Euro");

  Honigpreis = (Honigpreis + 1);

  if (Honigpreis == 11) {      // Honigpreis zwischen 5 und 10 Euro änderbar
      Honigpreis = 5;
    }  

  lcd.setCursor(0, 3);
  lcd.print("Neuer Preis:        ");
  lcd.setCursor(13, 3);
  lcd.print (Honigpreis);
  lcd.setCursor(16, 3);
  lcd.print("Euro");

  delay(1000);
}
}

  Serial.print("In Betrieb   ");
  digitalWrite(Relais7, LOW); // Münzprüfer ein
  EuroBezahlt=0; // Eventuellen Überzahlungsbetrag auf Null setzen
  Startanzeige();
  InterruptEin(); // Interrupt wieder einschalten

}

// Automat leer
void AutomatLeer () {
  lcd.setCursor(0, 0);
  lcd.print(" - ausser Betrieb - ");
  lcd.setCursor(0, 1);
  lcd.print("  Automat ist leer  ");
  lcd.setCursor(0, 2);
  lcd.print("  Bitte kein Geld   ");
  lcd.setCursor(0, 3);
  lcd.print("     einwerfen      ");

 EuroBezahlt = 0;
 InterruptAus(); // Interrupt ausschalten, wenn Münzprüfer abgeschaltet
 
 digitalWrite(Relais8, HIGH); // Beleuchtung ausschalten

}


//Aussenhelligkeit abfragen
void Aussenhelligkeit() {
  int Analogwert = analogRead(34);   // Wert vom Analogeingang Pin 34 lesen (0-1023)
  float AussenHelligkeit = Analogwert * 100.0 / 4095; // prozentwert errechnen  (0-100)
  if (AussenHelligkeit < (SchwellwertBeleuchtung - Hysterese)) {
    digitalWrite(Relais8, HIGH);    // Aussenbeleuchtung über Relais 8 einschalten
    Lautstaerke = LautstaerkeTag; // Lautstärke in der Nacht reduzieren
  }
  if (AussenHelligkeit > (SchwellwertBeleuchtung + Hysterese)) {
    digitalWrite(Relais8, LOW);   // Aussenbeleuchtung über Relais 8 ausschalten
    Lautstaerke = LautstaerkeNacht; // Lautstärke in der Nacht reduzieren
  }
}


void InterruptEin() {
  digitalWrite(Relais7, LOW); // Münzprüfer ein
  delay (100); // kurz abwarten
  attachInterrupt(digitalPinToInterrupt(coinpin), MuenzInterrupt, RISING); // Interrupt wieder einschalten
  }

void InterruptAus() {
  detachInterrupt(digitalPinToInterrupt(coinpin)); // Interrupt ausschalten, wenn Münzprüfer abgeschaltet
  delay (100); // kurz abwarten
   digitalWrite(Relais7, HIGH); // Münzprüfer ausschalten
  }

// Interrupt für Münzprüfer
void MuenzInterrupt() {
    
  coinsChange = 1;   
  unsigned long currentMillis = millis();
  int difference = currentMillis-oldMillis;
      Serial.print("Millis: ");
    Serial.println(difference); 
  oldMillis = currentMillis;
  if(difference < 175 && difference >165){
    EuroBezahlt = EuroBezahlt + 1;
    coinsChange = 1;
}
}