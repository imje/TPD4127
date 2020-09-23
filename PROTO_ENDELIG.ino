#include <Adafruit_BNO055.h>

byte finalScore = 0; //Variabel for å vise scoren

#define BNO055_SAMPLERATE_PERIOD_MS 10

Adafruit_BNO055 bno = Adafruit_BNO055(55); //IMU

unsigned long tnext, tnow; 

unsigned long previousMillis = 0;  //Se hvor lang tid det går mellom hvert triks
const long interval = 250;  

float acc_prew; //Forrige akselerasjon

byte antallTriks = 0; //teller



void setup(void)
{
  //serial2.begin(9600);
  Serial.begin(9600);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  // Initialise IMU
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  tnext = millis() + 100;
}

void loop(void)
{
  
   unsigned long currentMillis = millis(); 

  //Koden for å hente ut akselerasjon i forhold til jordens akse er hentet fra https://forums.adafruit.com/viewtopic.php?f=45&t=114125#wrap (innlegg av brukeren gammaburst, publisert Thu Mar 23, 2017) 
  
  // Henter negative quaternion
  imu::Quaternion quat = bno.getQuat();
  quat.x() = -quat.x();
  quat.y() = -quat.y();
  quat.z() = -quat.z();

  // Henter lineærakselerasjonen (ekskluderer gravitasjonen)
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Beregner akselerasjon i z-retning i forhold til jorden
  float acc;
  acc =   (2*(quat.x()*quat.z() + quat.w()*quat.y()))*linearaccel[0] +   (2*(quat.y()*quat.z() - quat.w()*quat.x()))*linearaccel[1] + (1-2*(quat.x()*quat.x() + quat.y()*quat.y()))*linearaccel[2];

  //Serial.println(acc);

  if(acc < -10 && acc_prew > -10) { //hvor den forrige akselerasjonen var større enn -10 og den nåværende er mindre enn -10 regnes det som ett triks
    if (currentMillis - previousMillis <= 300 && currentMillis - previousMillis >=150 ) { //men hvis tiden mellom triksene er for liten regnes det som om den spretter i bakken
         previousMillis = currentMillis;
         finalScore = antallTriks;
        Serial.println(finalScore); //sender final score til appen
        return;
       }
    }

  
  if(acc < -10 && acc_prew > -10) {
    if (finalScore > 0) {
        finalScore = 0; //setter finalScore tilbake til null dersom ballen har truffet bakken og personen har begynt å trikse igjen
        antallTriks = 1;
        }
    if (currentMillis - previousMillis >= interval) { //tar vekk uønskede "dobbelt-touch" 
      
      previousMillis = currentMillis;
      Serial.println((int)antallTriks); //sender antall triks til appen
      antallTriks += 1; //øker triks
    }     
  }
  
  acc_prew = acc; //setter denne akselrasjonen til å være den forrige neste gang
  
  tnow = millis();
  delay(tnext - tnow); //delay som passer til hvor ofte akselrasjonen hentes
  tnext += BNO055_SAMPLERATE_PERIOD_MS;
}
