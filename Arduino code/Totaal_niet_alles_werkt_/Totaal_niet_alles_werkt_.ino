//#include <ros.h>
//#include <std_msgs/String.h>


//------------------Pinnen layout----------------
//Analoog
#define Analoog A4
#define AJoystickHorizontaal A5
#define AJoystickVerticaal A6
#define AJoystickZas A7

//Spanning accu's
#define Accu1 A8
#define Accu2 A9

//Motordrivers
#define W1Vooruit 2
#define W1Achteruit 3
#define W2Vooruit 4
#define W2Achteruit 5
#define W3Vooruit 6
#define W3Achteruit 7

//Startknop
#define Startknop1 9
#define Startknop2 10


//Encoders
#define Encoder1 2
#define Encoder2 3
#define Encoder3 4
#define Encoder4 5

//Relais
#define R_K8_ 23
#define R_K7_ 25
#define R_K6_Powerknop 27
#define R_K5_ 29
#define R_K4_ 31
#define R_K3_ 33
#define R_K2_ 41
#define R_K1_ 37

//Knoppen
#define K46_Joystick 46
#define K48_Omhoog 48
#define K50_Omlaag 50
#define Knoppen4 52

//------------------Variable----------------

//joystick as
int VJoystickHorizontaal = 0;
int VJoystickVerticaal = 0;
int VJoystickZas = 0;
int X_as = 0;
int Y_as = 0;
int sensorValue3 = 0;

//joystick knop
int VJKnop = 0;

//gewenste pwm
int LWielVooruit = 0;
int LWielAchteruit = 0;
int RWielVooruit = 0;
int RWielAchteruit = 0;

//echte pwm
int LWV = 0;
int LWA = 0;
int RWV = 0;
int RWA = 0;

//oude pwm en timer ervoor
int OUDLWielVooruit = 0;
int OUDLWielAchteruit = 0;
int OUDRWielVooruit = 0;
int OUDRWielAchteruit = 0;
int Ouderest = 0;

//schaal voor de kracht
float motorPower = 0.1;
int joystickkracht = 0;
int joystickX = 0;
int joystickY = 0;

//joystick midden
int X_min = -60;
int X_max = 60;
int Y_min = -60;
int Y_max = 60;

//stoelverhoging
int VStoelOmhoog = 0;
int VStoelOmlaag = 0;

//powerknop en timer ervoor
int VPowerStatus = 1;
int VPowerTimer = 0;
bool BPower = false;

//uiterste batterij
float BatterijMin = 2.5;
float BatterijMax = 3.3;
float BatterijPercentageberekenen = (BatterijMax-BatterijMin);
float BatterijPercentage = 0;
float BatterijSpanning1 = 0;
float BatterijSpanning2 = 0;

//testen
int i = 0;
int test = 0;

//-----------------Program---------------------
void setup() {
  Startup();
}

void loop() {
  Joystickuitlezen();
  JoysticktoMotorcontrol();
  MotorAansturing();
  JoystickKnop();
  StoelVerhoging();
  Powerknop();
  Batterij();
}

//---------Functie's---------------------------------

//pinnen knoppelen
void Startup(){

  //Serial communicatie
  Serial.begin(9600);

  //knop joystick
  pinMode(K46_Joystick, INPUT);
  pinMode(13, OUTPUT);
  
  //pinnen voor pwm van de wielen
  pinMode(W1Vooruit, OUTPUT);
  pinMode(W1Achteruit, OUTPUT);
  pinMode(W2Vooruit, OUTPUT);
  pinMode(W2Achteruit, OUTPUT);
  pinMode(W3Vooruit, OUTPUT);
  pinMode(W3Achteruit, OUTPUT);

  //relais aanzetten motoren -----------------------uit wanneer niet nodig bij testen
  pinMode(R_K1_,HIGH);
  pinMode(R_K2_,HIGH);
  pinMode(R_K3_,HIGH);
  pinMode(R_K4_,HIGH);
  pinMode(R_K5_,HIGH);
  pinMode(R_K6_Powerknop,HIGH);

  //Spanning meten over de accu's
  pinMode(R_K7_, HIGH);
  pinMode(R_K8_, HIGH);
  
  //Powerknop
  pinMode(Startknop1,INPUT);
  pinMode(Startknop2, HIGH);

  //algemeen stroom naar de motoren -----------------------batterij besparen dus uit ffuit wanneer niet nodig bij testen
  //pinMode(R_K6_Powerknop,HIGH);

  //hoger pwm voor de motor geluiden
  //TCCR0B = TCCR0B & 0b11111000 | <setting>;
}

//Functie voor de knop van de joystick
void JoystickKnop(){
  if(VJKnop == HIGH){
    //Knop is ingedrukt

  } else {
    //Knop is niet ingedrukt

  }
}

//analoog pinnen uitlezen en waarde in een x en y assenstelsel plaatsen.
//knop uitlezen
void Joystickuitlezen(){
  VJoystickHorizontaal = analogRead(AJoystickHorizontaal);
  VJoystickVerticaal = analogRead(AJoystickVerticaal);
  VJoystickZas = analogRead(AJoystickZas);

  //Knop
  VJKnop = digitalRead(K46_Joystick);
  
  X_as = VJoystickHorizontaal - 510;
  Y_as = VJoystickVerticaal - 510;
}

//Afhankelijk van de coordinaten in het x,y assenstelsel de kracht en richting van de motoren bepalen.
void JoysticktoMotorcontrol(){
  
  //Midden
  if (X_as>X_min&&X_as<X_max&&Y_as>Y_min&&Y_as<Y_max){
    LWielVooruit = 0;
    LWielAchteruit = 0;
    RWielVooruit = 0;
    RWielAchteruit = 0;
    //Serial.println("Center stil");
  }
  // verticaal stil
  else if (Y_as>Y_min&&Y_as<Y_max){
    if (X_as>X_max){
      LWielVooruit = 500;
      LWielAchteruit = 0;
      RWielVooruit = 0;
      RWielAchteruit = 500;
      //Serial.println("Rechtsom");
    }
    if (X_as<X_min){
      LWielVooruit = 0;
      LWielAchteruit = 500;
      RWielVooruit = 500;
      RWielAchteruit = 0;
      //Serial.println("Linksom");
    }
    //Serial.println("Verticaal stil");
  }
  //Links boven
  else if (Y_as>Y_max&&X_as<0){
    joystickkracht = sqrt((X_as*X_as)+(Y_as*Y_as));
    LWielVooruit = joystickkracht-sqrt(X_as*X_as);
    LWielAchteruit = 0;
    RWielVooruit = joystickkracht;
    RWielAchteruit = 0;
  }
  //rechts boven
  else if (Y_as>Y_max&&X_as>0){
    joystickkracht = sqrt((X_as*X_as)+(Y_as*Y_as));
    LWielVooruit = joystickkracht;
    LWielAchteruit = 0;
    RWielVooruit = joystickkracht-sqrt(X_as*X_as);
    RWielAchteruit = 0;
  }
  //rechts onder
  else if (Y_as<Y_min&&X_as>X_max){
    joystickkracht = sqrt((X_as*X_as)+(Y_as*Y_as));
    LWielVooruit = 0;
    LWielAchteruit = joystickkracht;
    RWielVooruit = 0;
    RWielAchteruit = joystickkracht-sqrt(X_as*X_as);
  }
  else if (Y_as<Y_min&&X_as<X_min){
    joystickkracht = sqrt((X_as*X_as)+(Y_as*Y_as));
    LWielVooruit = 0;
    LWielAchteruit = joystickkracht-sqrt(X_as*X_as);
    RWielVooruit = 0;
    RWielAchteruit = joystickkracht;
  }


  
  //in stappen over een halfe seconden naar de snelheid gaan.
  if((millis()%50)< Ouderest){
    LWV = ((LWielVooruit - OUDLWielVooruit) / 10)+OUDLWielVooruit;
    LWA = ((LWielAchteruit - OUDLWielAchteruit) / 10)+OUDLWielAchteruit;
    RWV = ((RWielVooruit - OUDRWielVooruit) / 10)+OUDRWielVooruit;
    RWA = ((RWielAchteruit - OUDRWielAchteruit) / 10)+OUDRWielAchteruit;
    //Serial.println(LWV);
    //Serial.println(LWA);
    //Serial.println(RWV);
    //Serial.println(RWA);
  }
  Ouderest = (millis()%50);
}

void MotorAansturing(){
  analogWrite(W1Vooruit, (LWV*motorPower)); // max 255
  analogWrite(W1Achteruit, (LWA*motorPower)); // max 255
  analogWrite(W2Vooruit, (RWV*motorPower)); // max 255
  analogWrite(W2Achteruit, (RWA*motorPower)); // max 255
}

//Door de knop invoer word de motor aangestuurd
//Om omhoog of omlaag te bewegen
void StoelVerhoging(){
  //Knop
  VStoelOmhoog = digitalRead(K48_Omhoog);
  VStoelOmlaag = digitalRead(K50_Omlaag);

  if(VStoelOmhoog == HIGH && VStoelOmlaag == LOW){
    //Omhoog is ingedrukt
    analogWrite(W3Vooruit, (100)); // max 255
    analogWrite(W3Achteruit, (0)); // max 255
  } 
  if(VStoelOmhoog == LOW && VStoelOmlaag == HIGH){
    //Omlaag is ingedrukt
    analogWrite(W3Vooruit, (0)); // max 255
    analogWrite(W3Achteruit, (100)); // max 255
  }
}

//Aan en uitschakelen van het systeem
void Powerknop(){

  //na 3 seconden gaat het systeem uit)
  if(VPowerStatus==LOW&&digitalRead(Startknop1)==HIGH){
    //pinMode(R_Powerknop,HIGH);

    VPowerTimer = ((millis()%10000)/1000);
    BPower = true;
    Serial.println("ingedrukt");
  }
  VPowerStatus = digitalRead(Startknop1);

  if(BPower&&VPowerTimer+3 > 9){
    VPowerTimer = VPowerTimer - 7;
  }
  if(BPower&&VPowerTimer+3 == ((millis()%10000)/1000)){
    // relay die de stroom uitzet.
    pinMode(R_K6_Powerknop,HIGH);
  }
  
  //met Nuc
  /*if(VPowerStatus=LOW&&digitalRead(Startknop1)==HIGH){
    pinMode(R_Powerknop,HIGH);
  }
  VPowerStatus = digitalRead(Startknop1);*/
}

//Het uitlezen van de analoog pinnen waar de accu op aangesloten is met een spanningsdeler.
//hierdoor de spanning meten en het percentage van de accu's berekenen.
void Batterij(){
  BatterijSpanning1 = analogRead(Accu1);
  BatterijSpanning2 = analogRead(Accu2);
  //Voor het testen
  Serial.println(BatterijSpanning1* (3.3 / 1023.0));
  Serial.println(BatterijSpanning2* (3.3 / 1023.0));
  //Serial.println((BatterijSpanning1-BatterijMin));
  //Serial.println(BatterijPercentageberekenen);
  
  BatterijPercentage = (BatterijSpanning1-BatterijMin)/BatterijPercentageberekenen*100;
  //Serial.println(BatterijPercentage);
}
