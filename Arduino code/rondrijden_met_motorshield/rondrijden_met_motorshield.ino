#define AJoystickHorizontaal A5
#define AJoystickVerticaal A6

#define W1Vooruit 5
#define W1Achteruit 6
#define W2Vooruit 9
#define W2Achteruit 8

//joystick as
int VJoystickHorizontaal = 0;
int VJoystickVerticaal = 0;
int VJoystickZas = 0;
long X_as = 0;
long Y_as = 0;
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
double OUDLWielVooruit = 0;
double OUDLWielAchteruit = 0;
double OUDRWielVooruit = 0;
double OUDRWielAchteruit = 0;
int Ouderest = 0;

//schaal voor de kracht
float motorPower = 0.5;
long joystickkracht = 0;
long joystickkracht2 = 0;
int joystickX = 0;
int joystickY = 0;

//joystick midden
int X_min = -60;
int X_max = 60;
int Y_min = -60;
int Y_max = 60;

void setup() {

  //Serial communicatie
  Serial.begin(9600);

    //pinnen voor pwm van de wielen
  pinMode(W1Vooruit, OUTPUT);
  pinMode(W1Achteruit, OUTPUT);
  pinMode(W2Vooruit, OUTPUT);
  pinMode(W2Achteruit, OUTPUT);

}

void loop() {
    Joystickuitlezen();
    JoysticktoMotorcontrol();
    MotorAansturing();

}

void Joystickuitlezen(){
  VJoystickHorizontaal = analogRead(AJoystickHorizontaal);
  VJoystickVerticaal = analogRead(AJoystickVerticaal);

  /*
  Serial.println();
  Serial.print("LMW");
  Serial.print("\t");
  Serial.print(LWV);
  Serial.print("\t");
  Serial.print("LWA");
  Serial.print("\t");
  Serial.print(LWA);
  Serial.print("\t");
  Serial.print("RWV");
  Serial.print("\t");
  Serial.print(RWV);
  Serial.print("\t");
  Serial.print("RWA");
  Serial.print("\t");
  Serial.print(RWA);
  Serial.print("\t");
  Serial.print("HOR");
  Serial.print("\t");
  Serial.print(X_as);
  Serial.print("\t");
  Serial.print("VER");
  Serial.print("\t");
  Serial.print(Y_as);
  Serial.print("\t");
  Serial.print("KRACHt");
  Serial.print("\t");
  Serial.print(joystickkracht);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("KRACHt2");
  Serial.print("\t");
  Serial.print(joystickkracht2);
  Serial.print("\t");
  */
  
  X_as = VJoystickHorizontaal - 320;
  Y_as = VJoystickVerticaal - 320;
}

//Afhankelijk van de coordinaten in het x,y assenstelsel de kracht en richting van de motoren bepalen.
void JoysticktoMotorcontrol(){
  joystickkracht2 = (X_as*X_as)+(Y_as*Y_as);
  joystickkracht = sqrt(joystickkracht2);
  
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
    LWielVooruit = joystickkracht;
    LWielAchteruit = 0;
    RWielVooruit = joystickkracht-sqrt(X_as*X_as);
    RWielAchteruit = 0;
  }
  //rechts boven
  else if (Y_as>Y_max&&X_as>0){
    joystickkracht = sqrt((X_as*X_as)+(Y_as*Y_as));
    LWielVooruit = joystickkracht-sqrt(X_as*X_as);
    LWielAchteruit = 0;
    RWielVooruit = joystickkracht;
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

    LWV = LWielVooruit;
    LWA = LWielAchteruit;
    RWV = RWielVooruit;
    RWA = RWielAchteruit;
  
  //in stappen over een halfe seconden naar de snelheid gaan.
  /*if((millis()%50)< Ouderest){
    LWV = ((LWielVooruit - OUDLWielVooruit) / 10)+OUDLWielVooruit;
    LWA = ((LWielAchteruit - OUDLWielAchteruit) / 10)+OUDLWielAchteruit;
    RWV = ((RWielVooruit - OUDRWielVooruit) / 10)+OUDRWielVooruit;
    RWA = ((RWielAchteruit - OUDRWielAchteruit) / 10)+OUDRWielAchteruit;
    //Serial.println(LWV);
    //Serial.println(LWA);
    //Serial.println(RWV);
    //Serial.println(RWA);
  }*/
  Ouderest = (millis()%50);
}

void MotorAansturing(){
  if(LWV>250){LWV=250;}
  if(LWA>250){LWA=250;}
  if(RWV>250){RWV=250;}
  if(RWA>250){RWA=250;}
  analogWrite(W1Vooruit, (LWV*motorPower)); // max 255
  analogWrite(W1Achteruit, (LWA*motorPower)); // max 255
  analogWrite(W2Vooruit, (RWV*motorPower)); // max 255
  analogWrite(W2Achteruit, (RWA*motorPower)); // max 255
}
