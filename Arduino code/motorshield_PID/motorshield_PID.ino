/*#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
*/
/*
zwart = aarde
geel = 5v
rood = gnd
oranje = interrupt
bruin = interrupt

encoder

zwart   = zwart   = aarde
geel  = geel    = 5v
rood  = blauw   = gnd
oranje  = oranje  = interrupt
bruin = bruin   = interrupt
*/

// Oude programma struct
/*
struct Wheel {
    double speedCommand = 0;        // desired pulses per second
    double totalOutput = 0;         // output of the PID controller
    double error = 0;               // current error
    double somError = 0;            // sommation of the error
    double deltaError = 0;          // delta from the error
    double previousError = 0;   

    double encoderValue = 0;
    double pulsesPerSecond = 0;     
    double linearVelocity = 0;      // [m/s]
    double radiantsPerSecond = 0;   // [rad/s]
} LeftWheel, RightWheel;

volatile long lastEncodedA = 0;
volatile long lastEncodedB = 0;

double pulsesPerRotation = 27466;     // max pulses per rotation
double meterPerPuls = 0.0000343143;   // distance per pulse in meters
double wheelRadius = 0.15;            // meters
double baseLine = 0.56;               // meters

double linearSpeed = 10;               // [m/s]
double angularSpeed = 0;              // [rad/s]

double x = 0.0;     // x-position
double y = 0.0;     // y-position    
double th = 0.0;    // angular position

double vx = 0;
double vy = 0;
double vth = 0;

double delta_x = 0.0;       // delta x-position
double delta_y = 0.0;       // delta y-position
double delta_th = 0.0;      // delta angular position

float Kp = 0.2;                   
float Ki = 0.4;
float Kd = 0.0;
*/
//einde van het oude programma

//De encoder maakt 256 pulsen per rotatie van de motor 
// maar omdat er geen gegevens zijn gevonden over de overbrengen word er een schatting gedaan van het aantal pulsen
// schatting is 70 revolutie's voor de encoder voor 1 revolutie van het wiel komt neer op 17920
// radies van wiel is 0.16 m
// 0.16 * Pi = 0.50265

//kan beter maar is oke
double MeterPerRevolutie = 0.50265;
int PulsenPerRevolutie = 17920;
double MeterPerPuls = 0.0000280499;

int Speed = 100;
 
int Vooruit = 5;
int Achteruit = 6;

int InterruptVooruit = 2;
int InterruptAchteruit = 3;
int G = 0;
int L = 0;

//Hz
int clockfrequentie = 2000;
int Hz = 100;
int Teller = clockfrequentie/Hz;
int ms = 1000/Hz;
int i = 0;

//PID
int Pulsen = 0;
int GemPulsen = 0;
double Snelheid = 0;
float RPM = 0;

//PID constants
double kp = 13;
double ki = 0.08;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

//voor aan of uit
String SerialString = "";
String SerialAan = "aan";
String SerialUit = "uit";
bool Testen = false;

//Hard stop
long Stop = 6000;
long StopCount = 0;
int aantalPID = 301;
int PIDp[301];
double PIDi[301];
int PIDd[301];
long PIDRPM[301];
int PIDGem[301];
float pulsnaarrpm = 0.3348214;
long PIDCount = 0;
bool A = false;
bool B = true;

void setup()
{
  //PID
  setPoint = 20;

  //Communicatie ros
  /*nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);*/
  
  //cheatsheet voor alle pwm https://playground.arduino.cc/Main/TimerPWMCheatsheet/
  //fast pwm door de prescale_factor naar 1 te plaatsen
  //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  //TCCR0B = _BV(CS00);
  pinMode(Achteruit, OUTPUT);
  pinMode(Vooruit, OUTPUT);
  analogWrite(Achteruit, 0);
  analogWrite(Vooruit, Speed);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(InterruptVooruit), ISRVooruit, RISING); // rising falling testen
  attachInterrupt(digitalPinToInterrupt(InterruptAchteruit), ISRAchteruit, RISING); // rising falling testen

  //timer interrupt
  
  cli();//stop interrupts

//set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei();//allow interrupts
}
 
void loop(){
  /*Serial.println("----------");
  Serial.print("G : ");
  Serial.println(G);
  /*Serial.print("PID : ");
  Serial.println(PIDCount);*/
  
  if (StopCount<Stop){
    analogWrite(Vooruit, Snelheid);
  }else{
    analogWrite(Vooruit, 0);
    A = true;
  }
  if(A==true&&B==true){
    for(int L = 0; L < PIDCount; L++){
      Serial.println();
      Serial.print(L);
      //Serial.print("\t");
      //Serial.print(PIDGem[L]);
      Serial.print("\t");
      Serial.print(PIDp[L]);
      Serial.print("\t");
      Serial.print(PIDi[L]);
      Serial.print("\t");
      Serial.print(PIDd[L]);
      Serial.print("\t");
      Serial.print("\t");
      Serial.print(PIDRPM[L]);
    }
    B = false;
  }

  // com naar ros
  //Communicatie();
  
}

/*void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}*/

//communicatie met ros
/*void Communicatie(){
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
}*/

void ISRVooruit(){
  G++;
}

void ISRAchteruit(){
  L++;
}

//interupt software
//timer0 interrupt 2kHz
ISR(TIMER0_COMPA_vect){
  if(StopCount<Stop){
    i++;
    StopCount++;
  }
  if(i==Teller){
    i=0;
    //Code voor PID
    Pulsen = G;
    G -= Pulsen;
    if(Pulsen>GemPulsen){GemPulsen++;}
    if(Pulsen>GemPulsen+5){GemPulsen++;}
    if(Pulsen>GemPulsen+10){GemPulsen++;}
    if(Pulsen>GemPulsen+15){GemPulsen++;}
    if(Pulsen>GemPulsen+20){GemPulsen++;}
    if(Pulsen<GemPulsen-5){GemPulsen--;}
    if(Pulsen<GemPulsen-10){GemPulsen--;}
    if(Pulsen<GemPulsen-15){GemPulsen--;}
    if(Pulsen<GemPulsen-20){GemPulsen--;}
    if(Pulsen<GemPulsen){GemPulsen--;}
    RPM = (GemPulsen * pulsnaarrpm);
    //RPM = GemPulsen * Hz * 60 * MeterPerRevolutie;

    //PID
    error = setPoint - RPM;                                // determine error
    cumError += error * ms;                // compute integral
    rateError = (error - lastError)/ms;   // compute derivative
 
    Snelheid = kp*error + ki*cumError + kd*rateError;                //PID output    
    if (Snelheid>250){Snelheid = 250;}
    PIDGem[PIDCount] = GemPulsen;
    PIDp[PIDCount] = kp*error;
    PIDi[PIDCount] = ki*cumError;
    PIDd[PIDCount] = kd*rateError;
    PIDRPM[PIDCount] = RPM;
    
    PIDCount++;
 
    lastError = error;                                //remember current error
  }
}
