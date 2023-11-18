/*
   Complete Project Details https://randomnerdtutorials.com
*/
//Sms Alarm to phone
#include <GPRS_Shield_Arduino.h>
#include <SoftwareSerial.h>
#define PIN_TX    1
#define PIN_RX    0
#define BAUDRATE  19200

#define PHONE_NUMBER1 "+255767268856"  //At the moment, we want to receive the sms alarm bu you may also decide to be called
#define MESSAGE_Soil_Alarm  " Udongo unahitaji kumwagiliwa"

//#define PHONE_NUMBER1 "+255767268856"  //This is also the one being called. The rest are for sending the SMS
#define MESSAGE_Tank_Alarm  " Maji yameisha kwenye tank"

//#define PHONE_NUMBER1 "+255767268856"
#define MESSAGE_After_Irrigation_Alarm  "Nimemaliza kumwagilia"



//hapa najaribu atomatic sms
//#define PHONE_NUMBER1 "+255767268856"
#define MESSAGE7  "Sitamwagilia, maji kwenye tank"

//#define PHONE_NUMBER2 "+255767268856"
#define MESSAGE8  "Sitamwagilia, unyevunyevu unatosha"

//#define PHONE_NUMBER3 "+255767268856"
#define MESSAGE9  "Sitamwagilia, unyevunyevu unatosha ila kuwa makini, tank halina maji"
//mwisho wa kujaribu

GPRS gprsTest(PIN_TX, PIN_RX, BAUDRATE); //RX,TX,BaudRate

/*#if defined(ARDUINO_ARCH_SAMD)//deliting of sms in simCard
  // for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
  #define Serial SerialUSB
  #endif

*/

//Analog values from Mc sensor
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)


//Ultrasonic sensor
#define echoPin 11 // Echo Pin
#define trigPin  12 // Trigger Pin
long duration, distance; // Duration used to calculate distance
//const int LED = 6;


// Include Software Serial library to communicate with GSM
//#include <SoftwareSerial.h>

// Configure software serial port
SoftwareSerial SIM0(7, 8);

// Variable to store text message
String textMessage;

// Create a variable to store Lamp state
String lampState = "Hapana";

//Variable to store waterTank state
String waterTankState = " Maji yapo";

//Variable to store soilState
String soilState = "Undongo ni mkavu";

// Relay connected to pin 12
const int relay = 13;
int NetworkLED = 6;

void setup() {
  // Automatically turn on the shield
  digitalWrite(9, HIGH);
  delay(1000);
  digitalWrite(9, LOW);
  delay(5000);

  // Set relay as OUTPUT
  pinMode(relay, OUTPUT);

  //Set networkLED as OUTPUT
  pinMode(NetworkLED, OUTPUT);

  // By default the relay is off
  digitalWrite(relay, LOW);

  // Initializing serial commmunication
  Serial.begin(19200);
  SIM0.begin(19200);

  // Give time to your GSM shield log on to network
  delay(20000);
  Serial.print("SIM0 ready...");
  digitalWrite(NetworkLED, HIGH); //Taa ya kuonesha kama network imeanza kusoma

  // AT command to set SIM0 to SMS mode
  SIM0.print("AT+CMGF=1\r");
  delay(100);
  // Set module to send SMS data to serial out upon receipt
  SIM0.print("AT+CNMI=2,2,0,0,0\r");
  delay(100);

  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration / 58.2;

  //Serial.print("distance = ");
  //Serial.print(distance);
  //Serial.println("cm");

  /*  if ( distance > 90) {            //No water in Tank, send automatic sms to a mobile phone
      //START SENDING SMS
      //Serial.println("start to send message ...");
      gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE_Tank_Alarm); //define phone number and text
      //delay(7200000);
    }
  */

  //SMS ISSUES
  if (SIM0.available() > 0) {
    textMessage = SIM0.readString();
    Serial.print(textMessage);
    delay(10);

  }


  if (textMessage.indexOf("Zima") >= 0) {
    // Turn on relay and save current state
    digitalWrite(relay, LOW);
    lampState = "Hapana";
    //Serial.println("Relay set to OFF");
    textMessage = "";
  }
  //Mapping of soil moisture sensor to get the output values instead of the sensor values
  // read the input on analog pin 0:
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);

  // print the results to the Serial Monitor:
  //Serial.print("sensor = ");
  //Serial.print(sensorValue);
  //Serial.print("\t output = ");
  //Serial.println(outputValue);

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);

  //Water available in a tank, soil is moist, then pump should not work
  if (textMessage.indexOf("Washa") >= 0 && distance <= 90 &&  outputValue < 100) {

    // Turn off relay and save current state
    digitalWrite(relay, LOW);
    lampState = "Hapana, hapakuwa na sababu ya kumwagilia";
    waterTankState = " Tank lilikuwa na maji";
    soilState = "Unyevunyevu wa udongo ulikuwa unatosha ";
    //Serial.println("Relay set to ON");
    //Serial.println(distance);
    textMessage = "";
  }


  //No water in a tank, then pump should not work
  /* if (textMessage.indexOf("Washa") >= 0 && distance >= 90 ) {

     // Turn off relay and save current state
     digitalWrite(relay, LOW);
     lampState = "Hapana";
     waterTankState = "Hakuna maji kwenye tanki";
     soilState = "Hali ya udongo haijulikani";
     //Serial.println("Relay set to off");
     //Serial.println(distance);
     textMessage = "";
     //send automatic sms without being asked on height of water in tank
     gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE_Tank_Alarm); //define phone number and text
    }*/

  //Water available in a tank but soil is not moist, then pump should work
  while (textMessage.indexOf("Washa") >= 0 && distance <= 90 && outputValue < 100) {

    //send automatic sms without being asked on moisture content of soil
    gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE_Soil_Alarm); //define phone number and text
    // Turn off relay and save current state
    digitalWrite(relay, HIGH);
    lampState = "Ndio";
    waterTankState = " Kwasababu maji yalikuwa kwenye tank";
    soilState = " na udongo ulikuwa mkavu";
    //Serial.println("Relay set to off");
    //Serial.println(distance);
    textMessage = "";
  }
  //Sababu za kufanya tanki ikate kazi za umwagiliaji ni iwapo udogo umepata unyevunyevu wa kutosha au maji yameisha kwenye Tank
  if ( outputValue < 100 || distance >= 90) {
    digitalWrite(relay, LOW);
    //send automatic sms baada ya kuonesha pump imemaliza kumwagilia
    gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE_After_Irrigation_Alarm); //define phone number and text
    lampState = "Hapana";
    soilState = "Unyevunyu umetosha";
    textMessage = "";
  }

  //if (distance >90){
  // digitalWrite(relay, LOW);
  //  waterTankState = "Hakuna maji kwenye tanki";
  //  textMessage = "";
  //}


  // program za kujalibu kujibiwa direct baazi ya vitu bila kutuma sms ya kuuliza kama ilimwagilia
  if (textMessage.indexOf("Washa") >= 0 && distance > 90) {

    //send automatic sms without being asked on moisture content of soil
    gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE7); //define phone number and text
  }

  if (textMessage.indexOf("Washa") >= 0 && outputValue < 100) {

    //send automatic sms without being asked on moisture content of soil
    gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE8); //define phone number and text
  }


  if (textMessage.indexOf("Washa") >= 0 && outputValue < 100 && distance > 90) {

    //send automatic sms without being asked on moisture content of soil
    gprsTest.sendSMS(PHONE_NUMBER1, MESSAGE9); //define phone number and text
  }
  //mwisho wa atomatic program za hapo juu


  if (textMessage.indexOf("Ulimwagilia?") >= 0) {
    String message = " " + lampState;
    sendSMS(message);
    String message2 = " " +  waterTankState;
    sendSMS(message2);
    String message3 = " " +  soilState;
    sendSMS(message3);

    //Serial.println("Lamp state resquest");
    textMessage = "";
  }
}

// Function that sends SMS
void sendSMS(String message) {
  // AT command to set SIM0 to SMS mode
  SIM0.print("AT+CMGF=1\r");
  delay(100);

  // REPLACE THE X's WITH THE RECIPIENT'S MOBILE NUMBER
  // USE INTERNATIONAL FORMAT CODE FOR MOBILE NUMBERS
  SIM0.println("AT + CMGS = \"+255767268856\"");
  delay(100);
  // Send the SMS
  SIM0.println(message);
  delay(100);

  // End AT command with a ^Z, ASCII code 26
  SIM0.println((char)26);
  delay(100);
  SIM0.println();
  // Give module time to send SMS

  delay(5000);
}
