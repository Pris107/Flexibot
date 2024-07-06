//this code is for a arduino powered robot
#include <avr/interrupt.h>
#define SOFT_UART_DISABLE_INTERRUPT // Disable interrupt handling for SoftwareSerial
#include <AltSoftSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define rx 10 //mp3 response
#define tx 11 // mp3 trigger
AltSoftSerial mySerial(rx,tx);
#define IN1 6 //motor
#define IN2 9
#define IN3 5
#define IN4 3
//Assume IN1 and IN2 are for the left motor, IN3 and IN4 are for the right.
#define TrigPin1 8 //ultrasonic sensor
#define EchoPin1 7
#define cmdStor 0x09 //look for storage device
#define cardAdr 0x02 //sd card address
#define cmdPwVol 0x22 //command play with volume
#define cmdstop 0x16 //command stop playing


int getDist(int EchoPin, int TrigPin);
int Distance1;
long pulseDuration;
const int maxDistance = 7;
bool moveab; //Check to see if robot can move

//Movement Commands
void brake();
void circle_cw(int);
void forward(int);
void backward(int);
void zigzag();
void circle_ccw(int);
void sr(int); //start right motor
void sl(int); //start left motorl

void moverand(int);
void sendCommand(int8_t, int8_t, int8_t); //send command, command number on left then command parameters
void playSong(int, int); //play a song, volume 'v', file name 'f'xxx.mp3
void stopSong();



LiquidCrystal_I2C lcd(0x27, 16, 2); //lcd screen size
int scoreA = 0, scoreB = 0, timer, player = 0; // Initialize variables with default values
bool multi = false; //check if multiplayer

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TrigPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);
  delay(500);
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(200);
  sendCommand(cmdStor, 0, cardAdr);
  delay(200);
  moveab = true;
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP); //taget board A
  pinMode(4, INPUT_PULLUP); //target board B
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  interrupts();

  PCICR |= (1 << PCIE2);    // Enable pin change interrupts for PCINT23..16 (PCINT2_vect)
  PCMSK2 |= (1 << PCINT10) | (1 << PCINT20); // Enable interrupts for pin 2 and pin 4
}

void how_many_players() //multiplayer or single player mode
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("How many players?");
  lcd.setCursor(0, 1);
  lcd.print("1. left   2.right ");

  while (true) {
    if (!digitalRead(2)) {
      player = 1; //activate single player mode
      break;
    }
    if (!digitalRead(4)) {
      player = 2; //activate multiplayer mode
      break;
    }
  }
}

void difficulty_mode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Choose mode:");
  lcd.setCursor(0, 1);
  lcd.print("Normal       Easy");
  delay(500);
  lcd.clear();

   while (true) {
    if (!digitalRead(2)) {
      timer = 30; //activate single player mode
      break;
    }
    if (!digitalRead(4)) {
      timer = 15 ; //activate multiplayer mode
      break;
    }
  }

}

void single_player_screen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Single player mode");
  lcd.setCursor(0, 1);
  lcd.print("Try to reach 10!");
  delay(500);
  lcd.clear();

  while (timer > 0) {
    lcd.setCursor(0, 0);
    if (timer>=10) 
    lcd.print("Time: ");
    else lcd.print("Time: 0");
    lcd.print(timer); //30 second count down
    lcd.setCursor(0, 1);
    lcd.print("Score: ");
    lcd.print(scoreA);
    delay(1000);
    Serial.print("Score A: "); Serial.print(scoreA); Serial.println();
    Serial.print("Score B: "); Serial.print(scoreB); Serial.println();
    Serial.print("Time: "); Serial.print(timer); Serial.println();
    Serial.println();
    timer--;
  }

  if (scoreA >= 10) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You win!");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You lose!");
  }
  lcd.setCursor(0, 1);
  lcd.print("Reset to restart");
  noInterrupts();
}

void multiplayer_screen() {
  lcd.clear();
  while (scoreA<10 && scoreB<10){
    lcd.setCursor(0, 0);
    lcd.print("A score is:");
    lcd.print(scoreA);

    lcd.setCursor(0, 1);
    lcd.print("B score is:");
    lcd.print(scoreB);
    delay(100);
    Serial.print("Score A: "); Serial.print(scoreA); Serial.println(); //debugging
    Serial.print("Score B: "); Serial.print(scoreB); Serial.println();
  }
  

  if (scoreA >= 10) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("A wins! Good Job!");
  }
  if (scoreB >= 10) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("B wins! Good Job");
  }
  lcd.setCursor(0, 1);
  lcd.print("Reset to restart");
  noInterrupts();

}

ISR(PCINT2_vect) {
  if (!multi) {
    if (!digitalRead(2) || !digitalRead(4))
      scoreA++;
  } else {
    if (!digitalRead(2))
      scoreA++;
    if (!digitalRead(4))
      scoreB++;
  }
}

//motor
/* playSong: Accepts 2 arguments, v for volume, f for folder name (01, 02, etc)
   Example: playSong(15,1) plays song 01xxx.mp3 at volume 15
*/

void playSong(int v, int f){
  sendCommand(cmdPwVol, v, f);
}

/* stopSong: Stops song. */

void stopSong(){
  sendCommand(cmdstop,0,0);
}
/* sc: Send Command
   Accepts 3 arguments, cmd for cmd name, byte d1 and d2 for data needed in command
   Example: sc(0x22,15,01) sends the "Play With Volume" command for song 01xxx.mp3 at volume 15
*/

void sendCommand(int8_t cmd, int8_t data1, int8_t data2){
  delay(20);
  int8_t Send_buf[8] = {0};
  Send_buf[0] = 0x7e;
  Send_buf[1] = 0xFF;
  Send_buf[2] = 0x06;
  Send_buf[3] = cmd;
  Send_buf[4] = 0x00;
  Send_buf[5] = data1;
  Send_buf[6] = data2;
  Send_buf[7] = 0xef;
  for (uint8_t i = 0;i < 8;i++){
    mySerial.write(Send_buf[i]);
    Serial.print(Send_buf[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
} 

void moverand(int t){
  int n = 6;
  int r = rand()%n;
  switch (r){
  case 1: circle_cw(t); break;
  case 2: forward(t); break;
  case 3: backward(t); break;
  case 4: zigzag(); break;
  case 5: circle_ccw(t); break;
  }
  //create a random number, start a movement according to number for 't' ms
}
void brake(){
    sendCommand(0x0D,0,0);
    Serial.println("Braking...");
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,HIGH);
    //brake by having all motor pins set to HIGH
}


void circle_cw(int t){
  if (moveab) {
    stopSong();
    Serial.println("Rotating in clockwise circle");
    int spd = 128, slspd = 127;
    sl(spd);
    delay(t);
    brake();
    //have the left motor move to achieve a clockwise circular rotation, move this way for 't' ms.
  } else {
    Serial.println("Unable to act.");
  }
}

void circle_ccw(int t){
  if (moveab) {
    stopSong();
    Serial.println("Rotating in counter-clockwise circle");
    int spd = 128;
    sr(spd);
    delay(t);
    brake();
    //have right motor move to achieve counter-clockwise circular rotation, move for 't' ms.
  } else {
    Serial.println("Unable to act.");
  }
}

void forward(int t){
  if (moveab) {
    stopSong();
    Serial.println("Moving forward");
    int spd = 128;
    sb(spd);
    delay(t);
    brake();
    //move forward at speed 'spd' for 't' milliseconds
  } else {
    Serial.println("Unable to act.");
  }
}

void backward(int t){
  if (moveab) {
    stopSong();
    
    Serial.println("Moving backward");
    int spd = 128;
    analogWrite(IN2,spd);
    digitalWrite(IN1,LOW);
    analogWrite(IN4,spd);
    digitalWrite(IN3,LOW);
    delay(t);
    brake();
    //move back at speed 'spd' for 't' milliseconds
  } else {
    Serial.println("Unable to act.");
  }
}

void zigzag(){
  if (moveab) {
    stopSong();
    
    Serial.println("Moving zigzag");
    int spd = 101, mspd = 127, turntime = 220, movetime = 400, buf = turntime * (0.2);
    analogWrite(IN1, spd);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    Serial.println("Z:right");
    delay(turntime);
    //move left motor to turn right, turn for 'turntime' ms
    for(int i=0;i<3;i++){
      Serial.println("Z:NE");
      sr(spd);
      delay(movetime);
      //restart right motor to move forward for 'movetime' ms
      Serial.println("Z:left");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      delay(turntime);
      //stop left motor, let car turn left, twice of 'turntime' to get back to forward position and then turn left the same angle
      Serial.println("Z:NW");
      sl(spd);
      delay(movetime);
      //restart left motor to move forward for 'movetime' ms
      Serial.println("Z:right");
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      delay(turntime*1.4);
      //stop right motor, let car turn right, twice of 'turntime' to get back to forward position and then turn right the same angle
    }
    //for loop causes car to make 'i' cycles of 'move, turn left, move, turn right'
    brake();
  } else {
    Serial.println("Unable to act.");
  }
}

void sl(int s) {
  analogWrite(IN1,s);
  digitalWrite(IN2, LOW);
}

void sr(int s){
  analogWrite(IN3, s);
  digitalWrite(IN4, LOW);
}

void sb(int s){
  sl(s);
  sr(s);
}

int getDist(int EchoPin, int TrigPin) {
  digitalWrite(TrigPin, LOW);
delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  pulseDuration = pulseIn(EchoPin, HIGH);
  int cm = pulseDuration / 58;
  if (cm>maxDistance) {moveab = false; brake();}
  else moveab = true;
  return pulseDuration / 58; // Convert to cm
}

void loop() {

  how_many_players(); //single or multiplayer mode
  if (player == 1) {
    difficulty_mode(); //normal or difficult setting
    single_player_screen(); //score and timer displayed,override when timer reaches 0
  } else if (player == 2) {
    multi = true;
    multiplayer_screen();
  }
  playSong(80,3);
  delay(15000);
  stopSong();
  delay(10000);
  circle_cw(2000);
  delay(10000);
  forward(2000);
  delay(10000);
  backward(2000);
  delay(10000);
  zigzag();
  delay(10000);
  circle_ccw(2000);
  delay(10000);
}