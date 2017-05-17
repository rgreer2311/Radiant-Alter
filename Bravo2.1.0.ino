/********************************************************************************************************************
 *                                           Radiant Alter - Station Bravo                                          *
 *  This software is intended for use with the constructed Station Bravo hardware, and is protected under GPL v3.0. *
 *  This software is provided "as is" and any express or implied warranties, including, but not limited to, the     *
 *  implied warranties of merchantability and fitness for a particular purpose are disclaimed. In no event shall the* 
 *  provider or contributors be liable for any direct, indirect, incidental, special, exemplary, or consequential   *
 *  damages however caused through the use of this software, even if advised of the possibility of such damage.     *
 ********************************************************************************************************************/

// Required Arduino libraries
#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>
#include "Keypad.h"
#include <LiquidCrystal.h>

// Function definitions
void LaserControl(char lsrCmd);
void KeyPressed(KeypadEvent pressdKey);
void CheckPin(void);
void RedLED(bool activateRed);
void GreenLED(void);
void CodeEntry(void);
void CodeGenerator(void);
void AccessDenied(void);
void AccessGranted(void);
bool EvaluatePin(bool *isValid);
void ResetPin(void);
void AppendPin(char alpha);

/******************************************Communication Variables*******************************/
// Identifies the transceiver to Station Alpha (0)
bool radioNumber = 1;
// Sets radio to SPI bus pins CN & CSN on Arduino Mega (See Fritzing Diagram for pin out)
RF24 radio(7, 8);
byte addresses[][6] = {"1Alpha", "2Bravo"};

/*********************************************Keypad Variables**********************************/
// Define keypad array
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {{'1', '2', '3', 'A'}, {'4', '5', '6', 'B'}, {'7', '8', '9', 'C'}, {'*', '0', '#', 'D'}};
// Define digital row and column pins for keypad
byte rowPins[ROWS] = {14, 15, 16, 17};
byte colPins[COLS] = {18, 19, 20, 21};
// Define Keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
// Define secret pin and position variables
char newPin[6];  // Global variable for the generated pin number. Called in in CodeGenerator(), CodeEntry(), and CheckPin()
char tempPin[6]; // Global variable for the user supplied pin number. Called in EvaluatePin(), ResetPin(), AppendPin()
int currPos = 5; // Global variable to control '*' printed on LCD during pin entry. Called in KeyPressed(), ResetPin(), CheckPin()

/**********************************************LCD Digital Pins*********************************/
// Define LCD object
LiquidCrystal lcd(22, 23, 27, 26, 25, 24);

void setup()
/* This function initializes function specific variables and objects
 * PRE:  Global variables and data structures are defined 
 * POST: Function specific variables and data structures are initialized to default values*/
{
  Serial.begin(115200); // Ensure baud rate is set appropriatly in Serial Monitor
  /******************************************Communication Setup*********************************/
  radio.begin();
  // Set the transmitters power to low
  radio.setPALevel(RF24_PA_LOW);
  // Open communication channel with Alpha
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  // Set the transceiver to listen
  radio.startListening();
  Serial.println(F("Station Bravo Listening..."));

  /********************************************LCD Setup ***************************************/
  // Initialize the LCD
  lcd.begin(16, 2);

  /*****************************************Define Digital/Analog Pins***************************/
  // See Fritzing Diagram for pin out
  pinMode(6, OUTPUT);           // Red LED array    D6
  pinMode(5, OUTPUT);           // Green LED array  D5
  pinMode(10, OUTPUT);          // Laser One        D10
  pinMode(11, OUTPUT);          // Laser Two        D11
  pinMode(12, OUTPUT);          // Laser Three      D12
  pinMode(13, OUTPUT);          // Laser Four       D13

  /******************************************Event listener for keypad***************************/
  // Initialize the Keypad
  keypad.addEventListener(KeyPressed);

  // Displays code entry screen on LCD 
  CodeEntry();
}

void loop()
/* This is a continuous loop that receives transceiver input and calls related functions
 * PRE:  Function specific variables and objects are initialized to default values
 * POST: Transceiver listens for commands from Station Alpha, keypad listener and waits for input from the keypad*/
{
     // Listen for commands from Station Alpha
    char cmd;
    if(radio.available())
    {
      // Receive command from Alpha
      while (radio.available()) 
      {                                  
        radio.read( &cmd, sizeof(cmd));  
      }
      // Send acknowledgement  
      radio.stopListening();                                           
      radio.write( &cmd, sizeof(cmd) );    
      radio.startListening();
      /* UNCOMMENT SERIAL.PRINTLN STATEMENTS BELOW TO DEBUG COMMUNICATION*/                                
      //Serial.print(F("Response sent "));
      //Serial.println(cmd);  

      // Pass command to LaserControl
      LaserControl(cmd);
      // Re-initialize cmd to blank for next command
      cmd = ' ';
  }
    
  // Wait for keypad entry
  keypad.getKey();
}

void LaserControl(char lsrCmd)
/* This function controls the laser obstacles, and the red and green LED arrays.
*  PRE: A char variable is received from "loop()" corresponding to a specific case in the switch statement.
*  POST: A laser obstacle is activated or deactivated, RedLED() or GreenLED() is called.*/
{
  bool brknLaser = true;
  /* UNCOMMENT SERIAL.PRINTLN STATEMENTS BELOW TO DEBUG FUNCTIONALITY*/
  switch (lsrCmd)
  {
    case 'A':
      // All lasers on
      digitalWrite(10, HIGH);
      digitalWrite(11, HIGH);
      digitalWrite(12, HIGH);
      digitalWrite(13, HIGH);
      //Serial.println("All lasers on");
      break;

    case 'B':
      // Laser ONE off
      //Serial.println("Laser ONE off");
      digitalWrite(10, LOW);
      break;

    case 'C':
      // Laser TWO off
      //Serial.println("Laser TWO off");
      digitalWrite(11, LOW);

      break;
    case 'D':
      // Laser THREE off
      //Serial.println("Laser THREE off");
      digitalWrite(12, LOW);

      break;
    case 'E':
      // Laser FOUR off
      //Serial.println("Laser FOUR off");
      digitalWrite(13, LOW);
      break;

    case 'F':
      // Activate  array
      //Serial.println("Red LED strobe activated");
      RedLED(brknLaser);
      break;
      
    case 'G':
      // Diagnostic Comms check
      //Serial.println("Comm check with Aplha");
      GreenLED();
      break;
      
    default:
      Serial.println("Error in LaserControl function.");
  }
}

void KeyPressed(KeypadEvent pressdKey)
/* This function accepts user input from the keypad
*  Pre:  The function is called from the keypad event listener when the user presses a key on the keypad
*  Post: If reserved charachter '*' is pressed CheckPin() is called. If reserved charachter '#' is pressed ResetPin() is called.
*  By default, a charachter variable is passed to AppendKey(), currPos is advanced, and an '*' is printed on the LCD */
{
  switch (keypad.getState())
  {
    case PRESSED:
      switch (pressdKey)
      {
        case '*':
          CheckPin();
          break;

        case '#':
          ResetPin();
          lcd.setCursor(0, 1);
          lcd.print("                ");
          currPos = 5;
          lcd.setCursor(currPos, 1);
          break;

        default:
          // Reset cursor position after five digit pin
          if (currPos == 10)
          {
            currPos = 5;
          }
          AppendPin(pressdKey);
          Serial.println(pressdKey);
          Serial.println(tempPin);

          lcd.setCursor(currPos, 1);
          lcd.print('*');
          currPos += 1;
      }
  }
}

void CheckPin(void)
/* This function provides feedback based on the evaluation of the user supplied pin
*  Pre:   The function is called from KeyPressed() upon entry of the reserved charachter '*'
*  Post:  A boolean variable is passed by reference to EvaluatePin(). If returned true, ResetPin(), AccessGranted() and GreenLED() are called.
*         If false, ResetPin() and AccessDenied() are called, and a false boolean variable is passed to RedLED()*/
{
  bool valTest = false;
  EvaluatePin(&valTest);

  if (valTest == true)
  {
    Serial.println("Access Granted");
    ResetPin();
    AccessGranted();
    GreenLED();
  }
  else if (valTest == false)
  {
    Serial.println("Access Denied");
    ResetPin();
    AccessDenied();
    bool brknLaser = false;
    RedLED(brknLaser);
  }
}

void CodeGenerator(void)
/* This function generates a new random pin for the user to input from char array keys[][]
*  Pre:  The function is called from CodeEntry()
*  Post: Global variable newPin is reset with a new 5 digit number with terminal charachter advanced */
{
  // Subtle variations in analog pin 0 provide a random event for random()
  randomSeed(analogRead(0));
  
  for (int i = 0; i <= 4; ++i)
  {
    int randrowIndex = random(4);
    int randcolIndex = random(4);
    // While loop ensures that reserved charachters * and # are excluded from the new pin
    while (keys[randrowIndex][randcolIndex] == '*' || keys[randrowIndex][randcolIndex] == '#')
    {
      randrowIndex = random(4);
      randcolIndex = random(4);
    }
    newPin[i] = keys[randrowIndex][randcolIndex];
    newPin[i + 1] = '\0';
  }
}

void CodeEntry(void)
/* This function displays text "Enter Pin: " and global variable newpin on the first line of the LCD.
*  Pre:  The function is called from setup(), AccessGranted(), or AccessDenied()
*  Post: CodeGenerator() generates a new pin and it is displayed on the LCD */
{
  lcd.clear();
  CodeGenerator();
  lcd.setCursor(0, 0);
  lcd.print("Enter Pin: " + String(newPin));
}

void AccessDenied(void)
/* This function displays Access Denied on the LCD and activates the red LEDs
*  Pre:  The function is called from CheckPin() on invalid pin input from the user
*  Post: Access Denied is displayed on the LCD, a boolean value of false is passed to RedLED(),
*  delays for 500 miliseconds, CodeEntry() is called */
{
  lcd.clear();
  bool denied = false;
  lcd.setCursor(5, 0);
  lcd.print("Access");
  lcd.setCursor(5, 1);
  lcd.print("Denied");
  RedLED(denied);
  delay(500);
  CodeEntry();
}

void AccessGranted(void)
/*  This function displays Access Granted on the LCD and activates the green LEDs
*   Pre:  The function is called from CheckPin() on valid pin input from the user
*   Post: Access Granted is displayed on the LCD, GreenLED() is called, delays for 500 miliseconds,
*   LaserCommand() is passed a char variavle, CodeEntry() is called */
{
  char allOn = 'A';
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Access");
  lcd.setCursor(5, 1);
  lcd.print("Granted");
  LaserControl(allOn);
  GreenLED();
  delay(500);
  CodeEntry();
}
bool EvaluatePin(bool *isValid)
/*  This function evaluates whether the player supplied "tempPin" is equal to the global variable "newPin"
*   Pre:  The function is called from CheckPin() on entry of the reserved charachter '*'
*   Post: If equal, boolean variable isValid is returned true, else it is returned false */
{
  int validate = strcmp(tempPin, newPin);
  
  if (validate == 0)
    return *isValid = true;
  else
    return *isValid = false;
}

void ResetPin(void)
/*  This function resets the global variable "tempPin"
*   Pre:  The function is called from KeyPressed() when the the reserved charachter '#' is pressed, and from CheckPin()
*   Post: The terminal charachter of tempPin is reset to index[0] */
{
  tempPin[0] = '\0';
}

void AppendPin(char alpha)
/*  This function appends charachters typed by the player to global variable "tempPin"
*   Pre:  The function is called from KeyPressed() and receives the char variable the player types a character on the keypad
*   Post: The charachter is appended to "tempPin" and terminal charachter advanced */
{
  int len = strlen(tempPin);
  tempPin[len] = alpha;
  tempPin[len + 1] = '\0';
}

void RedLED(bool activateRed)
/* This function activates the red LED array if a laser obstacle is broken, or if an incorrect code is entered via keypad
*  PRE:  This function is provided a boolean value from LaserControl() or AccessDenied()
*  POST: If the variable "activatedRed" is true, a strobe effect is triggered indicating a laser has been broken
*        If false, the array is activated for 500 miliseconds indicating an incorrect code was entered */
{
  if (activateRed == true)
  {
    for (int i = 0; i <= 12; ++i)
    {
      digitalWrite(6, HIGH);
      delay(80);
      digitalWrite(6, LOW);
      delay(80);
      ++i;
    }
  }
  if (activateRed == false)
  {
    digitalWrite(6, HIGH);
    delay(500);
    digitalWrite(6, LOW);
  }
}

void GreenLED(void)
/* This function activates the green LED array if the initial communication test succeeds, or the player enters the correct code via keypad.
*  PRE: The function is called from LaserCommand() or AccessGranted()
*  POST: The green LED array activates for 500 miliseconds */
{
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(5, LOW);
}
