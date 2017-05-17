/********************************************************************************************************************
 *                                           Radiant Alter - Station Alpha                                          *
 *  This software is intended for use with the constructed Station Alpha hardware, and is protected under GPL v3.0. *
 *  This software is provided "as is" and any express or implied warranties, including, but not limited to, the     *
 *  implied warranties of merchantability and fitness for a particular purpose are disclaimed. In no event shall the* 
 *  provider or contributors be liable for any direct, indirect, incidental, special, exemplary, or consequential   *
 *  damages however caused through the use of this software, even if advised of the possibility of such damage.     *
 ********************************************************************************************************************/
// Required Arduino libraries
#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>
#include <Adafruit_GFX.h>   
#include <Adafruit_TFTLCD.h> 
#include <TouchScreen.h>
#include <CountUpDownTimer.h>

// Function definitions
void Diagnostics(void);
char SendCmd(char data);
void Calibrate(void);
void Game(void);
void NumberofPlayers(void);
void PlayerStatistics(void);
void Alarm(void);
void AccumulateScores(int numPlyr, int lazers, int minutes, int secs);

/******************************************Communication Variables*******************************/
// Identifies the transceiver to Station Bravo (1)
bool radioNumber = 0;
// Sets radio to SPI bus pins CN & CSN on Arduino Mega (See Fritzing Diagram for pin out)
RF24 radio(7, 8);
byte addresses[][6] = {"1Alpha", "2Bravo"};
const char ALLON = 'A';     // Constant variable passed to SendCmd(), activates all laser obstacles on Station Alpha
const char LEDSTROBE = 'F'; // Constant variable passed to SendCmd(), activates red LED array on broken laser event
const char TESTCOMMS = 'G'; // Constant variable passed to SendCmd(), activates green LED array if comms test succeeds
bool commsValid = false;    // Set to true to bypass Diagnostics() halt on communications failure 

/*****************************************Define Sensor Algorithm Variables****************************/
int cdsSensor[] = {A0, A1, A2, A3}; //Coresponds to analog pins connecting respective CDS cells
char lsrDesignation[] = {'B', 'C', 'D', 'E'}; //Commands corresponding to respective laser on Bravo
const int numCDS = 4;  // Total number of CDS sensors currently attached
int curValue;          // Current brightness value held for sensor 
int prevValue[numCDS]; // Previous brightness value held for current sensor
int baseValue[numCDS]; // Base brightness value for current sensor
int trigAlarm[numCDS]; // Current sensor that has sensed a broken laser event
int sensitivity = 50;  // Default value, modify this setting to compensate for brightness in maze environment

/*****************************************TouchScreen*********************************************/

// See Fritzing Diagram for touch screen pin out
#define YP A14 
#define XM A15  
#define YM 23   
#define XP 22   
// Defines touch screen area
#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940
// Instantiate Touchscreen object
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// See Fritzing Diagram for LCD pin out
#define LCD_CS A14
#define LCD_CD A15
#define LCD_WR 30
#define LCD_RD 31
// optional
#define LCD_RESET A4 // Not used, but definition is required

// Assign human-readable names to 16-bit color values used
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Instantiate the LCD object
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

/*********************************************Game Play*****************************************/
int screenNum = 0;    // Set between screens, ensures touch events correspond to screen buttons
int brknLZRcount = 0; // The number of laser obstacles broken during players turn, reinitialized 
                      // to zero at start of each players turn
int plyrCount;        // The number of players chosen from NumPlayers()
int plyrNum;          // Assigns the current player
int player[4];        // Max number of players
int lzrCount[4];      // Holds players broken lazer count
int gameMin[4];       // Holds players minutes in completing maze
int gameSec[4];       // Holds players seconds in completing maze
// Button states ensure touch events are not activated at wrong time during game play
bool buttonGreen = false;
bool buttonRed = false;
bool buttonTime = false;
bool buttonNext = false;

/*********************************************Game Timer*****************************************/  
CountUpDownTimer T(UP, HIGH); // Set timer type to count up with high fidelity

void setup()
{
  /******************************************Communication Setup**********************************/
  Serial.begin(115200); // Ensure baud rate is set appropriatly in Serial Monitor
  radio.begin();

  // Set the transmitters power to low to avoid burnout of transceiver over time
  radio.setPALevel(RF24_PA_LOW);

  // Open communication channel with Bravo
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

   // Set the transceiver to listen
  radio.startListening();
  Serial.println(F("Station Alpha Listening..."));
  /************************************************TouchScreen******************************************/
  tft.reset();
  // Ensures that TFT LCD touchscreen driver is supported by AdaFruit TFTLCD and graphics libraries
  uint16_t identifier = tft.readID();
   if(identifier == 0x9325) 
  {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    return;
  }
  tft.begin(identifier);
  tft.setRotation(1); // Sets LCD to landscape (90 degrees)
  tft.fillScreen(BLACK);
  pinMode(13, OUTPUT);
  
  /******************************************Define Digital/Analog Pin**********************************/
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  for (int i = 0; i < numCDS; i++)
     pinMode(cdsSensor[i], INPUT);
   /* UNCOMMENT GAME SCREEN FUNCTIONS BELOW ONE AT A TIME TO DEBUG OPERATION*/
   Diagnostics(); // Default start up screen during normal operation.
   //Calibrate();
   //Game();
   //NumberofPlayers();
   //PlayerStatistics();
   
   // Initialize game timer
   T.SetStopTime(15,0,0); // Max time in maze is 15 minutes
   T.StartTimer();
} 

// Define Min and Max touch pressure to activate touch event
#define MINPRESSURE 10
#define MAXPRESSURE 1000

void loop()
/* This is a continuous loop that operates the broken laser algorithm, senses touch screen events, and runs the game timer
 * PRE:  Function specific variables and objects are initialized to default values
 * POST: Broken laser algorithm monitors sensor input, triggering events during gameplay, touch screen listener waits for input from the touchscreen*/
{
  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint(); // Touch screen listener gets touch area for touch event activation
  digitalWrite(13, LOW);
  // Since custom wiring harness was built for TFT LCD, pins are shared by LCD and touch screen. (See Fritzing Diagram)
  // Ensures touchscreen uses the correct read/write pins
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  
  // Broken laser algorithm 
  for (int i = 0; i < numCDS; i++)
  {
      curValue = analogRead(cdsSensor[i]);
      /* UNCOMMENT THE CODE BELOW TO TROUBLESHOOT CDS SENSOR INPUT, SENSOR VALUES, AND CORRECT SENSOR AND LASER ARE PAIRED */
      // Serial.println(curValue*10+i);
      // Serial.println(lsrDesignation[i]);
      if (curValue < (baseValue[i] - sensitivity) && prevValue[i] < (baseValue[i] - sensitivity) && trigAlarm[i] == false)
      {
        trigAlarm[i] = true;
        Serial.println("Alarm");
        // Serial.println("Alarm Activated!");
        brknLZRcount++;
        SendCmd(lsrDesignation[i]);
        SendCmd(LEDSTROBE);
        Alarm();
      }
      else if (curValue > (baseValue[i] - sensitivity) && prevValue[i] > (baseValue[i] - sensitivity))
        trigAlarm[i] = false;
  
       prevValue[i] = curValue;
  }
   // Menu: Waits for touch event
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) 
  {
      // Restrict touch screen area to LCD area. Since LCD is rotated to landscape (90 degrees) X, Y coordinates are revesred.
       p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
       p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);    
       
     // If pressure is sensed at p.x and p.y coordinates and current screen is NumberofPlayers(1) - One Player
      if (p.y > 30 && p.y < 70 && p.x > 110 && p.x < 190)
        if (screenNum == 1)
        {
          plyrCount = 1;
          tft.drawRect(25, 100, 60, 60, WHITE); 
          tft.drawRect(165, 100, 60, 60, RED); 
          tft.drawRect(235, 100, 60, 60, RED); 
          tft.drawRect(95, 100, 60, 60, RED);
        }
         
      if (p.y > 90 && p.y < 130 && p.x > 110 && p.x < 190)// Two Players
        if (screenNum == 1)
        {
          plyrCount = 2;
          tft.drawRect(95, 100, 60, 60, WHITE);
          tft.drawRect(25, 100, 60, 60, RED); 
          tft.drawRect(165, 100, 60, 60, RED); 
          tft.drawRect(235, 100, 60, 60, RED); 
        } 
        
      if (p.y > 140 && p.y < 180 && p.x > 110 && p.x < 190) // Three Players
        if (screenNum == 1)
        {
          plyrCount = 3;
          tft.drawRect(165, 100, 60, 60,  WHITE); 
          tft.drawRect(95, 100, 60, 60, RED); 
          tft.drawRect(25, 100, 60, 60, RED);
          tft.drawRect(235, 100, 60, 60, RED);
        }
      if (p.y > 190 && p.y < 220 && p.x > 110 && p.x < 190) // Four Players
        if (screenNum == 1)
        {
          plyrCount = 4;  
          tft.drawRect(235, 100, 60, 60, WHITE);
          tft.drawRect(165, 100, 60, 60, RED);          
          tft.drawRect(95, 100, 60, 60, RED); 
          tft.drawRect(25, 100, 60, 60, RED); 
        }
      if (p.y > 90 && p.y < 180 && p.x > 20 && p.x < 70) // Start Game Button
        if (screenNum == 1)
        {
           Game();
           tft.reset(); // Ensures removal of any drawing artifacts
        }
        
      // If pressure is sensed at p.x and p.y coordinates, current screen is Game(2), and specific buttonState is set - Start
      if (p.y > 14 && p.y < 70 && p.x > 15 && p.x < 88)
      { 
        if (screenNum == 2)
        {
           if(buttonGreen == true) // Ensures other buttons touch area is disabled while button displayed
           {
            // Draw stop button
            tft.fillRoundRect(110, 180, 110, 60, 5, RED); 
            tft.setCursor(133,200); 
            tft.setTextColor(WHITE); 
            tft.setTextSize(3);
            tft.print("STOP"); 
            // Remove start button
            tft.fillRoundRect(0, 180, 110, 60, 5, BLACK);
            tft.setCursor(10,200); 
            tft.setTextColor(BLACK); 
            tft.setTextSize(3);
            tft.print("");
            buttonTime = true;   // Start timer
            buttonGreen = false; // Disables start button touch area
            buttonRed = true;    // Enables stop button touch area
            buttonNext = false;  // Disables next button touch area
            brknLZRcount = 0;    // Reset broken laser count to zero to get accurate count during players turn       
           }    
        }
      } 
      
      // If pressure is sensed at p.x and p.y coordinates, current screen is Game(2), and specific buttonState is set - Stop
      if (p.y > 103 && p.y < 170 && p.x > 14 && p.x < 70)
      { 
        if (screenNum == 2) 
        {     
          if (buttonRed == true) // Ensures other buttons touch area is disabled while button displayed
          { 
            // Remove Stop button
            tft.fillRect(110, 180, 110, 60, BLACK); 
            tft.setCursor(130,200); 
            tft.setTextColor(BLACK); 
            tft.setTextSize(3);
            tft.print("");
            // Draw Next button
            tft.fillRoundRect(220, 180, 99, 61, 5, BLUE);
            tft.setCursor(238,200); 
            tft.setTextColor(WHITE); 
            tft.setTextSize(3);
            tft.print("NEXT");
            buttonTime = false; // Stop timer
            buttonRed = false;     // Disables stop button touch area       
            buttonNext = true;     // Enables next button touch area
            // Add scores to array
            AccumulateScores(plyrNum, brknLZRcount, T.ShowMinutes(), T.ShowSeconds());
          }
        }
      }  
    
      // If pressure is sensed at p.x and p.y coordinates, current screen is Game(2), and specific buttonState is set - Next Button   
      if (p.y > 184 && p.y < 235 && p.x > 14 && p.x < 70)
      {
        if (screenNum == 2)
        {
          if (buttonNext == true) // Ensures other buttons touch area is disabled while button displayed
          {
            if (plyrNum < plyrCount)
             {
              plyrNum++;
              buttonNext = false; // Disables next button touch area
              SendCmd('A');
              Game();
              T.ResetTimer(); // Reset timer between turns
              tft.reset(); // Ensures removal of any drawing artifacts from screen
             }
             else
             {
              buttonNext = false;
              PlayerStatistics();
              tft.reset(); // Ensures removal of any drawing artifacts from screen
             }
          }
        }
      }
      // If pressure is sensed at p.x and p.y coordinates, current screen is PlayerStatistics(3) - New Game Button
      if (p.y > 90 && p.y < 180 && p.x > 20 && p.x < 70)
      {
        if (screenNum == 3)
        {
          plyrNum = 1;
          NumberofPlayers();  
        }
      }
        /* UNCOMMENT THE CODE BELOW TO TROUBLE SHOOT TOUCH SCREEN MAPPING ISSUES AND TOUCH PRESSURE */
        //Serial.print("X = "); Serial.print(p.x);
        //Serial.print("\tY = "); Serial.print(p.y);
        //Serial.print("\n"); //Serial.println(p.z);
    }

    // Game Timer
    if (buttonTime == true)
    {
      T.Timer();
      if (T.TimeHasChanged())
      {
          // Overwrite each number with a black box on time change, avoids numbers overwriting each other
          tft.fillRect(65, 75, 75, 42, BLACK); 
          tft.setCursor(98, 75);
          tft.setTextColor(WHITE); 
          tft.setTextSize(6);
          tft.print(String(T.ShowMinutes()));

          tft.setCursor(138, 75);
          tft.print(":");
          // Overwrite each number with a black box on time change, avoids numbers overwriting each other
          tft.fillRect(178, 75, 70, 42, BLACK);
          tft.setCursor(178, 75);
          tft.print(String(T.ShowSeconds()));
      }
   }
}

char SendCmd(char data)
/* This function sends control commands to Station Bravo 
 * PRE:  A char variable is received corresponding to a specific command function on Station Bravo
 * POST: The command is sent, verification of receipt is received*/
{
  Serial.println(data);
  radio.stopListening();
  /* UNCOMMENT SERIAL.PRINTLN STATEMENTS TO TROUBLESHOOT COMMUNICATION */
  // Send command to station Bravo
  //Serial.println(F("Now sending"));
  unsigned long start_time = micros();
  if (!radio.write(&data, sizeof(data)))
    Serial.println(F("Send Failed"));
  radio.startListening();
  unsigned long started_waiting_at = micros();
  boolean timeout = false;

  while (!radio.available())
  {
    if (micros() - started_waiting_at > 200000 )
    {
      timeout = true;
      break;
    }
  }
  if (timeout)
    Serial.println(F("Send Failed, the response timed out."));
  else
  {
    // Verify data was received by Bravo
    char verify;
    radio.read( &verify, sizeof(verify));
    if (verify == 'G')
      commsValid = true;
    // Debug information
    unsigned long end_time = micros();
    //Serial.print(F("Sent "));
    //Serial.print(start_time);
    //Serial.print(F(", Got response "));
    //Serial.print(verify);
    //Serial.print(F(", Round-trip delay "));
    //Serial.print(end_time - start_time);
    //Serial.println(F(" microseconds"));
  }
  delay(1000);
}

void Diagnostics(void)
/* This function tests initial communication between Stations Alpha and Bravo,  
 * calibrates the laser obstacles, and reports the number of sensors found
 * PRE:  Station Bravo is powered on
 * POST: The number of players screen is displayed. If there is an error in communications 
 * it is displayed on the screen with troubleshooting instructions*/
{
  tft.fillScreen(BLACK);
  tft.setCursor(0,0); 
  tft.setTextColor(WHITE); 
  tft.setTextSize(3);
  tft.print("Radiant");
  tft.setTextColor(RED); 
  tft.print("Alter");
  tft.setCursor(0,30);
  tft.setTextColor(YELLOW); 
  tft.setTextSize(1);
  tft.println("Running Diagnostics...");
  delay(2000);
  tft.println("Testing communication...");
  SendCmd(TESTCOMMS); 
  delay(6000);
  if (commsValid == true)
     {
     SendCmd(ALLON);
     tft.setTextColor(WHITE);
     tft.println("Communication Successful");
     delay(4000);
     tft.setTextColor(YELLOW); 
     tft.println("Initializing LAZERS...");
     delay(4000);
     tft.println("Calibrating Sensors...");
     Calibrate();
     delay(4000); 
     tft.setTextColor(WHITE);
     tft.println("Calibration Complete");
     tft.println("CDS sensors found: " + String(numCDS));
     tft.setTextSize(1);
     tft.println("Diagnostics Complete!");
     delay(4000);
     NumberofPlayers();
     }
  else 
    {
     tft.setCursor(0,50);
     tft.setTextColor(RED); tft.setTextSize(2);
     tft.println("Communication Error");
     tft.setTextColor(WHITE); tft.setTextSize(1);
     tft.println("No response from Station Bravo.");
     tft.println("Ensure Bravo is powered on first!");
     tft.println("If the problem persists debug via");
     tft.println("Serial Monitor per user manual");
    }
}

void Calibrate(void)
/* This function establishes a baseline reading for the CDS sensors, there is a global sensitivity  
 * value that can be set to compensate for the amount of light in the maze environment 
 * PRE:  Available sensors are added to the cdsSensors[] array and receiving input from the lasers on Station Bravo
 * POST: The average baseline value for each sensor is set. */
{
  Serial.println("Calibrating Sensors");
  for (int i = 0; i < numCDS; i++)
  {
    baseValue[i] = analogRead(cdsSensor[i]);
    delay(20);
    baseValue[i] += analogRead(cdsSensor[i]);
    delay(20);
    baseValue[i] += analogRead(cdsSensor[i]);
    baseValue[i] /= 3;
  }
  /* UNCOMMENT TO TROUBLESHOOT CALIBRATION */
  // Serial.println("Calibration Succeded");
}

void Alarm(void)
/* This function activates the LED array and an audible alarm if a sensors read value falls below the baseline value
 * PRE:  The sensors are receiving input from the lasers
 * POST: The LED array flashes twelve times and the alarm sounds for 1000 microseconds*/
{
  for (int i = 0; i <= 12; ++i)
  {
    digitalWrite(6, HIGH);
    analogWrite(9, 20);
    delay(80);
    digitalWrite(6, LOW);
    analogWrite(9, 0);
    delay(80);
    ++i;  
  }
}

void NumberofPlayers(void)
/* This function loads the GUI interface that allows a player to select the number of participants.
 * PRE:  The Diagnostics subroutine completed all tasks successfully. One player is set as the default.
 * POST: The global numPlayers variable is updated, Game() is called*/
{
  // Intialize game variables
  screenNum = 1; 
  plyrNum = 1;
  plyrCount = 1;
  // Format screen output
  tft.fillScreen(BLACK);
  tft.setCursor(37,0);
  tft.setTextColor(WHITE); 
  tft.setTextSize(3);
  tft.print("Select Players");
  tft.drawRect(25, 100, 60, 60, RED); 
  tft.drawRect(95, 100, 60, 60, RED); 
  tft.drawRect(165, 100, 60, 60, RED); 
  tft.drawRect(235, 100, 60, 60, RED); 
  tft.setCursor(48,120); tft.print("1");
  tft.setCursor(118,120); tft.print("2");
  tft.setCursor(187,120); tft.print("3");
  tft.setCursor(256,120); tft.print("4");
  tft.fillRoundRect(95, 180, 128, 60, 5, GREEN); 
  tft.setTextColor(WHITE);
  tft.setCursor(125,200); 
  tft.print("Next");
  tft.drawRect(25, 100, 60, 60, WHITE);
}

void Game(void)
/* This function loads the game screen GUI that starts and stops the players time and advances to the next player 
 * PRE:  The global numPlayers variable has been updated
 * POST: The global brknLaser and plyrTime variables are reinitialized to zero.*/
{
  // Intialize game variables
  screenNum = 2;
  buttonGreen = true; // Enables start button touch area
  T.ResetTimer();
  // Format screen output
  tft.fillScreen(BLACK);
  tft.drawRoundRect(8, 50, 305, 90, 5, RED);
  tft.setCursor(100,0); 
  tft.setTextColor(YELLOW); 
  tft.setTextSize(3);
  tft.print("Player " + String(plyrNum));

  tft.fillRoundRect(0, 180, 110, 60, 5, GREEN);
  tft.setCursor(10,200); 
  tft.setTextColor(WHITE); 
  tft.setTextSize(3);
  tft.print("START");  
  SendCmd('A');
}

void PlayerStatistics(void)
/* This function loads the player statistics GUI that displays the individual players time and number of broken lasers
 * PRE:  The global brknLaser and plyrTime variables are updated.
 * POST: Player number, global brknLaser and plyrTime variables are displayed */
{
  // Intialize game variables
  screenNum = 3;
  // Format screen output
  tft.fillScreen(BLACK);
  tft.setCursor(40,0); 
  tft.setTextColor(WHITE); 
  tft.setTextSize(3);
  tft.print("Player Scores");

  tft.setCursor(30,35); 
  tft.setTextColor(WHITE); 
  tft.setTextSize(2);
  tft.print("Player");
  
  tft.setCursor(130,35); 
  tft.setTextColor(RED); 
  tft.setTextSize(2);
  tft.print("LAZERS");

  tft.setCursor(232,35); 
  tft.setTextColor(WHITE); 
  tft.setTextSize(2);
  tft.print("Time");

  tft.fillRoundRect(80, 180, 170, 60, 5, GREEN); 
  tft.setTextColor(WHITE);
  tft.setCursor(94,200); 
  tft.setTextSize(3);
  tft.print("New Game");
  // Print scores to screen
  int Y = 60;
  for (int i = 0; i < plyrCount; i++)
  {
    tft.setCursor(38,Y); 
    tft.setTextColor(YELLOW); 
    tft.setTextSize(2);
    tft.println("  "+ String(player[i]) + "       " + String(lzrCount[i]) + "     " + String(gameMin[i]) + ":" + String(gameSec[i]));
    Y = Y + 30; // Adds 30 pixels to space scores
  }
}

void AccumulateScores(int numPlyr, int lazers, int minutes, int secs)
/* This function receives the current players number, broken laser count, and time and adds them to the appropriate array
 * PRE:  The STOP button has been pressed by the user. 
 * POST: The current players scores are added to the appropriate array  */
{
  int i = (plyrNum - 1); // Get correct array index value
  player[i] = numPlyr;
  lzrCount[i] = lazers;
  gameMin[i] = minutes;
  gameSec[i] = secs;
}





