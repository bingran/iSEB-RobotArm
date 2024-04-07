#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

/* To store the calibration value for each servo motor */
Preferences preferences;


/* PWM DECLARATION START*/
// use 12 bit precission for LEDC timer
#define LEDC_TIMER_12_BIT 12
// Use a common base frequency for all servos
#define LEDC_BASE_FREQ 50

#define MIN 50
#define MAX 550

#define ARM_CHANNEL         0   /* Chanel 0 */
#define SHOULDER_CHANNEL    1   /* Chanel 1 */
#define GRIPPER_CHANNEL     2   /* Chanel 2 */
#define BASE_CHANNEL        3   /* Chanel 3 */
#define BUZZER_PWM          8 /* Channel 8 */

#define ARM_PIN             23  /* PIN 23 */
#define SHOULDER_PIN        4   /* PIN  4 */
#define GRIPPER_PIN         32  /* PIN 32 */
#define BASE_PIN            12  /* PIN 12 */
#define buzzerPin           22  /* PIN 22 */
/* PWM DECLARATION END */

/* SERVER DECLARATION START */
const char* ssid = "SMLab iRobotArm"; // SSID for the ESP32 access point
const char* password = "12345678"; // Password for the access point

IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
/* SERVER DECLARATION END */
/* MOTOR DECLARATION START */
// Motion data index
int Servo_PROGRAM = 0;

// Servos matrix
const int ALLMATRIX = 5; 
const int ALLSERVOS = 4; 

// Servo delay base time
const int BASEDELAYTIME = 20; // 10 ms

// Backup servo value
int Running_Servo_POS [ALLMATRIX] = {};

// Standby 
int Servo_Act_1 [ ] PROGMEM = {  90,  90, 90,  90,  500  };

// Servo zero position 
//Array 
int Servo_Act_0 [ ] PROGMEM = {  90,  90, 90,  90,  500  };

// Action 1 
int Servo_Prg_1_Step = 14;
int Servo_Prg_1 [][ALLMATRIX] PROGMEM = {
  //ARM,SHOULDER,BASE,GRIPPER, ms
  {  90,  90,  90,  90,   500  }, // origin
  {  90,  90,  60,  90,   1000  }, // go left 20
  {  90,  50,  60,  90,   1000  }, // go down
  {  50,  50,  60,  90,   1000  }, // shake 1
  { 130,  50,  60,  90,   1000  }, // shake 2
  { 130,  50,  60, 130,   1000  }, // grab
  {  90,  50,  60, 130,   1000  }, // arm go original 
  {  90,  90,  60, 130,   1000  }, // go up
  {  90,  90,  90, 130,   1000  }, // go right 20
  {  90,  50,  90, 130,   1000  }, // go down
  { 130,  50,  90, 130,   1000  }, // shake 1
  {  50,  50,  90, 130,   1000  }, // shake 2
  {  50,  50,  90,  90,   1000  }, // release
  {  90,  90,  90,  90,   2000  }, // origin
};

// Action 2  
int Servo_Prg_2_Step = 14;
int Servo_Prg_2 [][ALLMATRIX] PROGMEM = {
  //ARM,SHOULDER,BASE,GRIPPER, ms
  {  90,  90,  90,  90,   500  }, // origin
  {  90,  90,  60,  90,   1000  }, // go left 40
  {  90,  50,  60,  90,   1000  }, // go down
  {  50,  50,  60,  90,   1000  }, // shake 1
  { 130,  50,  60,  90,   1000  }, // shake 2
  { 130,  50,  60, 130,   1000  }, // grab
  {  90,  50,  60, 130,   1000  }, // arm go original 
  {  90,  90,  60, 130,   1000  }, // go up
  {  90,  90, 120, 130,   1000  }, // go right 40
  {  90,  50, 120, 130,   1000  }, // go down
  { 130,  50, 120, 130,   1000  }, // shake 1
  {  50,  50, 120, 130,   1000  }, // shake 2
  {  50,  50, 120,  90,   1000  }, // release
  {  90,  90,  90,  90,   2000  }, // origin
};

// Action 3 
int Servo_Prg_3_Step = 32;
int Servo_Prg_3 [][ALLMATRIX] PROGMEM = {
  //ARM,SHOULDER,BASE,GRIPPER, ms
  {  90,  90,  90,  90,   500  }, // origin
  {  90,  90,  60,  90,   1000  }, // go left 20
  {  90,  50,  60,  90,   1000  }, // go down
  {  50,  50,  60,  90,   1000  }, // shake 1
  { 130,  50,  60,  90,   1000  }, // shake 2
  { 130,  50,  60, 130,   1000  }, // grab
  {  90,  50,  60, 130,   1000  }, // arm go original 
  {  90,  90,  60, 130,   1000  }, // go up
  {  90,  90,  90, 130,   1000  }, // go right 20
  {  90,  50,  90, 130,   1000  }, // go down
  { 130,  50,  90, 130,   1000  }, // shake 1
  {  50,  50,  90, 130,   1000  }, // shake 2
  {  50,  50,  90,  90,   1000  }, // release
  {  50,  50,  90,  90,   1000  }, // shake 1
  { 130,  50,  90,  90,   1000  }, // shake 2
  { 130,  50,  90, 130,   1000  }, // grab
  {  90,  90,  90, 130,   1000  }, // go up
  {  90,  90, 120, 130,   1000  }, // go right 20
  {  90,  50, 120, 130,   1000  }, // go down
  { 130,  50, 120, 130,   1000  }, // shake 1
  {  50,  50, 120, 130,   1000  }, // shake 2
  {  50,  50, 120,  90,   1000  }, // release
  {  50,  50, 120,  90,   1000  }, // shake 1
  { 130,  50, 120,  90,   1000  }, // shake 2
  { 130,  50, 120, 130,   1000  }, // grab
  {  90,  90, 120, 130,   1000  }, // go up
  {  90,  90, 160, 130,   1000  }, // go right 20
  {  90,  50, 160, 130,   1000  }, // go down
  { 130,  50, 160, 130,   1000  }, // shake 1
  {  50,  50, 160, 130,   1000  }, // shake 2
  {  50,  50, 160,  90,   1000  }, // release
  {  90,  90,  90,  90,   2000  }, // origin
};

void ConvertDegreeToPwmAndSetServo(int iServo, int iValue)
{
  Serial.print(F("iServo: "));
  Serial.print(iServo); 
  Serial.print(F(" iValue: "));
  Serial.println(iValue);
  // Read from EEPROM to fix zero error reading
  iValue = (iValue*(MAX-MIN)/180.0)+MIN; /* convertion to pwm value */
  double NewPWM = iValue + preferences.getDouble((String(iServo)).c_str(),0);
  Serial.print(F(" NewPWM: "));
  Serial.println(NewPWM);
  /* 50 = zero degree 550 = 180 degree*/
  ledcWrite(iServo,NewPWM);
}

void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX], int iSteps)
{
  int INT_TEMP_A, INT_TEMP_B, INT_TEMP_C;

  for (int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++) { // iSteps number of step
    Serial.print(F(" iSteps: "));
    Serial.println(iSteps);
    int InterTotalTime = iMatrix[MainLoopIndex][ALLMATRIX - 1]; // InterTotalTime - total time needed

    int InterDelayCounter = InterTotalTime / BASEDELAYTIME; // InterDelayCounter time / step 

    for (int InterStepLoop = 0; InterStepLoop < InterDelayCounter; InterStepLoop++) { 

      for (int ServoIndex = 0; ServoIndex < ALLSERVOS; ServoIndex++) { 

        INT_TEMP_A = Running_Servo_POS[ServoIndex]; // servo motor current position
        INT_TEMP_B = iMatrix[MainLoopIndex][ServoIndex]; // servo motor next position

        if (INT_TEMP_A == INT_TEMP_B) { // no update in servo motor position
          INT_TEMP_C = INT_TEMP_B;
        } else if (INT_TEMP_A > INT_TEMP_B) { // servo motor position position reduce
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_A - INT_TEMP_B); 
          if (INT_TEMP_A - INT_TEMP_C >= INT_TEMP_B) {
            ConvertDegreeToPwmAndSetServo(ServoIndex, INT_TEMP_A - INT_TEMP_C);
          }
        } else if (INT_TEMP_A < INT_TEMP_B) { /// servo motor position position increase
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_B - INT_TEMP_A); 
          if (INT_TEMP_A + INT_TEMP_C <= INT_TEMP_B) {
            ConvertDegreeToPwmAndSetServo(ServoIndex, INT_TEMP_A + INT_TEMP_C);
          }
        }

      }
      delay(BASEDELAYTIME);
    }

    // back of current servo motor position
    for (int Index = 0; Index < ALLMATRIX; Index++) {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }
  }
}

void Servo_PROGRAM_Zero()
{
  /* Update zero value to servo motor variable */
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

  /* Update the servo motor to zero position */
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    ConvertDegreeToPwmAndSetServo(iServo,Running_Servo_POS[iServo]);
    delay(50);
  }
}

void motorInit() {
  // Set base frequency and resolution for all channels
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(BUZZER_PWM, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  // Attach each servo motor pin to a channel
  ledcAttachPin(ARM_PIN       , ARM_CHANNEL);       /* ARM */     /* CN0  *//* PIN 32*/
  ledcAttachPin(SHOULDER_PIN  , SHOULDER_CHANNEL);  /* SHOULDER *//* CN1 *//* PIN  4*/
  ledcAttachPin(BASE_PIN      , BASE_CHANNEL);      /* BASE */    /* CN2 *//* PIN 23*/
  ledcAttachPin(GRIPPER_PIN   , GRIPPER_CHANNEL);   /* GRIPPER */ /* CN3  *//* PIN 12*/
  ledcAttachPin(buzzerPin     , BUZZER_PWM);
}

void setup() {

  Serial.begin(115200);
  Serial.write("Hello World\n");

  // Set ESP32 as an access point
  WiFi.softAP(ssid);/* without password */
  // WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet); // Configure access point settings

  // Setup MDNS responder
  if (!MDNS.begin("iSEBRobotArm")) {
    Serial.println("Error setting up MDNS responder!");
  }

  // Start the web server
  // server.on("/", handleIndex);
  server.on("/",handleIndex);
  // server.on("/editor", handleEditor);
  server.on("/controller", handleController);
  // server.on("/zero", handleZero);
  // server.on("/setting",handleSetting);
  // server.on("/save", handleSave);
  Serial.println("HTTP server started");
  MDNS.addService("http", "tcp", 80);

  server.begin();
  delay(100); 

  motorInit();


  Serial.write("Buzzer\n"); 
  // NOTE_C, NOTE_Cs, NOTE_D, NOTE_Eb, NOTE_E, NOTE_F, NOTE_Fs, NOTE_G, NOTE_Gs, NOTE_A, NOTE_Bb, NOTE_B, NOTE_MAX
  ledcWriteNote(BUZZER_PWM, NOTE_A, 4);
  delay(500);
  ledcWriteNote(BUZZER_PWM, NOTE_A, 4);
  delay(500);
  ledcWriteNote(BUZZER_PWM, NOTE_A, 4);
  delay(500);
  ledcWriteNote(BUZZER_PWM, NOTE_F, 4);
  delay(350);
  ledcWriteNote(BUZZER_PWM, NOTE_Cs, 4);
  delay(150);
  ledcWriteNote(BUZZER_PWM, NOTE_A, 4);
  delay(500);
  ledcWriteNote(BUZZER_PWM, NOTE_F, 4);
  delay(350);
  ledcWriteNote(BUZZER_PWM, NOTE_Cs, 5);
  delay(150);
  ledcWriteNote(BUZZER_PWM, NOTE_A, 4);
  delay(650);  
  ledcWriteTone(BUZZER_PWM,0);

  Servo_PROGRAM_Zero();
  Servo_PROGRAM = 0;
}


void loop() {
  server.handleClient();
   if (Servo_PROGRAM >= 1 ) {
    delay(500);
    switch (Servo_PROGRAM) {
      case 1: // Action 1 
        Serial.println("Action1 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        break;
      case 2: // Action 2 
        Serial.println("Action2 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        break;   
      case 3: // Action 3   
        Serial.println("Action3 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step);
        break;    
      case 100: // Zero
        Serial.println("Zero");
        Servo_PROGRAM_Zero();
        break;
    }
    Servo_PROGRAM = 0;
  }
}

void handleIndex() {
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "<title>SMLab iRobotArm ™</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body {";
  content += "margin:0px;";
  content += "backgound-color:#FFFFFF;";
  content += "font-family:helvetica,arial;";
  content += "font-size:100%;";
  content += "color: #555555;";
  content += "text-align: center;";
  content += "}";
  content += "td {";
  content += "text-align: center;";
  content += "}";
  content += "span {";
  content += "font-family:helvetica,arial;";
  content += "font-size:70%;";
  content += "color:#777777;";
  content += "}";
  content += ".button{";
  content += "width:90%;";
  content += "height:90%;";
  content += "font-family:helvetica,arial;";
  content += "font-size:100%;";
  content += "color:#555555;";
  content += "background:#BFDFFF;";
  content += "border-radius:4px;";
  content += "padding: 2px 2px 2px 2px;";
  content += "border:none;}";
  content += ".button:active{";
  content += "background-color:#999;";
  content += "color:white;}";
  content += ".button2{background-color:#BFFFCF;}";
  content += ".button3{background-color:#FFBFBF;}"; 
  content += ".button4{background-color:#FFCC99;}";
  content += ".button5{background-color:#FFE599;}";
  content += ".button6{background-color:#CFBFFF;}";
  content += "</style>";
  content += "</head>";
  content += "<body><h1>SMLab iRobotArm ™</h1>";
  content += "<table width=100% height=30%>";
  content += "<tr>";
  content += "<td width=33%><button class=\"button button2\" onclick=controlPm(1)>Action1</button></td>";
  content += "<td width=33%><button class=\"button\" onclick=controlPm(2)>Action2</button></td>";
  content += "<td width=33%><button class=\"button button3\" onclick=controlPm(3)>Action3</button></td>";
  content += "</tr>";
  content += "</table>";
  content += "<table width=100% height=50%>";
  content += "<tr><td colspan=4><span><br></span></td></tr>";
  content += "<tr>";
  content += "<td width=33%><button class=\"button button4\" onclick=controlServo(0,'range_0',1)>Clockwise</button></td>";
  content += "<td width=33%>Arm <span><br>0 <input type=range id=range_0 min=0 max=180 value=90 onchange=controlServo(0,'range_0',0)> 180</span>";
  content += "<td width=33%><button class=\"button button5\"  onclick=controlServo(0,'range_0',2)>AntiCLockise</button></td>";
  content += "</tr>";
  content += "<tr><td colspan=4><span><br></span></td></tr>";
  content += "<tr>";
  content += "<td width=33%><button class=\"button button4\"  onclick=controlServo(1,'range_1',1)>Clockwise</button></td>";
  content += "<td width=33%>Shoulder <span><br>0 <input type=range id=range_1 min=0 max=180 value=90 onchange=controlServo(1,'range_1',0)> 180</span>";
    content += "<td width=33%><button class=\"button button5\"  onclick=controlServo(1,'range_1',2)>AntiCLockise</button></td>";
  content += "</tr>";
  content += "<tr><td colspan=4><span><br></span></td></tr>";
  content += "<tr>";
  content += "<td width=33%><button class=\"button button4\"  onclick=controlServo(2,'range_2',1)>Clockwise</button></td>";
  content += "<td width=33%>Base <span><br>0 <input type=range id=range_2 min=0 max=180 value=90 onchange=controlServo(2,'range_2',0)> 180</span>";
    content += "<td width=33%><button class=\"button button5\"  onclick=controlServo(2,'range_2',2)>AntiCLockise</button></td>";
  content += "</tr>";
  content += "<tr><td colspan=4><span><br></span></td></tr>";
  content += "<tr>";
  content += "<td width=33%><button class=\"button button4\" onclick=controlServo(3,'range_3',1)>Clockwise</button></td>";
  content += "<td width=33%>Gripper <span><br>0 <input type=range id=range_3 min=0 max=180 value=90 onchange=controlServo(3,'range_3',0)> 180</span>";
  content += "<td width=33%><button class=\"button button5\" onclick=controlServo(3,'range_3',2)>AntiCLockise</button></td>";
  content += "</tr>";
  content += "</table>";
  content += "</body>";
  content += "<script>";  
  content += "function controlServo(id, textId,bfAdd) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "var value = document.getElementById(textId).value;";
  content += "if(1 == bfAdd) value = parseInt(value)-parseInt(\"10\");";
  content += "if(2 == bfAdd) value = parseInt(value)+parseInt(\"10\");";
  content += "if(parseInt(value) > 180 ) value = 180; ";
  content += "if(parseInt(value) < 0  ) value = 0; ";
  content += "document.querySelector('#range_' + id).value = value;";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\",\"controller?servo=\"+id+\"&value=\"+value, true);";
  content += "xhttp.send();";
  content += "}";
  content += "function controlPm(id) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\", \"controller?pm=\"+id, true);";
  content += "xhttp.send();";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);

  String page = "<html><head><style>button {font-size: 20px;}</style></head><body>";
  page += "<h1>Robot Arm Control</h1>";
  page += "<p><button style='color: red;' onclick='rotate(\"base\", \"clockwise\")'>Base Rotate Clockwise</button>";
  page += "<button style='color: blue;' onclick='rotate(\"base\", \"anticlockwise\")'>Base Rotate Anticlockwise</button></p>";
  page += "<p><button style='color: red;' onclick='rotate(\"shoulder\", \"clockwise\")'>Shoulder Rotate Clockwise</button>";
  page += "<button style='color: blue;' onclick='rotate(\"shoulder\", \"anticlockwise\")'>Shoulder Rotate Anticlockwise</button></p>";
  page += "<p><button style='color: red;' onclick='rotate(\"arm\", \"clockwise\")'>Arm Rotate Clockwise</button>";
  page += "<button style='color: blue;' onclick='rotate(\"arm\", \"anticlockwise\")'>Arm Rotate Anticlockwise</button></p>";
  page += "<p><button style='color: red;' onclick='rotate(\"gripper\", \"clockwise\")'>Gripper Rotate Clockwise</button>";
  page += "<button style='color: blue;' onclick='rotate(\"gripper\", \"anticlockwise\")'>Gripper Rotate Anticlockwise</button></p>";
  page += "<script>function rotate(axis, direction) {var xhr = new XMLHttpRequest(); xhr.open('GET', '/rotate?axis=' + axis + '&direction=' + direction); xhr.send();}</script>";
  page += "</body></html>";

  server.send(200, "text/html", page);
}

void handleController()
{
  String pm = server.arg("pm");
  String servo = server.arg("servo");
  String value = server.arg("value");
  Serial.println("Controller pm: "+pm+" servo: "+servo +" value: "+value);
  if (pm != "") {
    Servo_PROGRAM = pm.toInt();
    server.send(200, "text/html", "(pm)=(" + pm + ")");
  }

  if (servo != "" && value!= "") {
    ConvertDegreeToPwmAndSetServo(servo.toInt(),value.toInt());
    server.send(200, "text/html", "servo =" + servo + " value =" + value);
  }
  server.send(200, "text/html", "Input invalid");
}


