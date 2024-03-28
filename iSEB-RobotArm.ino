#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

/* To store the calibration value for each servo motor */
Preferences preferences;

const char* ssid = "RobotArmAP"; // SSID for the ESP32 access point
const char* password = "12345678"; // Password for the access point

IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// Servos matrix
const int ALLMATRIX = 5; // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2 + Run Time
const int ALLSERVOS = 4; // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2

// Servo delay base time
const int BASEDELAYTIME = 20; // 10 ms

// Backup servo value
int Running_Servo_POS [ALLMATRIX] = {};


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

#define ARM_PIN             23  /* PIN 23 */
#define SHOULDER_PIN        4   /* PIN 23 */
#define GRIPPER_PIN         32  /* PIN 23 */
#define BASE_PIN            12  /* PIN 23 */

void Set_PWM_to_Servo(int iServo, int iValue)
{
  Serial.print(F("iServo: "));
  Serial.print(iServo); 
  Serial.print(F(" iValue: "));
  Serial.println(iValue);
  // Read from EEPROM to fix zero error reading
  iValue = (iValue*MAX/180.0)+MIN; /* convertion to pwm value */
  double NewPWM = iValue + preferences.getDouble((String(iServo)).c_str(),0);

  /* 0 = zero degree 550 = 180 degree*/
  ledcWrite(iServo,NewPWM);
}

int currentPosition[4] = {90, 90, 90, 90}; // Initial positions (90 for servos, 0 for closed gripper)

// Servo zero position 
//Array 
int Servo_Act_0 [ ] PROGMEM = {  90,  90, 90,  90,  500  };


void Servo_PROGRAM_Zero()
{
  /* Update zero value to servo motor variable */
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

    /* Update the servo motor to zero position */
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    Set_PWM_to_Servo(iServo,Running_Servo_POS[iServo]);
    delay(50);
  }
}

void motorInit() {
  // Set base frequency and resolution for all channels
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);

  // Attach each servo motor pin to a channel
  ledcAttachPin(ARM_PIN       , ARM_CHANNEL);       /* ARM *//* CN8  *//* PIN 32*/
  ledcAttachPin(SHOULDER_PIN  , SHOULDER_CHANNEL);  /* SHOULDER *//* CN10 *//* PIN  4*/
  ledcAttachPin(GRIPPER_PIN   , GRIPPER_CHANNEL);   /* GRIPPER *//* CN2  *//* PIN 12*/
  ledcAttachPin(BASE_PIN      , BASE_CHANNEL);      /* BASE *//* CN16 *//* PIN 23*/
}


void setup() {

  Serial.begin(115200);
  Serial.write("Hello World\n");

  // Set ESP32 as an access point
  // WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet); // Configure access point settings

  // Setup MDNS responder
  if (!MDNS.begin("esp32")) {
    Serial.println("Error setting up MDNS responder!");
  }

  // Start the web server
  // server.on("/", handleIndex);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/rotate", HTTP_GET, handleRotate);
  server.begin();

  Serial.println("HTTP server started");

  motorInit();
  Servo_PROGRAM_Zero();
}


void loop() {
  server.handleClient();
}

// void handleIndex() {
//   double servo7Val = preferences.getDouble("7", 0);
//   String servo7ValStr = String(servo7Val);
//   double servo6Val = preferences.getDouble("6", 0);
//   String servo6ValStr = String(servo6Val);
//   double servo5Val = preferences.getDouble("5", 0);
//   String servo5ValStr = String(servo5Val);
//   double servo4Val = preferences.getDouble("4", 0);
//   String servo4ValStr = String(servo4Val);
//   double servo3Val = preferences.getDouble("3", 0);
//   String servo3ValStr = String(servo3Val);
//   double servo2Val = preferences.getDouble("2", 0);
//   String servo2ValStr = String(servo2Val);
//   double servo1Val = preferences.getDouble("1", 0);
//   String servo1ValStr = String(servo1Val);
//   double servo0Val = preferences.getDouble("0", 0);
//   String servo0ValStr = String(servo0Val);
//   String content = "";

//   content += "<html>";
//   content += "<head>";
//   content += "<title>Servo calibration</title>";
//   content += "<meta charset=UTF-8>";
//   content += "<meta name=viewport content=width=device-width>";
//   content += "<style type=text/css>";
//   content += "body {";
//   content += "margin: 0px;";
//   content += "backgound-color: #FFFFFF;";
//   content += "font-family: helvetica, arial;";
//   content += "font-size: 100%;";
//   content += "color: #555555;";
//   content += "}";
//   content += "td {";
//   content += "text-align: center;";
//   content += "}";
//   content += "span {";
//   content += "font-family: helvetica, arial;";
//   content += "font-size: 70%;";
//   content += "color: #777777;";
//   content += "}";
//   content += "input[type=text] {";
//   content += "width: 40%;";
//   content += "font-family: helvetica, arial;";
//   content += "font-size: 90%;";
//   content += "color: #555555;";
//   content += "text-align: center;";
//   content += "padding: 3px 3px 3px 3px;";
//   content += "}";
//   content += "button {";
//   content += "width: 40%;";
//   content += "font-family: helvetica, arial;";
//   content += "font-size: 90%;";
//   content += "color: #555555;";
//   content += "background: #BFDFFF;";
//   content += "padding: 5px 5px 5px 5px;";
//   content += "border: none;";
//   content += "}";
//   content += "</style>";
//   content += "</head>";
//   content += "<body>";
//   content += "<br>";
//   content += "<table width=100% height=90%>";
//   content += "<tr>";
//   content += "<td width=50%>WALKING1<br/><input type=text id=servo_0 value=\"" + servo0ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(0,'servo_0')>SET</button></td>";
//   content += "<td width=50%>MERUS1<br/><input type=text id=servo_4 value=\"" + servo4ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(4,'servo_4')>SET</button></td>";
//   content += "</tr>";
//   content += "<tr>";
//   content += "<td>WALKING2<br/><input type=text id=servo_1 value=\"" + servo1ValStr + "\"><button type=button onclick=saveServo(1,'servo_1')>SET</button></td>";
//   content += "<td>MERUS2<br/><input type=text id=servo_5 value=\"" + servo5ValStr + "\"><button type=button onclick=saveServo(5,'servo_15')>SET</button></td>";
//   content += "</tr>";
//   content += "<tr>";
//   content += "<td>WALKING3<br/><input type=text id=servo_2 value=\"" + servo2ValStr + "\"><button type=button onclick=saveServo(2,'servo_2')>SET</button></td>";
//   content += "<td>MERUS3<br/><input type=text id=servo_6 value=\"" + servo6ValStr + "\"><button type=button onclick=saveServo(6,'servo_6')>SET</button></td>";
//   content += "</tr>";
//   content += "<tr>";
//   content += "<td>WALKING4<br/><input type=text id=servo_3 value=\"" + servo3ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(3,'servo_3')>SET</button></td>";
//   content += "<td>MERUS4<br/><input type=text id=servo_7 value=\"" + servo7ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(7,'servo_7')>SET</button></td>";
//   content += "</tr>";
//   content += "<tr>";
//   content += "<td colspan=2><button type=button style=background:#FFBFBF onclick=saveServo(100,0)>RESET ALL</button></td>";
//   content += "</tr>";
//   content += "</table>";
//   content += "</body>";
//   content += "<script>";
//   content += "function saveServo(id, textId) {";
//   content += "var xhttp = new XMLHttpRequest();";
//   content += "var value = \"0\";";
//   content += "if(id==100){";
//   content += "document.getElementById(\"servo_3\").value = \"0\";";
//   content += "document.getElementById(\"servo_2\").value = \"0\";";
//   content += "document.getElementById(\"servo_1\").value = \"0\";";
//   content += "document.getElementById(\"servo_0\").value = \"0\";";
//   content += "}else{";
//   content += "value = document.getElementById(textId).value;";
//   content += "}";
//   content += "xhttp.onreadystatechange = function() {";
//   content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
//   content += "document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
//   content += "}";
//   content += "};";
//   content += "xhttp.open(\"GET\",\"save?key=\"+id+\"&value=\"+value, true);";
//   content += "xhttp.send();";
//   content += "}";
//   content += "</script>";
//   content += "</html>";

//   server.send(200, "text/html", content);
// }

void handleRoot() {
  
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

void handleRotate() {
  
  String axis = server.arg("axis");
  String direction = server.arg("direction");
  Serial.print("handle rotate axis: ");
  Serial.print(axis);
  Serial.print(" | direction: ");
  Serial.println(direction);

  int index;
  if (axis == "arm") {
    index = 0;
  } else if (axis == "shoulder") {
    index = 1;
  } else if (axis == "gripper") {
    index = 2;
  } else if (axis == "base") {
    index = 3;
  } else {
    server.send(400, "text/plain", "Invalid axis");
    return;
  }

  if (direction == "clockwise") {
    currentPosition[index] += 10;
  } else if (direction == "anticlockwise") {
    currentPosition[index] -= 10;
  }

  if(MIN > currentPosition[index])
  {
    currentPosition[index] = MIN;
  }
  else if(MAX < currentPosition[index])
  {
    currentPosition[index] = MAX;
  }
  Set_PWM_to_Servo(index,currentPosition[index]); 

  server.send(200, "text/plain", "Rotated");
}




