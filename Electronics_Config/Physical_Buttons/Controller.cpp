#define EButton 2 // Emergency button pin
#define PButton 4 // Another button pin
#define RedButton 7
#define BlueButton 8
//1 is manual, 0 is automatic
char receivedChars[12];
bool newData = false;
int ebit;
bool ambit = 0;
bool prev_p = 1;
bool curr_p;
int e_stop_curr, e_stop_prev, op_mode_prev, op_mode_curr;
unsigned long previousMillis = 0;
const long blinkInterval = 1000;

void setup() {
  Serial.begin(115200);
  // Configure button pins with pull-up resistors
  pinMode(EButton, INPUT_PULLUP);
  pinMode(PButton, INPUT_PULLUP);
  pinMode(RedButton, OUTPUT);
  pinMode(BlueButton, OUTPUT);
}

// void loop() {
//   recvWithStartEndMarkers();
//   if (newData == true)
//   {
//     parseData();
//     newData = false;
//   }
//   //ambit = 0;
//   curr_p = digitalRead(PButton);
//   ebit = digitalRead(EButton);
//   if ((prev_p == 1) && (curr_p == 0)){
//     ambit = !ambit;
//   }
//   prev_p = curr_p;
//   if (op_mode_prev!=op_mode_curr){
//     ambit = op_mode_curr;
//   }
//   op_mode_prev = op_mode_curr;
//   // Check if the PButton is pressed for manual override
//   // Handle emergency flag set by the interrupt
//   Serial.print("<");
//   Serial.print(ebit);
//   Serial.print(",");
//   Serial.print(ambit);
//   Serial.print(">\n");
//   delay(100); // Small delay for loop stability
//   lights();
// }
void loop() {
  // Handle serial input
  recvWithStartEndMarkers();
  if (newData) {
    parseData(); // Parse new data
    newData = false;
  }

  // Read button states
  curr_p = digitalRead(PButton);
  ebit = digitalRead(EButton);

  // Toggle `ambit` when PButton is pressed
  if ((prev_p == 1) && (curr_p == 0)) {
    ambit = !ambit;
  }
  prev_p = curr_p;

  // Update `ambit` when `op_mode_curr` changes
  if (op_mode_prev != op_mode_curr) {
    ambit = op_mode_curr;
    Serial.print("op_mode changed! New ambit: ");
    Serial.println(ambit);
  }
  op_mode_prev = op_mode_curr;

  // Debug output
  Serial.print("<");
  Serial.print(ebit);
  Serial.print(",");
  Serial.print(ambit);
  Serial.println(">");

  // Update lights
  lights();
}
// void lights() {
//   if(e_stop_curr) {
//     digitalWrite(RedButton, HIGH);
//     digitalWrite(BlueButton, LOW);
//   }
//   else {
//     digitalWrite(RedButton, LOW);
//     if (op_mode_curr) {
//       digitalWrite(BlueButton, LOW);
//       delay(200);
//       digitalWrite(BlueButton, HIGH);
//       delay(200);
//     }
//     else {
//       digitalWrite(BlueButton, HIGH);
//       delay(100);
//     }
//   }
// }
void lights() {
  if (e_stop_curr) {
    digitalWrite(RedButton, HIGH);
    digitalWrite(BlueButton, LOW);
  } 
  else {
    digitalWrite(RedButton, LOW);

    unsigned long currentMillis = millis();
    if (op_mode_curr) { // Manual mode (blinking blue light)
      if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;
        digitalWrite(BlueButton, !digitalRead(BlueButton)); // Toggle BlueButton
      }
    } else { // Automatic mode (solid blue light)
      digitalWrite(BlueButton, HIGH);
    }
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void parseData()
// split the data into its parts
{
  char *strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(receivedChars, ","); // get the first part - left speed
  e_stop_curr = (bool) atoi(strtokIndx); //front-back speed
  strtokIndx = strtok(NULL, ","); // get the second part - right speed
  op_mode_curr = (bool) atoi(strtokIndx);
}
