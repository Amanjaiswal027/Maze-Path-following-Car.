#define ENA 9  // Left Motor Speed
#define ENB 3  // Right Motor Speed
#define IN1 8  // Left Motor Direction
#define IN2 7
#define IN3 5  // Right Motor Direction
#define IN4 4

int sensorPins[5] = { 6, 12, 11, 10, 2 };  // IR Sensor Pins
int sensorValues[5];                       // Stores sensor readings
int position[] = { -2, -1, 0, 1, 2 };      // Sensor weight
float Kp = 8, Ki = 0, Kd = 6;             // PID Constants
float error = 0, previous_error = 0, integral = 0;
int baseSpeed = 55;
int startTime = 0;
int startFlag = 1;
int time = millis();

void setup() {
  // Serial.begin(9600);
  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  float sum = 0;
  int activeSensors = 0;
  int leftBiase = 0;

  // Serial.print("Active Sensors: ");

  // Read sensor values
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
    if (sensorValues[i] == 1) {  // Sensor detects black
      sum += position[i];
      activeSensors++;
      // Serial.print(i);
      // Serial.print(", ");
    }
  }

  // if(startFlag) {
  //   startFlag = 0;
  //   analogWrite(ENA, 255);
  //   analogWrite(ENB, 255);
  //   digitalWrite(IN1, HIGH);
  //   digitalWrite(IN2, LOW);
  //   digitalWrite(IN3, LOW);
  //   digitalWrite(IN4, HIGH);
  //   delay(2);
  // }

  // Calculate error
  if (activeSensors >= 5) {
    error = -previous_error;
    // if (!startFlag) {
    //   startFlag = 1;
    //   startTime = millis();
    //   Serial.println("Started");
    // }
  } else if (activeSensors > 0) error = sum / activeSensors;
  else {
    error = previous_error;  // If no sensor detects black, use previous error
    // if (startFlag) {
    //   int currentTime = millis();
    //   if (currentTime - startTime > 300) {
    //     analogWrite(ENA, 0);
    //     analogWrite(ENB, 0);
    //     digitalWrite(IN1, LOW);
    //     digitalWrite(IN2, LOW);
    //     digitalWrite(IN3, LOW);
    //     digitalWrite(IN4, LOW);
    //     while (1) delay(5000);
    //   } else {
    //     startFlag = 0;
    //   }
    // }
  }

  // Serial.print("Sum: ");
  // Serial.print(sum);
  // Serial.print(", Error: ");
  // Serial.print(error);

  // PID Calculation
  float P = error;
  integral += error;
  float D = error - previous_error;
  float correction = (Kp * P) + (Kd * D);
  if (Ki) {
    correction += integral / Ki;
  }
  previous_error = error;

  // if(correction)
  //   baseSpeed = TSPEED;
  // else
  //   baseSpeed = SPEED;

  // Adjust motor speeds
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Serial.print(", Correction: ");
  // Serial.println(correction);
  // Serial.print("Left: ");
  // Serial.print(leftSpeed);
  // Serial.print(", Right: ");
  // Serial.println(rightSpeed);

  if (activeSensors) {
    moveMotors(leftSpeed, rightSpeed, 0);
    startFlag = 0;
  }
  else {
      moveMotors(leftSpeed, rightSpeed, correction > 0 ? -1 : 1);
    
  }

  moveMotors(leftSpeed, rightSpeed, 0);
  delay(2);
}

void moveMotors(int left, int right, int flag) {
  int speed = 60;
  int activeSensors = 0;

  if (flag == 0) {
    analogWrite(ENA, left);
    analogWrite(ENB, right);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    if (flag == 1) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }

    int preMillis = millis();
    while (true) {
      for (int i = 0; i < 5; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
        if (sensorValues[i] == 1) {
          activeSensors++;
        }
      }
      if (activeSensors)
        break;

      int currMillis = millis();
      if (currMillis - preMillis > 1300) {
        analogWrite(ENA, 255);
        analogWrite(ENB, 255);
        delay(2);
      }
      speed = constrain(speed - 1, 40, 60);
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      delay(2);
    }
  }
}
