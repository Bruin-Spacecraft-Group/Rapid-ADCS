int motDirection = 8;
int motSpeed = 5;
int detectPot = A5;

volatile int pwm_value = 0;
volatile int prev_time = 0;
volatile double rpm = 0;
volatile int count = 0;
volatile double totalRPM = 0;
int maxCount = 3;
volatile double timeSec = 0;
volatile double prevTime = 0;
bool highSpeed = 0;

void setup() {
  pinMode(motDirection, OUTPUT);
  pinMode(motSpeed, OUTPUT);
  pinMode(detectPot, INPUT);
  Serial.begin(115200);

  attachInterrupt(0, rising, RISING);
}

void rising() {
  attachInterrupt(0, falling, FALLING);
}
 
void falling() {
  attachInterrupt(0, rising, RISING);
  pwm_value = micros()-prev_time;
  rpm = 1.0 / (((double) pwm_value) / 1000000.0);
  rpm = (rpm / 6.0) * 60.0;
  totalRPM += rpm;
  timeSec += (((double) pwm_value) / 1000000.0);
  count++;
  if(count >= maxCount){
    rpm = totalRPM / (double) maxCount;
    Serial.print(timeSec);
    Serial.print(", ");
    Serial.print(rpm);
    Serial.println();
    count = 0;
    totalRPM = 0;
  }
  prev_time = micros();
}

int speed = 0;
float adjustment = (256.0 / 1024.0);
int adjustedSpeed = 100;
void loop() {
  speed = analogRead(detectPot);
  //adjustedSpeed = (int) (((float) speed) * adjustment);
  adjustedSpeed = adjustedSpeed;
  if(timeSec - prevTime > 1.0){
    if(highSpeed){
      adjustedSpeed = 200;
    }
    else{
      adjustedSpeed = 100;
    }
    highSpeed = !highSpeed;
    prevTime = timeSec;
  }
  analogWrite(motSpeed, adjustedSpeed);
  delay(10);
}
