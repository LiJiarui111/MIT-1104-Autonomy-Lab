// L298N Motor A (left)
#define ENA 27
#define IN1 26
#define IN2 25

// L298N Motor B (right)
#define ENB 14
#define IN3 17
#define IN4 16

void setup() {
  // Configure all motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Start with motors turned off
  stopMotors();
}

void loop() {
  // 1. Move Forward for 2 seconds
  moveForward();
  delay(2000);

  // 2. Stop for 1 second
  stopMotors();
  delay(1000);

  // 3. Move Backward for 2 seconds
  moveBackward();
  delay(2000);

  // 4. Stop for 2 seconds before repeating
  stopMotors();
  delay(2000);
}

// --- Motor Control Functions ---

void moveForward() {
  // Turn on Left Motor (Forward)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, HIGH); // Full speed

  // Turn on Right Motor (Forward)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, HIGH); // Full speed
}

void moveBackward() {
  // Turn on Left Motor (Backward)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(ENA, HIGH);

  // Turn on Right Motor (Backward)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(ENB, HIGH);
}

void stopMotors() {
  // Cut power to both motors
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  
  // Reset direction pins (optional, but good practice)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}