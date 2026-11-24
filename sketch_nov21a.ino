#include <Servo.h>

// --- 1. KHAI BÁO CHÂN KHỚP VỚI SƠ ĐỒ PROTEUS ---
// Động cơ bên TRÁI (Kênh A - OUT1, OUT2)
const int ENA = 5;   // PWM tốc độ Trái
const int IN1 = 7;
const int IN2 = 8;

// Động cơ bên PHẢI (Kênh B - OUT3, OUT4)
const int ENB = 6;   // PWM tốc độ Phải
const int IN3 = 12;
const int IN4 = 13;

// Servo 180 độ
const int SERVO_PIN = 9;
Servo myServo;

// Cảm biến siêu âm HC-SR04
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;

// Thanh dò line 5 mắt (5 nút LOGICSTATE)
const int LINE_PINS[5] = {A0, A1, A2, A3, A4};
int lineVal[5]; // Mảng lưu trạng thái 0/1 của 5 mắt

// --- 2. CÁC HÀM CƠ SỞ (DRIVER PHẦN CỨNG - TV1) ---

// Hàm điều khiển tốc độ 2 bên bánh (-255 đến 255)
// Số dương: quay tiến | Số âm: quay lùi | 0: dừng
void setMotor(int leftSpeed, int rightSpeed) {
  // Xử lý 2 bánh TRÁI
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // Xử lý 2 bánh PHẢI
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-rightSpeed, 0, 255));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// Hàm đọc 5 mắt dò line (lưu vào mảng lineVal)
void readLine() {
  for (int i = 0; i < 5; i++) {
    lineVal[i] = digitalRead(LINE_PINS[i]);
  }
}

// Hàm đo khoảng cách hiện tại của HC-SR04 (đơn vị: cm)
int readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Giới hạn thời gian chờ 30ms để tránh treo mạch nếu không có sóng phản hồi
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 400; // Quá tầm đo -> coi như đường thoáng
  return (int)(duration * 0.034 / 2);
}

// Hàm xoay Servo tới góc bất kỳ rồi đo khoảng cách
int scanDistance(int angle) {
  myServo.write(angle);
  delay(400); // Chờ Servo quay xong rồi mới đo
  return readDistance();
}

// --- 3. CHƯƠNG TRÌNH CHÍNH (TEST PHẦN CỨNG TRÊN PROTEUS) ---

void setup() {
  Serial.begin(9600);

  // Cấu hình chân động cơ
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Cấu hình chân siêu âm
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Cấu hình 5 chân dò line
  for (int i = 0; i < 5; i++) {
    pinMode(LINE_PINS[i], INPUT);
  }

  // Khởi động Servo ở góc nhìn thẳng (90 độ)
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  setMotor(0, 0);
  delay(500);
}

void loop() {
  int dist = readDistance();
  readLine();

  // TEST 1: Vặn biến trở RV1 xuống thấp (< 20cm) -> Giả lập gặp vật cản
  if (dist < 20) {
    setMotor(0, 0);           // Dừng xe
    scanDistance(0);          // Quay Servo nhìn Phải
    scanDistance(180);        // Quay Servo nhìn Trái
    myServo.write(90);        // Quay lại nhìn Thẳng
    delay(400);
  }
  // TEST 2: Đường thoáng (>= 20cm) -> Bấm 5 nút LOGICSTATE để test động cơ
  else {
    if (lineVal[2] == 1) {
      setMotor(255, 255);     // Mắt giữa (A2) = 1 -> Đi thẳng
    } else if (lineVal[0] == 1 || lineVal[1] == 1) {
      setMotor(0, 255);      // Mắt trái (A0/A1) = 1 -> Rẽ trái (bánh phải nhanh hơn)
    } else if (lineVal[3] == 1 || lineVal[4] == 1) {
      setMotor(255, 0);      // Mắt phải (A3/A4) = 1 -> Rẽ phải (bánh trái nhanh hơn)
    } else {
      setMotor(0, 0);         // Cả 5 mắt = 0 -> Dừng xe
    }
  }

  delay(50);
}