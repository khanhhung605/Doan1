#include <Servo.h>

// ==============================================================================
// 1. KHAI BÁO CHÂN KHỚP VỚI SƠ ĐỒ PROTEUS (PHẦN CỦA TV1)
// ==============================================================================
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

// Chân còi báo vật cản động (Chân D4 còn trống trên UNO R3)
const int BUZZER_PIN = 4;

// ==============================================================================
// 2. CÁC HÀM CƠ SỞ DRIVER PHẦN CỨNG (PHẦN CỦA TV1 + CẦU NỐI PID)
// ==============================================================================
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

void readLine() {
  for (int i = 0; i < 5; i++) {
    lineVal[i] = digitalRead(LINE_PINS[i]);
  }
}

// Cầu nối chuyển đổi 5 mắt dò line thành sai số PID (-100 đến +100, mất line = 999)
int getLineError() {
  readLine();
  const int weights[5] = {-100, -50, 0, 50, 100};
  int sum = 0;
  int activeCount = 0;

  for (int i = 0; i < 5; i++) {
    if (lineVal[i] == 1) {
      sum += weights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) return 999; // Cả 5 mắt đều không thấy line
  return sum / activeCount;
}

int readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 400;
  return (int)(duration * 0.034 / 2);
}

void setBuzzer(bool state) {
  digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}

// ==============================================================================
// 3. MÁY TRẠNG THÁI (FSM) & THUẬT TOÁN DÒ LINE / NÉ VẬT CẢN (PHẦN CỦA TV2)
// ==============================================================================
enum RobotState {
  STATE_FOLLOW_LINE,      // 1. Dò line PID
  STATE_WAIT_DYNAMIC,     // 2. Gặp vật cản -> Dừng + còi + chờ 5s
  STATE_SCAN_STATIC,      // 3. Hết 5s vẫn còn vật cản -> Quét 0° và 180°
  STATE_EXIT_LINE,        // 4. Rẽ thoát line & xoay Servo ngang
  STATE_WALL_FOLLOW,      // 5. Bám cạnh vật cản tĩnh
  STATE_REACQUIRE_LINE    // 6. Vòng cung bắt lại line
};

class RobotFSM {
private:
  // Tham số PID dò line (tinh chỉnh thực tế trên sa bàn/Proteus)
  float Kp = 1.4f;
  float Ki = 0.0f;
  float Kd = 0.5f;
  float lastError = 0.0f;
  float integral = 0.0f;
  int baseSpeed = 160;

  // Ngưỡng khoảng cách & thời gian
  const int DIST_SAFE = 20;                 // Ngưỡng vật cản trước mặt (cm)
  const int DIST_WALL_TARGET = 15;          // Khoảng cách bám cạnh hộp (cm)
  const unsigned long WAIT_DYNAMIC_MS = 5000; // Chờ vật cản động 5 giây

  // Bộ đếm thời gian không chặn (Non-blocking millis)
  unsigned long lastPidMillis = 0;
  unsigned long lastScanMillis = 0;
  unsigned long stateStartMillis = 0;

  // Biến xử lý vật cản tĩnh
  int scanStep = 0;
  int distRight0 = 0;
  int distLeft180 = 0;
  int avoidDirection = 1; // 1: Né sang Trái (bám cạnh Phải), -1: Né sang Phải (bám cạnh Trái)
  int sideServoAngle = 0;

  void changeState(RobotState newState, unsigned long now, const char* msg) {
    currentState = newState;
    stateStartMillis = now;
    Serial.print("[");
    Serial.print(now);
    Serial.print(" ms] -> ");
    Serial.println(msg);
  }

public:
  RobotState currentState = STATE_FOLLOW_LINE;

  void update() {
    unsigned long now = millis();

    switch (currentState) {
      // ------------------------------------------------------------------
      // TRẠNG THÁI 1: DÒ LINE PID & QUÉT SIÊU ÂM PHÍA TRƯỚC (GÓC 90°)
      // ------------------------------------------------------------------
      case STATE_FOLLOW_LINE: {
        // Cập nhật PID mỗi 20ms
        if (now - lastPidMillis >= 20) {
          lastPidMillis = now;
          int error = getLineError();

          if (error != 999) {
            integral += error;
            float derivative = error - lastError;
            float correction = Kp * error + Ki * integral + Kd * derivative;
            lastError = error;

            int leftPWM = baseSpeed + (int)correction;
            int rightPWM = baseSpeed - (int)correction;
            setMotor(leftPWM, rightPWM);
          } else {
            // Mất line tạm thời -> giảm tốc giữ thẳng tìm lại vạch
            setMotor(baseSpeed * 0.5, baseSpeed * 0.5);
          }
        }

        // Đo khoảng cách trước mặt mỗi 100ms
        if (now - lastScanMillis >= 100) {
          lastScanMillis = now;
          int frontDist = readDistance();
          if (frontDist > 0 && frontDist < DIST_SAFE) {
            setMotor(0, 0);
            setBuzzer(true);
            changeState(STATE_WAIT_DYNAMIC, now, "VAT CAN DONG: Dung xe + Bat coi cho 5s");
          }
        }
        break;
      }

      // ------------------------------------------------------------------
      // TRẠNG THÁI 2: CHỜ VẬT CẢN ĐỘNG (5 GIÂY)
      // ------------------------------------------------------------------
      case STATE_WAIT_DYNAMIC: {
        setMotor(0, 0);

        // Kiểm tra mỗi 200ms xem vật cản đã rời đi chưa
        if (now - lastScanMillis >= 200) {
          lastScanMillis = now;
          int frontDist = readDistance();

          if (frontDist >= DIST_SAFE + 5) {
            setBuzzer(false);
            integral = 0;
            lastError = 0;
            changeState(STATE_FOLLOW_LINE, now, "Vat can da roi di -> Tiep tuc do line");
            return;
          }
        }

        // Hết 5000ms mà vật cản vẫn đứng yên -> Chuyển sang né vật cản tĩnh
        if (now - stateStartMillis >= WAIT_DYNAMIC_MS) {
          setBuzzer(false);
          scanStep = 0;
          myServo.write(0); // Bắt đầu quay Servo sang Phải (0 độ)
          changeState(STATE_SCAN_STATIC, now, "Het 5s -> Quay Servo 0 do quet ben Phai");
        }
        break;
      }

      // ------------------------------------------------------------------
      // TRẠNG THÁI 3: QUÉT VẬT CẢN TĨNH (GÓC 0° VÀ 180° BẰNG MILLIS)
      // ------------------------------------------------------------------
      case STATE_SCAN_STATIC: {
        setMotor(0, 0);

        // Chờ 400ms cho Servo ổn định tại góc 0° rồi đo bên Phải
        if (scanStep == 0 && (now - stateStartMillis >= 400)) {
          distRight0 = readDistance();
          myServo.write(180); // Quay tiếp Servo sang Trái (180 độ)
          scanStep = 1;
          stateStartMillis = now;
        }
        // Chờ 400ms cho Servo ổn định tại góc 180° rồi đo bên Trái
        else if (scanStep == 1 && (now - stateStartMillis >= 400)) {
          distLeft180 = readDistance();

          if (distLeft180 >= distRight0) {
            avoidDirection = 1;   // Bên Trái rộng hơn -> Rẽ Trái thoát line
            sideServoAngle = 0;   // Xoay Servo sang Phải (0°) để bám cạnh hộp
            changeState(STATE_EXIT_LINE, now, "Chon re TRAI ne vat can tinh");
          } else {
            avoidDirection = -1;  // Bên Phải rộng hơn -> Rẽ Phải thoát line
            sideServoAngle = 180; // Xoay Servo sang Trái (180°) để bám cạnh hộp
            changeState(STATE_EXIT_LINE, now, "Chon re PHAI ne vat can tinh");
          }
        }
        break;
      }

      // ------------------------------------------------------------------
      // TRẠNG THÁI 4: RẼ THOÁT LINE & XOAY SERVO NGANG
      // ------------------------------------------------------------------
      case STATE_EXIT_LINE: {
        if (now - stateStartMillis < 500) {
          // Xoay xe thoát khỏi vạch line trong 500ms
          if (avoidDirection == 1) setMotor(-130, 130);
          else setMotor(130, -130);
        } else if (now - stateStartMillis < 900) {
          // Dừng nhẹ 400ms để quay Servo sang ngang nhìn vào cạnh hộp
          setMotor(0, 0);
          myServo.write(sideServoAngle);
        } else {
          changeState(STATE_WALL_FOLLOW, now, "Bat dau bam canh vat can tinh");
        }
        break;
      }

      // ------------------------------------------------------------------
      // TRẠNG THÁI 5: BÁM CẠNH VẬT CẢN TĨNH
      // ------------------------------------------------------------------
      case STATE_WALL_FOLLOW: {
        if (now - lastScanMillis >= 80) {
          lastScanMillis = now;
          int sideDist = readDistance();

          // Nếu khoảng cách ngang > 35cm -> Đã vượt qua chiều dài vật cản
          if (sideDist > 35) {
            changeState(STATE_REACQUIRE_LINE, now, "Vuot qua hop -> Vong cung bat lai line");
            return;
          }

          // Điều chỉnh nhẹ 2 bánh giữ khoảng cách song song với cạnh hộp
          int wallError = sideDist - DIST_WALL_TARGET;
          int adjust = (int)(wallError * 3.0f) * avoidDirection;
          setMotor(140 + adjust, 140 - adjust);
        }
        break;
      }

      // ------------------------------------------------------------------
      // TRẠNG THÁI 6: CHẠY VÒNG CUNG ĐỂ BẮT LẠI LINE
      // ------------------------------------------------------------------
      case STATE_REACQUIRE_LINE: {
        if (avoidDirection == 1) {
          setMotor(160, 90);  // Vòng cung sang phải tìm line
        } else {
          setMotor(90, 160);  // Vòng cung sang trái tìm line
        }

        // Sau tối thiểu 400ms rời hộp, đọc cảm biến TCRT5000L để bắt lại vạch
        if (now - stateStartMillis >= 400) {
          if (getLineError() != 999) {
            myServo.write(90); // Đưa Servo về thẳng 90°
            integral = 0;
            lastError = 0;
            changeState(STATE_FOLLOW_LINE, now, "Da bat lai line -> Tiep tuc do line PID");
          }
        }
        break;
      }
    }
  }
};

RobotFSM robot;

// ==============================================================================
// 4. CHƯƠNG TRÌNH CHÍNH SETUP & LOOP
// ==============================================================================
void setup() {
  Serial.begin(9600);

  // Cấu hình chân động cơ
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Cấu hình chân siêu âm & còi
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  // Cấu hình 5 chân dò line
  for (int i = 0; i < 5; i++) {
    pinMode(LINE_PINS[i], INPUT);
  }

  // Khởi động Servo ở góc nhìn thẳng (90 độ)
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  setMotor(0, 0);
  delay(500);

  Serial.println("=== KHOI DONG HE THONG ROBOT DO LINE & NE VAT CAN ===");
}

void loop() {
  robot.update();
}