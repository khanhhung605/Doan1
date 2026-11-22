#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std;

// ==============================================================================
// 1. TƯƠNG THÍCH MÔI TRƯỜNG (CHẠY TEST TRÊN PC HOẶC NẠP THẲNG XUỐNG UNO R3)
// ==============================================================================
#ifndef ARDUINO
auto programStart = chrono::steady_clock::now();

unsigned long millis() {
    auto now = chrono::steady_clock::now();
    return chrono::duration_cast<chrono::milliseconds>(now - programStart).count();
}
#endif

// ==============================================================================
// 2. KHAI BÁO NGUYÊN MẪU HÀM CƠ SỞ CỦA THÀNH VIÊN 1 (DRIVER UNO R3)
// ==============================================================================
void setMotor(int leftSpeed, int rightSpeed); // PWM: -255 đến 255
int readLine();                               // Sai số line: -100..100 (999 = mất line)
float scanDistance(int angle);                // Góc Servo: 0 (Phải), 90 (Thẳng), 180 (Trái)
void setBuzzer(bool state);                   // Còi báo khi gặp vật cản động

// ==============================================================================
// 3. MÁY TRẠNG THÁI (FSM) & BỘ ĐIỀU KHIỂN CỦA THÀNH VIÊN 2
// ==============================================================================
enum RobotState {
    STATE_FOLLOW_LINE,      // Dò line bằng PID
    STATE_WAIT_DYNAMIC,     // Gặp vật cản -> Dừng + còi + chờ 5s
    STATE_SCAN_STATIC,      // Hết 5s vẫn còn vật cản -> Quét Servo 0° và 180°
    STATE_EXIT_LINE,        // Rẽ thoát khỏi line về phía thoáng
    STATE_WALL_FOLLOW,      // Xoay Servo ngang, bám cạnh hộp
    STATE_REACQUIRE_LINE    // Chạy vòng cung bắt lại line
};

class RobotFSM {
private:
    // Tham số PID dò line (sẽ tinh chỉnh thực tế ở Tuần 2)
    float Kp = 0.8f, Ki = 0.0f, Kd = 0.3f;
    float lastError = 0.0f, integral = 0.0f;
    int baseSpeed = 140;

    // Ngưỡng khoảng cách & thời gian
    const float DIST_SAFE = 20.0f;          // Ngưỡng phát hiện vật cản trước mặt (cm)
    const float DIST_WALL_TARGET = 15.0f;   // Khoảng cách bám cạnh hộp (cm)
    const unsigned long WAIT_DYNAMIC_MS = 5000; // Chờ vật cản động 5 giây

    // Biến quản lý thời gian không chặn (Non-blocking millis)
    unsigned long lastPidMillis = 0;
    unsigned long lastFrontScanMillis = 0;
    unsigned long stateStartMillis = 0;

    // Biến phục vụ bài toán né vật cản tĩnh
    int scanStep = 0;
    float distRight0 = 0.0f;
    float distLeft180 = 0.0f;
    int avoidDirection = 1; // 1: Né sang Trái (bám cạnh Phải), -1: Né sang Phải (bám cạnh Trái)
    int sideServoAngle = 0;

    void changeState(RobotState newState, unsigned long now, const string& reason) {
        currentState = newState;
        stateStartMillis = now;
        cout << "[" << setw(5) << now << " ms] CHUYEN TRANG THAI -> " 
             << getStateName() << " (" << reason << ")\n";
    }

public:
    RobotState currentState = STATE_FOLLOW_LINE;

    string getStateName() const {
        switch (currentState) {
            case STATE_FOLLOW_LINE:    return "DO_LINE_PID";
            case STATE_WAIT_DYNAMIC:   return "CHO_VAT_CAN_DONG_5S";
            case STATE_SCAN_STATIC:    return "QUET_SERVO_0_180";
            case STATE_EXIT_LINE:      return "RE_THOAT_LINE";
            case STATE_WALL_FOLLOW:    return "BAM_CANH_HOP";
            case STATE_REACQUIRE_LINE: return "VONG_CUNG_BAT_LINE";
            default:                   return "UNKNOWN";
        }
    }

    // Hàm cập nhật chính - gọi liên tục trong loop() không dùng delay()
    void update() {
        unsigned long now = millis();

        switch (currentState) {
            // ------------------------------------------------------------------
            // TRẠNG THÁI 1: DÒ LINE PID & QUÉT VẬT CẢN PHÍA TRƯỚC
            // ------------------------------------------------------------------
            case STATE_FOLLOW_LINE: {
                // Chu kỳ tính PID mỗi 20ms
                if (now - lastPidMillis >= 20) {
                    lastPidMillis = now;
                    int error = readLine();
                    if (error != 999) {
                        integral += error;
                        float derivative = error - lastError;
                        float correction = Kp * error + Ki * integral + Kd * derivative;
                        lastError = error;

                        int leftPWM = baseSpeed + (int)correction;
                        int rightPWM = baseSpeed - (int)correction;
                        setMotor(leftPWM, rightPWM);
                    } else {
                        setMotor(baseSpeed * 0.6, baseSpeed * 0.6); // Mất line tạm thời -> đi chậm giữ hướng
                    }
                }

                // Chu kỳ kiểm tra siêu âm góc 90° mỗi 100ms
                if (now - lastFrontScanMillis >= 100) {
                    lastFrontScanMillis = now;
                    float frontDist = scanDistance(90);
                    if (frontDist > 0 && frontDist < DIST_SAFE) {
                        setMotor(0, 0);
                        setBuzzer(true);
                        changeState(STATE_WAIT_DYNAMIC, now, "Phat hien vat can cach " + to_string((int)frontDist) + "cm");
                    }
                }
                break;
            }

            // ------------------------------------------------------------------
            // TRẠNG THÁI 2: BÀI TOÁN 1 - VẬT CẢN ĐỘNG (PHANH + CÒI CHỜ 5 GIÂY)
            // ------------------------------------------------------------------
            case STATE_WAIT_DYNAMIC: {
                setMotor(0, 0);

                // Kiểm tra định kỳ mỗi 200ms xem vật cản đã đi khỏi chưa
                if (now - lastFrontScanMillis >= 200) {
                    lastFrontScanMillis = now;
                    float frontDist = scanDistance(90);

                    if (frontDist >= DIST_SAFE + 5.0f) {
                        setBuzzer(false);
                        changeState(STATE_FOLLOW_LINE, now, "Vat can dong da roi di -> Chay tiep");
                        return;
                    }
                }

                // Nếu hết 5000ms mà vật cản vẫn đứng yên -> Chuyển sang né vật cản tĩnh
                if (now - stateStartMillis >= WAIT_DYNAMIC_MS) {
                    setBuzzer(false);
                    scanStep = 0;
                    changeState(STATE_SCAN_STATIC, now, "Het 5s vat can van dung yen -> Chuyen ne tinh");
                }
                break;
            }

            // ------------------------------------------------------------------
            // TRẠNG THÁI 3: BÀI TOÁN 2 - QUÉT GÓC 0° VÀ 180° BẰNG MILLIS()
            // ------------------------------------------------------------------
            case STATE_SCAN_STATIC: {
                setMotor(0, 0);

                // Bước 0: Quay Servo sang 0° (Phải) và chờ 300ms ổn định cơ khí
                if (scanStep == 0 && (now - stateStartMillis >= 300)) {
                    distRight0 = scanDistance(0);
                    scanStep = 1;
                    stateStartMillis = now;
                }
                // Bước 1: Quay Servo sang 180° (Trái) và chờ 300ms ổn định cơ khí
                else if (scanStep == 1 && (now - stateStartMillis >= 300)) {
                    distLeft180 = scanDistance(180);

                    // Chọn hướng thoáng hơn để né
                    if (distLeft180 >= distRight0) {
                        avoidDirection = 1;   // Rẽ Trái thoát line
                        sideServoAngle = 0;   // Xoay Servo sang Phải (0°) để nhìn vào cạnh hộp
                    } else {
                        avoidDirection = -1;  // Rẽ Phải thoát line
                        sideServoAngle = 180; // Xoay Servo sang Trái (180°) để nhìn vào cạnh hộp
                    }
                    changeState(STATE_EXIT_LINE, now, 
                        "Quet 0d=" + to_string((int)distRight0) + "cm, 180d=" + to_string((int)distLeft180) + "cm");
                }
                break;
            }

            // ------------------------------------------------------------------
            // TRẠNG THÁI 4: RẼ THOÁT KHỎI LINE & XOAY SERVO NGANG
            // ------------------------------------------------------------------
            case STATE_EXIT_LINE: {
                if (avoidDirection == 1) {
                    setMotor(-110, 110); // Xoay trái thoát line
                } else {
                    setMotor(110, -110); // Xoay phải thoát line
                }

                // Quay đủ góc thoát line trong 500ms
                if (now - stateStartMillis >= 500) {
                    scanDistance(sideServoAngle); // Đưa Servo về góc ngang bám cạnh
                    changeState(STATE_WALL_FOLLOW, now, "Da thoat line -> Bat dau bam canh hop");
                }
                break;
            }

            // ------------------------------------------------------------------
            // TRẠNG THÁI 5: BÁM CẠNH VẬT CẢN TĨNH
            // ------------------------------------------------------------------
            case STATE_WALL_FOLLOW: {
                if (now - lastFrontScanMillis >= 80) {
                    lastFrontScanMillis = now;
                    float sideDist = scanDistance(sideServoAngle);

                    // Nếu cảm biến ngang báo khoảng cách lớn (> 35cm) -> Đã đi hết chiều dài hộp
                    if (sideDist > 35.0f) {
                        changeState(STATE_REACQUIRE_LINE, now, "Da vuot qua canh hop -> Vong cung bat lai line");
                        return;
                    }

                    // Điều chỉnh nhẹ 2 bánh để giữ khoảng cách song song với cạnh hộp
                    float wallError = sideDist - DIST_WALL_TARGET;
                    int adjust = (int)(wallError * 2.5f) * avoidDirection;
                    setMotor(120 + adjust, 120 - adjust);
                }
                break;
            }

            // ------------------------------------------------------------------
            // TRẠNG THÁI 6: CHẠY VÒNG CUNG ĐỂ BẮT LẠI LINE
            // ------------------------------------------------------------------
            case STATE_REACQUIRE_LINE: {
                // Đánh vòng cung ngược lại hướng đã rẽ để cắt ngang lại đường line
                if (avoidDirection == 1) {
                    setMotor(135, 75); // Vòng cung sang phải tìm line
                } else {
                    setMotor(75, 135); // Vòng cung sang trái tìm line
                }

                // Sau tối thiểu 400ms rời hộp, bắt đầu kiểm tra cảm biến TCRT5000L
                if (now - stateStartMillis >= 400) {
                    int lineVal = readLine();
                    if (lineVal != 999) {
                        scanDistance(90); // Đưa Servo về thẳng 90°
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

// ==============================================================================
// 4. MÔI TRƯỜNG GIẢ LẬP DỮ LIỆU CẢM BIẾN (MOCK DATA ĐỂ TEST TRÊN PC)
// ==============================================================================
#ifndef ARDUINO
int mockLineError = 0;
float mockDist0 = 80.0f;
float mockDist90 = 100.0f;
float mockDist180 = 80.0f;

void setMotor(int leftSpeed, int rightSpeed) {
    // Hàm giả lập nhận lệnh điều khiển từ FSM (bỏ comment dòng dưới nếu muốn in từng nhịp PWM)
    // cout << "   [Motor] L=" << leftSpeed << ", R=" << rightSpeed << "\n";
}

int readLine() {
    return mockLineError;
}

float scanDistance(int angle) {
    if (angle == 0)   return mockDist0;
    if (angle == 90)  return mockDist90;
    if (angle == 180) return mockDist180;
    return 100.0f;
}

void setBuzzer(bool state) {
    cout << "   [Buzzer] " << (state ? "BAT COI CANH BAO" : "TAT COI") << "\n";
}

int main() {
    RobotFSM robot;
    cout << "=== BAT DAU MO PHONG FSM ROBOT DO LINE & NE VAT CAN (DUNG MILLIS) ===\n\n";

    while (true) {
        unsigned long t = millis();

        // KỊCH BẢN 1 (0s -> 1.0s): Đường thoáng, xe dò line bình thường
        if (t < 1000) {
            mockLineError = 5;
            mockDist90 = 100.0f;
        }
        // KỊCH BẢN 2 (1.0s -> 3.0s): Vật cản động xuất hiện cách 12cm trong 2 giây rồi rời đi
        else if (t >= 1000 && t < 3000) {
            mockDist90 = 12.0f;
        }
        else if (t >= 3000 && t < 4000) {
            mockDist90 = 100.0f; // Vật cản động đi khỏi -> Xe tự chạy tiếp
        }
        // KỊCH BẢN 3 (4.0s -> 11.5s): Vật cản tĩnh chắn ngang > 5 giây -> Quét góc & Bám cạnh
        else if (t >= 4000 && t < 9800) {
            mockDist90 = 14.0f;  // Chắn trước mặt suốt hơn 5 giây
            mockDist0 = 25.0f;   // Bên phải hẹp (25cm)
            mockDist180 = 90.0f; // Bên trái rộng (90cm) -> Hệ thống sẽ chọn rẽ Trái
        }
        // Giả lập lúc xe đang bám cạnh hộp (9.8s -> 11.2s)
        else if (t >= 9800 && t < 11200) {
            mockDist90 = 100.0f;
            mockDist0 = 15.0f;   // Servo xoay sang 0° đo thấy cạnh hộp cách 15cm
            mockLineError = 999; // Đang rời khỏi line
        }
        // Giả lập lúc vượt qua chiều dài hộp & vòng cung bắt lại line (sau 11.2s)
        else if (t >= 11200 && t < 12500) {
            mockDist0 = 80.0f;   // Đã hết cạnh hộp (> 35cm) -> Chuyển sang vòng cung
            if (t >= 11800) {
                mockLineError = 0; // Bắt lại được vạch line đen
            }
        }
        else {
            cout << "\n=== HOAN THANH TOAN BO KICH BAN MO PHONG ===\n";
            break;
        }

        robot.update();
        this_thread::sleep_for(chrono::milliseconds(20)); // Chu kỳ mô phỏng 20ms
    }

    return 0;
}
#endif