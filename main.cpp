#include <iostream>
#include <iomanip>
#include <string>

using namespace std;

// ==============================================================================
// 1. ĐỊNH NGHĨA KIỂU DỮ LIỆU & TRẠNG THÁI (INTERFACE CHUNG)
// ==============================================================================
enum RobotState {
    FOLLOW_AI,          // Đi theo lệnh Camera AI (Tuần 4)
    AVOID_LEFT,         // Đánh lái trái né vật
    AVOID_RIGHT,        // Đánh lái phải né vật
    SIDE_ADJUST,        // Chỉnh vô lăng đi giữa luồng
    EMERGENCY_REVERSE,  // Quá sát vật cản -> Lùi
    TURN_AROUND         // Ngõ cụt -> Quay đầu
};

enum VisionCommand {
    CMD_FORWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_NONE
};

struct SensorData {
    float left_cm;
    float front_cm;
    float right_cm;
};

// ==============================================================================
// 2. CLASS ĐIỀU PHỐI VÀ NÉ VẬT CẢN (MODULE CỦA THÀNH VIÊN 2)
// ==============================================================================
class Navigator {
private:
    // Các ngưỡng khoảng cách (cm)
    const float D_EMERGENCY = 15.0;
    const float D_SAFE = 40.0;
    const float D_SIDE = 20.0;

    // Các mức vận tốc (v: m/s, omega: rad/s)
    const float V_NORMAL = 0.40;
    const float V_TURN = 0.22;
    const float V_REVERSE = -0.20;
    const float OMEGA_AVOID = 1.20;
    const float OMEGA_AI = 0.60;

public:
    RobotState currentState = FOLLOW_AI;

    void computeControl(SensorData sensors, VisionCommand aiCmd, float &v_out, float &omega_out) {
        // ƯU TIÊN 1: Lùi khẩn cấp nếu vật cản quá gần trước mặt
        if (sensors.front_cm < D_EMERGENCY) {
            currentState = EMERGENCY_REVERSE;
            v_out = V_REVERSE; 
            omega_out = 0.0;
            return;
        }

        // ƯU TIÊN 2: Bắt đầu né vật cản phía trước trong vùng an toàn
        if (sensors.front_cm < D_SAFE) {
            // Kiểm tra ngõ cụt (bị chặn cả hai bên)
            if (sensors.left_cm < D_SAFE && sensors.right_cm < D_SAFE) {
                currentState = TURN_AROUND;
                v_out = 0.0;
                omega_out = (sensors.left_cm >= sensors.right_cm) ? OMEGA_AVOID : -OMEGA_AVOID;
                return;
            }
            // Chọn hướng thoáng hơn để né
            if (sensors.left_cm > sensors.right_cm) {
                currentState = AVOID_LEFT;
                v_out = V_TURN; 
                omega_out = OMEGA_AVOID;
            } else {
                currentState = AVOID_RIGHT;
                v_out = V_TURN; 
                omega_out = -OMEGA_AVOID;
            }
            return;
        }

        // ƯU TIÊN 3: Cân bằng hai bên hông (tránh quệt tường)
        if (sensors.left_cm < D_SIDE && sensors.right_cm >= D_SIDE) {
            currentState = SIDE_ADJUST;
            v_out = V_NORMAL * 0.8; 
            omega_out = -0.40; // Lái nhẹ sang phải
            return;
        }
        if (sensors.right_cm < D_SIDE && sensors.left_cm >= D_SIDE) {
            currentState = SIDE_ADJUST;
            v_out = V_NORMAL * 0.8; 
            omega_out = 0.40;  // Lái nhẹ sang trái
            return;
        }

        // ƯU TIÊN 4: Đường thoáng -> Đi theo lệnh của Camera AI (Thành viên 3)
        currentState = FOLLOW_AI;
        if (aiCmd == CMD_LEFT) {
            v_out = V_NORMAL * 0.85; 
            omega_out = OMEGA_AI;
        } else if (aiCmd == CMD_RIGHT) {
            v_out = V_NORMAL * 0.85; 
            omega_out = -OMEGA_AI;
        } else {
            v_out = V_NORMAL; 
            omega_out = 0.0;
        }
    }
};

// ==============================================================================
// 3. HÀM GIẢ LẬP ĐỘNG CƠ VÀ CHẠY TEST (MAIN)
// ==============================================================================
void mockMotorControl(float v, float omega, RobotState state) {
    string stateStr;
    switch(state) {
        case FOLLOW_AI:         stateStr = "BAM_AI_DI_THANG"; break;
        case AVOID_LEFT:        stateStr = "RE_TRAI_NE_VAT";  break;
        case AVOID_RIGHT:       stateStr = "RE_PHAI_NE_VAT";  break;
        case SIDE_ADJUST:       stateStr = "CAN_BANG_HONG";   break;
        case EMERGENCY_REVERSE: stateStr = "LUI_KHAN_CAP";    break;
        case TURN_AROUND:       stateStr = "QUAY_DAU_NGO_CUT";break;
    }
    
    cout << "  -> Trang thai: " << left << setw(18) << stateStr 
         << "| Lenh xuat: v = " << fixed << setprecision(2) << v 
         << " m/s, omega = " << omega << " rad/s\n";
}

int main() {
    Navigator nav;
    float v, omega;

    cout << "================ CHUONG TRINH GIA LAP NE VAT CAN ================\n";

    // Kịch bản 1: Đường thoáng, AI yêu cầu đi thẳng
    cout << "\n[TEST 1] Duong thoang (Left:100, Front:150, Right:100) - AI: FORWARD\n";
    SensorData test1 = {100.0, 150.0, 100.0};
    nav.computeControl(test1, CMD_FORWARD, v, omega);
    mockMotorControl(v, omega, nav.currentState);

    // Kịch bản 2: Có vật cản phía trước, bên phải rộng hơn
    cout << "\n[TEST 2] Vat can truoc (Left:20, Front:30, Right:80) - AI: FORWARD\n";
    SensorData test2 = {20.0, 30.0, 80.0};
    nav.computeControl(test2, CMD_FORWARD, v, omega);
    mockMotorControl(v, omega, nav.currentState);

    // Kịch bản 3: Vật cản quá sát đầu xe, ưu tiên lùi khẩn cấp bất chấp lệnh AI
    cout << "\n[TEST 3] Qua sat truoc (Left:50, Front:10, Right:50) - AI: LEFT\n";
    SensorData test3 = {50.0, 10.0, 50.0};
    nav.computeControl(test3, CMD_LEFT, v, omega);
    mockMotorControl(v, omega, nav.currentState);

    cout << "\n=================================================================\n";
    return 0;
}