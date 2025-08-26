#include "controller.hpp"

// -------------------- Public API --------------------
bool EyeController::begin(uint8_t i2c_addr)
{
    Wire.setTimeOut(50); // avoid hang if device missing

    pwm_ = Adafruit_PWMServoDriver(i2c_addr);
    pwm_.begin();
    pwm_.setOscillatorFrequency(OSCILLATOR); // set before setPWMFreq
    pwm_.setPWMFreq(SERVO_FREQ);
    delay(10);

    // Start centered/open
    left_sep_ = RESTING_SEP;
    right_sep_ = RESTING_SEP;

    last_tl_angle_ = left_sep_ / 2;
    last_bl_angle_ = left_sep_ / 2;
    last_tr_angle_ = right_sep_ / 2;
    last_br_angle_ = right_sep_ / 2;

    home();
    return true;
}

void EyeController::home()
{
    set_ud(0);
    set_lr(0);
    set_separation(RESTING_SEP);
    delay(MG90_DELAY_MS);
}

void EyeController::set_separation(uint8_t separation)
{
    const uint8_t left_sep = separation;
    const uint8_t right_sep = separation;

    const int new_lhs = int(left_sep) / 2;
    const int new_rhs = int(right_sep) / 2;
    const int old_lhs = int(left_sep_) / 2;
    const int old_rhs = int(right_sep_) / 2;

    // Adjust stored angles by delta (bugfix: +=)
    last_tl_angle_ += (new_lhs - old_lhs);
    last_bl_angle_ += (new_lhs - old_lhs);
    last_tr_angle_ += (new_rhs - old_rhs);
    last_br_angle_ += (new_rhs - old_rhs);

    left_sep_ = left_sep;
    right_sep_ = right_sep;

    set_tl(last_tl_angle_);
    set_tr(last_tr_angle_);
    set_bl(last_bl_angle_);
    set_br(last_br_angle_);
}

void EyeController::set_lr(int angle)
{
    angle = constrain(angle, -REL_LR_MAX, REL_LR_MAX);
    const int abs_servo = ABS_LR_CENTER + angle; // relative -> absolute
    set_servo(CH_LOOK_LR, abs_servo);
}

// eye.cpp (replace your set_ud with this)
void EyeController::set_ud(int angle)
{
    // 1) Drive the eyeball first (sign as-is: +up / -down)
    angle = constrain(angle, -REL_UD_MAX, REL_UD_MAX);
    const int abs_servo = ABS_UD_CENTER + angle;
    set_servo(CH_LOOK_UD, abs_servo);

    // 2) Lids track UD; optionally invert for your mechanism
    const int lid_ref = kInvertLidsUD ? -angle : angle;

    const int lhs = int(left_sep_) / 2;  // half separation (left eye)
    const int rhs = int(right_sep_) / 2; // half separation (right eye)

    auto clamp_open = [](int v)
    { return v < 0 ? 0 : v; };

    // Left lids (TL/BL)
    {
        int disp = abs(lid_ref) - lhs;
        if (disp < 0)
            disp = 0;
        if (lid_ref >= 0)
        {                                            // "up" for lids
            last_tl_angle_ = clamp_open(lhs + disp); // top opens
            last_bl_angle_ = clamp_open(lhs - disp); // bottom closes
        }
        else
        {                                            // "down" for lids
            last_tl_angle_ = clamp_open(lhs - disp); // top closes
            last_bl_angle_ = clamp_open(lhs + disp); // bottom opens
        }
        set_tl(last_tl_angle_);
        set_bl(last_bl_angle_);
    }

    // Right lids (TR/BR)
    {
        int disp = abs(lid_ref) - rhs;
        if (disp < 0)
            disp = 0;
        if (lid_ref >= 0)
        {
            last_tr_angle_ = clamp_open(rhs + disp);
            last_br_angle_ = clamp_open(rhs - disp);
        }
        else
        {
            last_tr_angle_ = clamp_open(rhs - disp);
            last_br_angle_ = clamp_open(rhs + disp);
        }
        set_tr(last_tr_angle_);
        set_br(last_br_angle_);
    }
}

void EyeController::blink()
{
    // Close
    set_bl(0);
    set_br(0);
    set_tl(0);
    set_tr(0);
    delay(BLINK_TIME_MS);
    // Open to last angles
    set_bl(last_bl_angle_);
    set_br(last_br_angle_);
    set_tl(last_tl_angle_);
    set_tr(last_tr_angle_);
}

// -------------------- Private helpers --------------------
uint16_t EyeController::angle_to_pulse(int angle,
                              int min_pulse,
                              int max_pulse)
{
    angle = constrain(angle, MG90_ANGLE_MIN, MG90_ANGLE_MAX);
    const int us = map(angle, MG90_ANGLE_MIN, MG90_ANGLE_MAX, min_pulse, max_pulse);
    float steps_f = float(us) / PCA9685_STEP_US;
    int steps = int(steps_f + 0.5f);
    if (steps < 0)
        steps = 0;
    if (steps > 4095)
        steps = 4095;
    return uint16_t(steps);
}

void EyeController::set_servo(uint8_t ch, int servo_angle_deg)
{
    servo_angle_deg = constrain(servo_angle_deg, MG90_ANGLE_MIN, MG90_ANGLE_MAX);
    const uint16_t pulse = angle_to_pulse(servo_angle_deg);
    pwm_.setPWM(ch, 0, pulse);
}

// Eyelid channel writers (use computed servo_angle, not raw "angle")
// 0 = closed, + opens
void EyeController::set_tr(uint32_t angle)
{
    int servo_angle = int(ABS_TR_CLOSED) - int(angle);
    servo_angle = constrain(servo_angle, ABS_TR_OPEN, ABS_TR_CLOSED);
    set_servo(CH_LID_TR, servo_angle);
}

void EyeController::set_tl(uint32_t angle)
{
    int servo_angle = int(ABS_TL_CLOSED) + int(angle);
    servo_angle = constrain(servo_angle, ABS_TL_CLOSED, ABS_TL_OPEN);
    set_servo(CH_LID_TL, servo_angle);
}

void EyeController::set_br(uint32_t angle)
{
    int servo_angle = int(ABS_BR_CLOSED) + int(angle);
    servo_angle = constrain(servo_angle, ABS_BR_CLOSED, ABS_BR_OPEN);
    set_servo(CH_LID_BR, servo_angle);
}

void EyeController::set_bl(uint32_t angle)
{
    int servo_angle = int(ABS_BL_CLOSED) - int(angle);
    servo_angle = constrain(servo_angle, ABS_BL_OPEN, ABS_BL_CLOSED);
    set_servo(CH_LID_BL, servo_angle);
}

void EyeController::test()
{
    Serial.println("BLINK");
    blink();
    delay(1000);

    Serial.println("LOOK UP");
    set_ud(+REL_UD_MAX);
    delay(1000);

    Serial.println("LOOK DOWN");
    set_ud(-REL_UD_MAX);
    delay(1000);

    Serial.println("LOOK LEFT");
    set_ud(0);
    set_lr(+REL_LR_MAX);
    delay(1000);

    Serial.println("LOOK RIGHT");
    set_lr(-REL_LR_MAX);
    delay(1000);

    Serial.println("BLINK");
    blink();
    set_lr(0);
    delay(150);
}