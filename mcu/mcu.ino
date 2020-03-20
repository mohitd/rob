// Copyright (c) 2020. Mohit Deshpande.

#include <SPI.h>

static const byte MOTOR_L_PWM = 5;
static const byte MOTOR_L_DIR = 4;
static const byte MOTOR_R_PWM = 6;
static const byte MOTOR_R_DIR = 7;

static const float VEL_TO_PWM = 1.9f;
static const byte PWM_MIN = 75;
static const byte PWM_MAX = 110;

static const float WHEEL_BASE = 0.185f;
static const float WHEEL_RADIUS = 0.02f;

static const float V_MAX = 1.0;
static const float V_MIN = -1.0;
static const float V_EPS = 0.05;

static const float W_MAX = 3.14;
static const float W_MIN = -3.14;
static const float W_EPS = 0.05;

static volatile byte buffer[8];
static volatile byte buffer_idx = 0;
static volatile boolean complete_packet = false;
float cmd_vel[2];

void init_drivetrain();
void drivetrain_command(float v, float w);

void setup()
{
    Serial.begin(9600);
    init_drivetrain();
    
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);
    SPDR = 0;
    Serial.print("MCU initialized!");
}

void loop()
{
    if (complete_packet)
    {
        memcpy(cmd_vel,&buffer,8);

        drivetrain_command(cmd_vel[0], cmd_vel[1]);
        
        buffer_idx = 0;
        complete_packet = false;
    }
    delay(10);
}

ISR(SPI_STC_vect)
{
    if (buffer_idx < sizeof(buffer))
    {
        buffer[buffer_idx++] = SPDR;
    }
    else
    {
        complete_packet = true;
    }
    
}

void init_drivetrain()
{
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);
    digitalWrite(MOTOR_L_DIR, LOW);
    digitalWrite(MOTOR_R_DIR, LOW);
}

// drive a commanded linear and angular velocity
void drivetrain_command(float v, float w)
{
    // small (v,w) -> 0 and rest get constrained to [MIN, MAX]
    v = abs(v) < V_EPS ? 0.0f : max(V_MIN, min(v, V_MAX));
    w = abs(w) < W_EPS ? 0.0f : max(W_MIN, min(w, W_MAX));

    // right-hand rule: a positive w => turning left, robot-relative
    float w_l = ((2 * v) - (w * WHEEL_BASE)) / (2*WHEEL_RADIUS);
    float w_r = ((2 * v) + (w * WHEEL_BASE)) / (2*WHEEL_RADIUS);

    // direction of motors
    digitalWrite(MOTOR_L_DIR, w_l > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_DIR, w_r > 0 ? HIGH : LOW);

    // map velocities to positive PWM by multiplying by a magic number
    int pwm_l = (int) abs(w_l * VEL_TO_PWM);
    int pwm_r = (int) abs(w_r * VEL_TO_PWM);
    
    // special case for 0
    pwm_l = pwm_l == 0 ? 0 : constrain(pwm_l, PWM_MIN, PWM_MAX);
    pwm_r = pwm_r == 0 ? 0 : constrain(pwm_r, PWM_MIN, PWM_MAX);

    analogWrite(MOTOR_L_PWM, pwm_l);
    analogWrite(MOTOR_R_PWM, pwm_r);
}
