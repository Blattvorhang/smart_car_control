#include "headfile.h"
#include "math.h"
#pragma section all "cpu1_dsram"

/* hardware parameters */
#define S_MOTOR_PIN ATOM1_CH1_P33_9
#define MOTOR_DIR P02_6
#define MOTOR_PWM ATOM0_CH7_P02_7
#define S_MOTOR_LIMIT 180        // degree
#define SERVO_MOTOR_MAX_ANGLE 40 // degree

/* mode */
#define DEBUG 0
#define START 1
#define START_DIRECTLY 0
#define FREEZONE_EN 1
#define GO_AROUND 0

/* data sent by uart */
extern volatile float speed;
extern volatile short angle;
#if FREEZONE_EN
extern volatile short dir_go_around; // 1 for right, -1 for left, 0 for none.
#endif

/* velocity sample */
int16 speed_value;

/* other parameter */
#define SAMPLING_INTERVAL 10 // ms, MUST NOT CHANGE!!!
#define SPEED2POWER(v) (1500 * v)
#define PERIMETER 9200
#define OFFSET 11 // negative as right, positive as left

/**
 * It takes an angle as an argument, and turns the servo motor to that angle
 *
 * @param angle the angle to turn the servo motor to, positive for left, negative for right.
 */
void turn_servo_motor(short angle)
{
    if (angle < -SERVO_MOTOR_MAX_ANGLE)
        angle = -SERVO_MOTOR_MAX_ANGLE;
    else if (angle > SERVO_MOTOR_MAX_ANGLE)
        angle = SERVO_MOTOR_MAX_ANGLE;
    uint32 pwm = OFFSET + 10000.0 * (1.5 + angle / (S_MOTOR_LIMIT / 2.0)) / 20.0;
    pwm_duty(S_MOTOR_PIN, pwm);
}

/**
 * It set the speed of the motor.
 *
 * @param speed the speed of the motor, in RPM
 */
void set_motor_speed(float speed)
{
    int32 speed_power = SPEED2POWER(speed); // SPEED2POWER convert formula
    if (0 <= speed_power)
    {
        pwm_duty(MOTOR_PWM, speed_power);
        gpio_set(MOTOR_DIR, 0);
    }
    else
    {
        pwm_duty(MOTOR_PWM, -speed_power);
        gpio_set(MOTOR_DIR, 1);
    }
}

/**
 * run for certain circle with constant velocity.
 *
 * @param n_circle number of circles to run
 */
void run_dist(float n_circle)
{
    int64 distance = 0;
    while (distance < (int64)(PERIMETER * n_circle))
    {
        speed_value = gpt12_get(GPT12_T2);
        gpt12_clear(GPT12_T2);
        systick_delay_ms(STM0, SAMPLING_INTERVAL);
        distance += speed_value * SAMPLING_INTERVAL;
    }
}

/**
 * Start the car by going straight, and turn left.
 */
void start_car(void)
{
    /* parameters */
    const float n_go_straight = 1.4;
    const float n_turn = 4.3;
    const uint32 motor_pwm_straight = 2700;
    const uint32 motor_pwm_turn = 2000;
    const short turn_angle = 25;

    /* actions */
    gpio_set(MOTOR_DIR, 0);

    /* Going straight for a certain distance. */
    turn_servo_motor(0);
    pwm_duty(MOTOR_PWM, motor_pwm_straight);
    run_dist(n_go_straight);

    /* Turning the car to the left. */
    turn_servo_motor(turn_angle);
    pwm_duty(MOTOR_PWM, motor_pwm_turn);
    run_dist(n_turn);

    turn_servo_motor(0);
    pwm_duty(MOTOR_PWM, 0);
}

/**
 * go around the forbidden sign.
 * 
 * @param dir the direction of the turn. 1 for right, -1 for left.
 */
void go_around(short dir)
{
    const short go_around_angle = 20;
    const uint32 motor_pwm_turn = 1800;
    const float n_go_around = 2.2;
    pwm_duty(MOTOR_PWM, motor_pwm_turn);
    turn_servo_motor(-dir * go_around_angle);
    run_dist(n_go_around);
    turn_servo_motor(dir * go_around_angle);
    run_dist(n_go_around);
    run_dist(n_go_around);
    turn_servo_motor(-dir * go_around_angle);
    run_dist(n_go_around);
    turn_servo_motor(0);
}

/**
 * shift the car vertical to the path.
 * 
 * @param dir the direction of the shift. 1 for right, -1 for left.
 */
void shift(short dir)
{
    const short go_around_angle = 30;
    const uint32 motor_pwm_turn = 1800;
    const float n_go_around = 2.0;
    pwm_duty(MOTOR_PWM, motor_pwm_turn);
    turn_servo_motor(-dir * go_around_angle);
    run_dist(n_go_around);
    turn_servo_motor(0);
    run_dist(n_go_around);
    turn_servo_motor(dir * go_around_angle);
    run_dist(n_go_around);
    turn_servo_motor(0);
}

void core1_main(void)
{
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    enableInterrupts();

    // init servo motor
    uint32 duty = 1.5 * 10000 / 20;
    gtm_pwm_init(S_MOTOR_PIN, 50, duty);

    // init motor
    gtm_pwm_init(MOTOR_PWM, 17000, 0);
    gpio_init(MOTOR_DIR, GPO, 0, PUSHPULL);
    gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
#if DEBUG
    int flag = 0;
    speed = 1;
#endif
#if START
    int flag_started = 0;
#endif
#if START_DIRECTLY
    systick_delay_ms(STM0, 3000);
    start_car();
#endif
#if GO_AROUND
    go_around(-1);
#endif

    while (1)
    {
#if START
        /* Used to start the car. */
        if (!flag_started && (0 != angle || fabs(speed) >= 1e-6))
        {
            start_car();
            flag_started = 1;
        }
        if (flag_started && (fabs(speed) < 1e-6))
        {
            flag_started = 0;
        }
#endif
#if FREEZONE_EN
        if (dir_go_around)
        {
            go_around(dir_go_around);
            dir_go_around = 0;
        }
#endif
#if DEBUG
        angle = 0; /*flag ? (angle + 1) : (angle - 1);
         if (angle > SERVO_MOTOR_MAX_ANGLE)
             flag = 0;
         else if (angle < -SERVO_MOTOR_MAX_ANGLE)
             flag = 1;*/
#endif
        /* A loop to get the speed value,
         * clear the speed value,
         * turn the servo motor,
         * set the motor speed,
         * and delay for sample interval.
         */
        speed_value = gpt12_get(GPT12_T2);
        gpt12_clear(GPT12_T2);
        turn_servo_motor(-angle);
        set_motor_speed(speed);
        systick_delay_ms(STM0, SAMPLING_INTERVAL);
    }
}

#pragma section all restore
