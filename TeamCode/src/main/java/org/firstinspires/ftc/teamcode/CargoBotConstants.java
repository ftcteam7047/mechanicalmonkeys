package org.firstinspires.ftc.teamcode;

/**
 * Created by michaelchen on 10/7/17.
 */

public interface CargoBotConstants {

    // relic servo position magic numbers
    double RELIC_ARM_CLOSE = 0.0;
    double RELIC_ARM_OPEN = 0.4;

    // block grabber position
    double GRABBER_CLOSE = 0.5; // only position that can be used throughout the program

    // lifter encoder magic numbers
    int COUNTS_PER_MOTOR_REV_60 = 1680;
    double SPROCKET_DIAMETER_INCHES_16_TOOTH = 1.3855; // for circumference
    double COUNTS_PER_INCH_16_TOOTH = COUNTS_PER_MOTOR_REV_60 / (SPROCKET_DIAMETER_INCHES_16_TOOTH * Math.PI);
    double LIFT_SPEED = 0.5;
    double GRAB_DISTANCE_FROM_START = 0;
    double MOVE_HEIGHT = 2.0;
    double STACK_HEIGHT = 7.5;
    double PLACE_HEIGHT = 13.5;
    double MOVE_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * MOVE_HEIGHT;
    double STACK_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * STACK_HEIGHT;
    double PLACE_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * PLACE_HEIGHT;
    double DIAGONAL_HALF_BAND_WIDTH = 0.30; // max shall be less than sqrt(2)

    double BALL_ARM_UP = 0.0;
    double BALL_ARM_DOWN = 1.0;

    double BALL_SPEED = 0.1;
    double BALL_DISTANCE = 2.75;

    double DRIVE_OFF_PLATFORM_MORE_OFFSET = 2.75;
    double DRIVE_OFF_PLATFORM_LESS_OFFSET = -2.75;
    double DRIVE_OFF_PLATFORM_DISTANCE_WITHOUT_OFFSET = 26;
    double DRIVING_OFF_PLATFORM_SPEED = 0.3;

    double RED_TURN_ANGLE = -35;
    double BLUE_TURN_ANGLE = 70;

    double VU_MARK_DETECTION_TIMEOUT = 3.0;

    double TELEOP_SLOW_MODE_TOP_SPEED = 0.3;
    boolean USE_Y_BUTTON_FOR_LOW_SPEED_MODE = true;

    double MOTOR_STALL_CHECKING_PERIOD = 0.3;
    int    secondsPerMinute            = 60;
    double ANDYMARK_60_COUNTS_PER_MOTOR_REV = 1680.0;
    int    ANDYMARK_60_NO_LOAD_MAX_RPM  = 105; // Andymark NeverRest 60 See http://www.andymark.com/NeveRest-60-Gearmotor-p/am-3103.htm
    int ANDYMARK_60_MAX_COUNT_PER_SEC = (int)(CargoBotConstants.ANDYMARK_60_COUNTS_PER_MOTOR_REV * CargoBotConstants.ANDYMARK_60_NO_LOAD_MAX_RPM / CargoBotConstants.secondsPerMinute);
    double MOTOR_TOP_SPEED = 1.0;
    double MOTOR_STALL_RATIO = 0.3;
    double MOTOR_VOLTAGE = 12.0; // Andymark performance spec, See https://www.andymark.com/NeveRest-60-Gearmotor-p/am-3103.htm
}
