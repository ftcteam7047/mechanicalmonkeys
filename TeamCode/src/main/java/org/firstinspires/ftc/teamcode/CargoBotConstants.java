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
}
