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
    double GRAB_HEIGHT = 1.0;
    double STACK_HEIGHT = 7.5;
    double PLACE_HEIGHT = 13.5;
    double MOVE_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * GRAB_HEIGHT;
    double STACK_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * STACK_HEIGHT;
    double PLACE_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * PLACE_HEIGHT;
    double DIAGONAL_HALF_BAND_WIDTH = 0.30; // max shall be less than sqrt(2)

    double BALL_ARM_UP = 0.0;
    double BALL_ARM_DOWN = 1.0;

    double BALL_SPEED = 0.1;
    double BALL_DISTANCE = 2.75;

    //TODO: For driving off platform, if going from center go 26 inches witout offset of 2.75 to clear platfrom and speed of 0.3
}
