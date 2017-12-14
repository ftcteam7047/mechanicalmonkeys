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
    double LIFT_HI_SPEED = 1.0;
    double GRAB_DISTANCE_FROM_START = 0;
    double MOVE_HEIGHT = 2.0;
    double STACK_HEIGHT = 7.5;
    double PLACE_HEIGHT = 14.5; // after decreasing the lower limit of the lifter, it gains extra height
    double MOVE_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * MOVE_HEIGHT;
    double STACK_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * STACK_HEIGHT;
    double PLACE_DISTANCE_FROM_START = COUNTS_PER_INCH_16_TOOTH * PLACE_HEIGHT;
    double DIAGONAL_HALF_BAND_WIDTH = 0.30; // max shall be less than sqrt(2)
    // for ver 2 slider
    double SLIDER_LOW_HEIGHT = 0.0;
    double SLIDER_HIGH_HEIGHT = 7.0;
    int COUNTS_PER_MOTOR_REV_20 = 560;
    double SLIDER_PULLEY_DIAMETER_INCHES = 0.465;
    double COUNTS_PER_INCH_SLIDER_PULLEY = COUNTS_PER_MOTOR_REV_20 / (SLIDER_PULLEY_DIAMETER_INCHES * Math.PI);
    double LOW_DISTANCE_FROM_START = COUNTS_PER_INCH_SLIDER_PULLEY * SLIDER_LOW_HEIGHT;
    double HIGH_DISTANCE_FROM_START = COUNTS_PER_INCH_SLIDER_PULLEY * SLIDER_HIGH_HEIGHT;

    double BALL_ARM_UP = 0.0;
    double BALL_ARM_DOWN = 1.0;

    double BALL_SPEED = 0.1;
    double BALL_DISTANCE = 2.75;

    double DRIVE_OFF_PLATFORM_MORE_OFFSET = 2.75;
    // Increased due to possible slip when on platform
    double DRIVE_OFF_PLATFORM_LESS_OFFSET = -3.75;
    double DRIVE_OFF_PLATFORM_DISTANCE_WITHOUT_OFFSET = 26;
    double DRIVE_OFF_TIP_PLATFORM_DISTANCE_WITHOUT_OFFSET = 26;
    double DRIVING_OFF_PLATFORM_SPEED = 0.3;
    // drive off platform parameters for V2
    double DRIVE_OFF_PLATFORM_DISTANCE_V2_WITHOUT_OFFSET = 28; // this has to be adjusted from V1, because the intake is in the center whereas the gripper is off-centered
    double DRIVE_OFF_BLUE_PLATFORM_DISTANCE_V2_WITHOUT_OFFSET = 22.5;
    double DRIVE_OFF_TIP_PLATFORM_V2_DISTANCE_WITHOUT_OFFSET = 27;

    double RED_TURN_ANGLE = -35;
    double BLUE_TURN_ANGLE = 60;
    double PHONE_MOUNT_V2_ANGLE = -12; // since phone mount is at an angle in Curious George V2
    double RED_TURN_ANGLE_V2 = RED_TURN_ANGLE + PHONE_MOUNT_V2_ANGLE;
    double BLUE_TURN_ANGLE_V2 = BLUE_TURN_ANGLE + PHONE_MOUNT_V2_ANGLE;

    double VU_MARK_DETECTION_TIMEOUT = 5.0;

    double TELEOP_SLOW_MODE_TOP_SPEED = 0.42;
    boolean USE_Y_BUTTON_FOR_LOW_SPEED_MODE = true;

    double MOTOR_STALL_CHECKING_PERIOD = 0.3;
    int    secondsPerMinute            = 60;
    double ANDYMARK_60_COUNTS_PER_MOTOR_REV = 1680.0;
    int    ANDYMARK_60_NO_LOAD_MAX_RPM  = 105; // Andymark NeverRest 60 See http://www.andymark.com/NeveRest-60-Gearmotor-p/am-3103.htm
    int ANDYMARK_60_MAX_COUNT_PER_SEC = (int)(CargoBotConstants.ANDYMARK_60_COUNTS_PER_MOTOR_REV * CargoBotConstants.ANDYMARK_60_NO_LOAD_MAX_RPM / CargoBotConstants.secondsPerMinute);
    double ANDYMARK_20_COUNTS_PER_MOTOR_REV = 560.0;
    int    ANDYMARK_20_NO_LOAD_MAX_RPM  = 315;
    int ANDYMARK_20_MAX_COUNT_PER_SEC = (int)(CargoBotConstants.ANDYMARK_20_COUNTS_PER_MOTOR_REV * CargoBotConstants.ANDYMARK_20_NO_LOAD_MAX_RPM / CargoBotConstants.secondsPerMinute);

    double MOTOR_TOP_SPEED = 1.0;
    double MOTOR_STALL_RATIO = 0.3;
    double MOTOR_VOLTAGE = 12.0; // Andymark performance spec, See https://www.andymark.com/NeveRest-60-Gearmotor-p/am-3103.htm
    float TZ_CORRECTION_FACTOR = (float) 0.01;

    // servo position

    double LEFT_OPEN = 0.4; // was 0.0 because now used for half open
    double RIGHT_OPEN = 0.6; // was 1.0 because now used for half open

    double LEFT_CLOSE = 0.5; // was .55
    double RIGHT_CLOSE = 0.5; // was .45

    double LEFT_WIDE_OPEN = 0.0; // was 1.0 because now is used for open
    double RIGHT_WIDE_OPEN = 1.0; // was 0.0 because now is used for open

    // cryptoBox for relic
    // for curious george v1
    double ANGLE_TO_FACE_BOX_RED_RELIC = 90.0;
    double CRYPTO_BOX_DISTANCE_RED_RELIC = 7.0;
    double APPROACH_SPEED = 0.2;
    double BACKUP_DISTANCE = -4.5;
    double MOVE_TO_CENTER_DISTANCE_RELIC = 7.5;
    double MOVE_TO_LEFT_DISTANCE_RELIC = MOVE_TO_CENTER_DISTANCE_RELIC * 2;
    // for curious george v2
    double CRYPTO_BOX_DISTANCE_V2_RED_RELIC = 6.0;
    double BACKUP_DISTANCE_V2 = -5.5; // need to back up enough so that there is enough room to turn around without hitting the cryptoBox
    double BACKUP_PUSH_BLOCK_DISTANCE = -11.0;
    double ANGLE_TO_FACE_FIELD_CENTER_RED_BLUE_RELIC = -90.0;
    double AWAY_FROM_BLOCK_DISTANCE = 6.0;

    // cryptoBox for tip
    double BACKUP_OFFSET_TO_APPROACH_TIP_BOX = 2.75;
    double ANGLE_TO_FACE_BOX_RED_TIP = 180.0;
    double CRYPTO_BOX_DISTANCE_RED_TIP = 5.5;
    // for curious george v2
    double ANGLE_TO_FACE_FIELD_CENTER_RED_TIP = 0;
    double CRYPTO_BOX_DISTANCE_V2_RED_TIP = 2.5;
    double BACKUP_OFFSET_V2_TO_APPROACH_TIP_BOX = 4.25;
    double ANGLE_TO_FACE_FIELD_CENTER_BlUE_TIP = 180;

    // cryptoBox for blue relic
    double BLUE_RELIC_COLUMN_OFFSET = 7.5;
    double DRIVE_BLUE_OFF_PLATFORM_MORE_OFFSET = 1.75;
    double CRYPTO_BOX_DISTANCE_BLUE_RELIC = 8.5;
    // for curious george v2
    double CRYPTO_BOX_DISTANCE_V2_BLUE_RELIC = 5.5;

    // cryptoBox for blue tip
    double ANGLE_TO_FACE_BLUE_TIP_COLUMN = 1;
    double BLUE_TIP_COLUMN_OFFSET = 8.0;
    // for curious george v2
    double BLUE_TIP_COLUMN_V2_OFFSET = 4.5; // this has to be adjusted from V1, because the intake is in the center whereas the gripper is off-centered


    // diagonal movement
    boolean DO_DIAGONAL = false;

    float PI_MULTIPLIER = 1.0f;
    float CONTROLLER_DEAD_ZONE = 0.40f;

    // remap controller y value to a different output curve
    boolean REMAP_CONTROLLER_Y = true;

    // pushing both sticks in the same direction result in same motor power on left and right
    boolean EQUALIZE_MOTOR_POWER = true;

    // during teleop, set motor zero power behavior
    boolean SET_MOTOR_ZERO_POWER = true;

    // set lift motor to high speed, always
    boolean SET_LIFT_MOTOR_HIGH_SPEED = true;

    // test constants
    double EXTENDED_BACKUP_DISTANCE = -16.5;
    double BACKUP_SPEED = 0.6;
    double FORWARD_TO_PIT_DISTANCE_RELIC = 12;
    double TEST_ANGLE = 0;
    double BACKUP_TO_SAFE_ZONE_SPEED = 0.8;

    // path constant
    String pathToLiftMotorOffset = "/FTC/liftMotorOffset";

    // back up distance to the platform for parking
    double BACKUP_ONTO_PLATFORM_DISTANCE = -6.9;
    double CONTROL_STICK_THRESHOLD_FOR_BACKUP_TO_PLATFORM = 0.1;
    double BACKUP_ONTO_PLATFORM_SPEED = 0.7;

    // lift position tolerance, to check how close the lift is to the target
    // if it's close enough, then stop requesting the moveRampToFlat or moaveRampToDown functions
    int LIFT_TARGET_TOLERANCE = 30;
    double LIFT_MOTOR_TIMEOUT = 3.0;

    // intake motors
    double BOTH_INTAKE_MOTOR_ACTIVATION_TIME = 2;
    double FRONT_INTAKE_POWER = 0.5;
    double LEFT_INTAKE_POWER = 0.85;
    double RIGHT_INTAKE_POWER = 0.85;
    double PRIMARY_INTAKE_MOTOR_ACTIVATION_TIME = 3;

    // Delay for lowering the ball arm
    double LOWER_BALL_ARM_DELAY = 0.5;

    // lift servo parameters
    // servo parameters
    double DOWN_TARGET = 0.82;
    double UP_TARGET = 0.0;
    double FLAT_TARGET = 0.66;
    double TIME_INCREMENT = 0.025;
    double POS_INCREMENT = 0.02;

    // ball detection timeout
    double BALL_DETECTION_TIMEOUT = 10.0;
    double PRE_INIT_BALL_DETECTION_TIMEOUT = 5.0;
}

