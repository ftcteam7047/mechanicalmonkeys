package org.firstinspires.ftc.teamcode;

/**
 * Created by chen14 on 11/1/2016.
 */

interface MMShooterBotConstants {
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    double     ANDYMARK_60_COUNTS_PER_MOTOR_REV     = 1680.0;
    double     ANDYMARK_40_COUNTS_PER_MOTOR_REV     = 1120.0;
    double     ANDYMARK_20_COUNTS_PER_MOTOR_REV     = 560.0;
    double     ANDYMARK_ORBITAL_20_COUNTS_PER_MOTOR_REV = 537.6;
    double     COUNTS_PER_MOTOR_REV    	            = ANDYMARK_ORBITAL_20_COUNTS_PER_MOTOR_REV ; // apply what is being installed as the drive motors
    double     COUNTS_PER_TETRIX_MOTOR_REV          = 1440.0 ;    // eg: TETRIX Motor Encoder: 1440 is the reference
    double     MOTOR_ENCODER_SCALE_FACTOR           = (COUNTS_PER_MOTOR_REV/COUNTS_PER_TETRIX_MOTOR_REV);    // For Textrix: set 1, for AndyMark: set 0.39
    double     DRIVE_SPEED             	            = 0.2 * MOTOR_ENCODER_SCALE_FACTOR;     // Nominal speed for better accuracy.
    double     TURN_SPEED              	            = 0.2 * MOTOR_ENCODER_SCALE_FACTOR;     // Nominal half speed for better accuracy.
    double     GYRO_TURN_SPEED                      = 0.15 * MOTOR_ENCODER_SCALE_FACTOR;     // Nominal half speed for better accuracy.
    double     STOP_DISTANCE           	            = 7.0;     // unit: cm Distance to stop in front of wall. This is the distance between the front of the range sensor and the wall.
    double     STOP_DISTANCE_HIGHER_SPEED           = 10.0;    // unit: cm Distance to stop in front of wall. This is the distance between the front of the range sensor and the wall.
    double     HEADING_THRESHOLD       	            = 3 ;      // As tight as we can make it with an integer gyro
    double     P_TURN_COEFF            	            = 0.1;     // 0.1 for Tetrix motor: Larger is more responsive, but also less stable
    double     P_DRIVE_COEFF           	            = 0.025;   // Larger is more responsive, but also less stable
    int        ALPHA_OFFSET                         = 8;
    int        ALPHA_OFFSET_CARPET                  = 0;
    double     BASE_MOTOR_SPEED                     = 0.2 * MOTOR_ENCODER_SCALE_FACTOR;
    double     ANGLE_THRESHOLD_FOR_TURN_SPEED_BUMP  = 30; // if delta angle is greater than the threshold, then speed up the turn speed
    double     GYRO_DRIVE_TOLERANCE_DEGREE          = 2 * COUNTS_PER_MOTOR_REV/360.0;
    double     FULL_DRIVE_SPEED                     = 1.0;
    double     INTERCEPTING_WHITE_LINE_DRIVE_SPEED  = 0.55;
    double     STOP_DISTANCE_FIRST_BEACON           = 10.5;    // unit: cm Distance to stop in front of wall. This is the distance between the front of the range sensor and the wall.

    double     TETRIX_COUNTS_PER_MOTOR_REV          = 1440.0;
    int        ANDYMARK_60_NO_LOAD_MAX_RPM          = 105; // Andymark NeverRest 60 See http://www.andymark.com/NeveRest-60-Gearmotor-p/am-3103.htm
    int        ANDYMARK_40_NO_LOAD_MAX_RPM          = 160; // Andymark NeverRest 40 See http://www.andymark.com/NeveRest-40-Gearmotor-p/am-2964a.htm
    int        ANDYMARK_20_NO_LOAD_MAX_RPM          = 315; // Andymark NeverRest 20 See http://www.andymark.com/NeveRest-20-12V-Gearmotor-p/am-3102.htm
    int        TETRIX_MAX_RPM                       = 152; // TETRIX MAX DC Motor http://www.tetrixrobotics.com/Motors/TETRIX_DC_Gear_Motor
    int        secondsPerMinute                     = 60;
    double     ANDYMARK_AM3104_COUNTS_PER_MOTOR_REV = 28;
    int        ANDYMARK_AM3104_NO_LOAD_MAX_RPM      = 6600; // http://www.andymark.com/product-p/am-3200.htm
    int        ANDYMARK_AM3104_WITH_LOAD_MAX_RPM    = (int) (3080.0/ANDYMARK_AM3104_COUNTS_PER_MOTOR_REV * secondsPerMinute); // 2600 is based on measurement using RUN_WITHOUT_ENCODER and set power to 1.0
    int        SHOOTER_MOTOR_MAX_TICK_PER_SEC       = (int)(ANDYMARK_AM3104_WITH_LOAD_MAX_RPM  * ANDYMARK_AM3104_COUNTS_PER_MOTOR_REV / secondsPerMinute);
    int        ANDYMARK_60_MAX_TICK_PER_SEC         = (int)(MMShooterBotConstants.ANDYMARK_60_COUNTS_PER_MOTOR_REV * MMShooterBotConstants.ANDYMARK_60_NO_LOAD_MAX_RPM / MMShooterBotConstants.secondsPerMinute);
    int        ANDYMARK_40_MAX_TICK_PER_SEC         = (int)(MMShooterBotConstants.ANDYMARK_40_COUNTS_PER_MOTOR_REV * MMShooterBotConstants.ANDYMARK_40_NO_LOAD_MAX_RPM / MMShooterBotConstants.secondsPerMinute);
    int        TETRIX_MAX_TICK_PER_SEC              = (int)(MMShooterBotConstants.TETRIX_COUNTS_PER_MOTOR_REV * MMShooterBotConstants.TETRIX_MAX_RPM / MMShooterBotConstants.secondsPerMinute);

    double     DRIVE_GEAR_REDUCTION                 = 1.0 ;     // This is < 1.0 if geared UP
    double     WHEEL_DIAMETER_INCHES                = 4.0 ;     // For figuring circumference
    double     CORRECTION_FACTOR                    = 30.0 / 29.21;
    double     COUNTS_PER_INCH         	            = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * CORRECTION_FACTOR;
    double     ABORT_DURATION          	            = 0.5;	   // Duration to stay in ABORT state for line following
    double     AUTO_AIM_ABORT_DURATION              = 0.1;	   // Duration to stay in ABORT state for line following
    double     BLACK_SQUARE_WIDTH_INCHES            = 23.5;	   // in inches
    double     LargeRobotWheelDistance              = 46;      // in cm, based on the design of the robot
    double     TestRobotWheelDistance               = 36;      // in cm, based on the design of the robot
    double     meterPerInch                         = 0.0254;
    double     meterPerMM                           = 0.001;
    double     g                                    = 9.8;      // standard gravity, see https://en.wikipedia.org/wiki/Standard_gravity
    double     meterShooterHeight                   = 11.5 * meterPerInch;  // based on the design of the robot  // TODO: 11/3/2016 measure actual height of the shooter wheel
    double     meterVortexCenterHeight              = 35.5 * meterPerInch;  // based on field set up             // TODO: 11/3/2016 find out actual height of the vortex center
    double     meterShooterWheelRadius              = 2 * meterPerInch;     // based on the design of the robot
    double     radiansPerRPM                        = 0.10471;
    int        shooterGearBoxReduction              = 4;
    double     shooterWheelRpmAt100Percent          = ANDYMARK_AM3104_WITH_LOAD_MAX_RPM/shooterGearBoxReduction;
    double     maxMotorSpeed                        = 1.0;
    double     shooterMotorTuneAmount               = 0.05;
    double     mmPerInch                            = 25.4f;
    double     shooterOffsetFromAxleOfDrivingWheels = 8 * mmPerInch;
    double     ONE_HUNDRED                          = 100;      // for converting the number to percentage
    double     POWER_THRESHOLD_FOR_SWITCH_DIRECTION = 0.1;      // a threshold for switching drive motor direction

    // puncher constants
    double      leftPunch                           = 0.00;     // left servo limit
    double      rightPunch                          = 1.00;     // right servo limit
    double      neutralPunch                        = 0.5;      // neutral servo position
    double      punchDuration                       = 1.0;      // how long the arm stays in punch state
    double      FULLY_PRESSED                       = 0.8;      // when an analog control is fully activated
    double      neutralOffsetCalibration            = 0.0;      // to compensate for one arm not able to fully return to the neutral position
    int         beaconPressRetryLimit               = 1;        // number of retry for button punching before giving up
    double      RELEASE_THRESHOLD                   = 0.1;      // the low limit that is considered releasing of the trigger

    // shooter constants
    double      SHOOTER_REV_UP_TIME                 = 2;        // seconds
    double      MOTOR_ADJUSTMENT_DELAY              = 1;        // seconds
    double      ELEVATE_MOTOR_ON_DURATION_OLD       = 1.0;      // seconds
    double      ELEVATE_MOTOR_ON_DURATION           = 1.5;      // seconds
    double      ELEVATE_MOTOR_ON_DURATION_2ND_BALL  = 1.5;      // seconds
    double      ELEVATE_MOTOR_ON_DURATION_TELEOP    = 2.0;      // seconds
    double      INTAKE_MOTOR_ON_DURATION            = 0.7;      // seconds
    String      SHOOTER_TAG                         = "Shooter data";
    String      GYRO_DRIVE_TAG                      = "Gyro drive data";
    double      SHOOTER_MOTOR_STABLE_TIME           = 5.0;      // seconds
    double      RPM_SCALE_FACTOR                    = 1.34 * (6.0/8.0);     // based on the ratio of theoretical distance V.S. actual distance measurement
    double      ELEVATOR_MOTOR_POWER_FOR_SHOOTING   = 0.8;
    double      INTAKE_MOTOR_POWER_FOR_SHOOTING     = 0.8;
    double      SHOOTER_MOTOR_REVERSE_POWER         = 0.15;     // only used if the ball is accidentally elevated into the barrel
    double      INITIAL_SHOOTER_POWER               = 0.65;     // was 0.65
    double      SHOOTER_TO_ROBOT_CENTER_OFFSET      = 3.5 * mmPerInch;
    double      SHOOTER_POWER_OFFSET                = 0.05; // add x% to the shooter power after we calculate the distance

    // autonomous
    double     BACKUP_DISTANCE                      = -11.5;
    double     DIAGONAL_DISTANCE                    = 30;
    double     ROTATION_ANGLE_TOWARDS_WHITE_LINE    = -132;
    double     NAVX_ANGLE_TOWARDS_WHITE_LINE        = -138;
    double     RED_ANGLE_TOWARDS_WHITE_LINE         = -133;
    double     BLUE_ANGLE_TOWARDS_WHITE_LINE        = 133;
    double     BACKUP_DISTANCE_TO_CAP_BALL          = -35;
    double     BACKUP_DISTANCE_TO_BLACK_SQUARE      = -10;
    double     ANGLE_TO_TIP_THE_CAP_BALL            = -30;
    double     ANGLE_TO_TIP_THE_BALL_VARIANT_1_2    = 30;
    double     TIME_PER_11_INCH                     = 1.5;
    double     TIMEOUT_SCALING_FACTOR               = TIME_PER_11_INCH * (MMShooterBotConstants.DRIVE_SPEED * 2.2)/11.0; // reference distance is 11 inches; reference speed is 2x DIRVE_SPEED
    double     TIMEOUT_DIAGONAL_DISTANCE            = 3.6; // 3.6 for field; 4.0 for home
    double     TIMEOUT_DIAGONAL_DISTANCE_VAR_SPEED  = 3.7; // home carpet use 3.0, field matt measure at 2.62
    double     DIAGONAL_DECELERATION_TIME           = 1.7;
    int        COLOR_READING_LIMIT                  = 255;
    int        MIN_ANGLE_CORRECTION_LIMIT           = 5;   // only make correction if we are off by more than 5 degrees
    double     ROTATION_ANGLE_TOWARDS_VORTEX        = 27;
    long       WAIT_TIME_VARIANT_1_1                = 10000; // unit is milliseconds so this is 10 seconds
    long       WAIT_TIME_VARIANT_1_2                = 10000; // unit is milliseconds so this is 10 seconds
    long       WAIT_TIME_VARIANT_2_1                = 5000; // unit is milliseconds so this is 5 seconds
    double     ROTATION_ANGLE_TOWARDS_PLATFORM      = -27;
    double     BACKUP_DISTANCE_VARIANT_1_1          = -47;
    double     BACKUP_DISTANCE_VARIANT_2_1          = -45;
    double     BACKUP_DISTANCE_VARIANT_1_2          = -3;
    double     BACKUP_DISTANCE_FROM_BEACON          = -13.25;
    double     ROTATION_ANGLE_TOWARDS_2ND_BEACON    = -80;
    double     WHITE_LINE_DECELERATION_TIME         = 1.45;
    double     TIMEOUT_WHITELINE_DISTANCE           = 2.8;
    long       WHILE_LOOP_TICK_IN_MS                = 2;
    double     BACKUP_RETRY_WHITELINE_DISTANCE      = -13.5;
    double     BACKUP_RETRY_NAVX_WHITELINE_DISTANCE = -9;
    double     SMALL_ANGLE_TOWARDS_WHITE_LINE       = 35;
    boolean    USE_VARIABLE_SPEED                   = false;
    boolean    USE_VARIABLE_SPEED_2_BEACONS         = true;
    double     RETRY_SCALING_FACTOR                 = 1.2;
    double     WHITE_REFERENCE_CAL                  = 40.0;
    double     CURVE_START_ANGLE                    = 45;
    double     CURVE_RATIO                          = 0.53; // inner radius divided by outer radius of the curve. inner circle is the trajectory of the wheels on the inside.
    double     CURVE_TIMEOUT                        = 2.92;
    double     WHITE_COLOR_OFFSET_DURING_TURN       = 3;
    double     BAKCKUP_DISTANCE_FROM_WALL           = -3.0;
    double     WHITE_LINE_DETECTION_RETRY_ANGLE     = -179; // absolute angle relative to the staring position
    double     SPEED_MULTIPLIER_CURVE_DRIVE         = 5.0;
    boolean    USE_LINE_FOLLOWING                   = false;
    double     SMALL_FORWARD_DISTANCE               = 1.0;
    double     MEDIUM_FORWARD_DISTANCE              = 1.75;
    double     LONG_FORWARD_DISTANCE                = 2.5;
    double     APPROACH_BEACON_ANGLE                = -90;
    double     APPROACH_BEACON_MOTOR_SPEED          = 0.40;
    double     APPROACH_1ST_BEACON_MOTOR_SPEED      = 0.35; // was 35% of max power
    double     NAVX_ANGLE_TOWARDS_2ND_BEACON        = -179;
    double     NAVX_BACKUP_DISTANCE_FROM_BEACON     = -10; // was -11.25 inches
    double     BACKUP_RETRY_2ND_WHITELINE_DISTANCE  = -10;
    double     DISTANCE_TO_AVOID_FALSE_DETECTION    = 2.0;
    double     COLOR_DETECTION_MASK_TIME            = 1.0; // unit seconds: choose a small amount of time since it is only meant to avoid false detection of the 2nd white line if robot is still near the first white line
    long       WHILE_LOOP_INTERCEPTING_WHITE_LINE   = 2;   // 2 ms
    // enable/disable telemetry data display
    boolean    displayVuforiaTelemetry              = false;
    boolean    displayDriveMotorPower               = false;
    boolean    diplayLineFollowState                = false;
    boolean    diplayAutoAimState                   = false;
    boolean    displayGyroDrive                     = false;
    boolean    displayColorReading                  = false;
    boolean    displayGyroDriveContinuous           = false;
    boolean    displayRangeSensorInfo               = false;
    boolean    displayShooterPowerPhase2            = false;
    long       TELEMETRY_UPDATE_INTERVAL            = 2;   // in milliseconds
    boolean    displayNavXTelemetryTeleOp           = false;

    // gyro
    double     GYRO_RESET_TIMEOUT                   = 1.0;

    // task timing
    long       TASK_TIME_MS                         = 2;
    boolean    CHECK_TASK_TIMING                    = false;
    boolean    MEASURE_SHOOTER_SPEED                = false;

    // vision target
    double     GEARS_Y_COORDINATE                   = -304.8; // mm
    double     TOOLS_Y_COORDINATE                   = 914.4;  // mm
    double     LEGOS_X_COORDINATE                   = -914.4; // mm
    double     WHEELS_X_COORDINATE                  = 304.8;  // mm
    double     LEGOS_WHEELS_TARGET_HEADING          = 90.0;
    double     GEARS_TOOLS_TARGET_HEADING           = 180.0;
    double     KP_SCALE_FACTOR_VISION_TARGET        = 0.6;

    // 2nd driver input of the robot's coordinate
    int        DRIVER_INPUT_X_MAX                   = 6; // unit is tile number on the field, the right most one is 6, always count relative to the alliance's driver station
    int        DRIVER_INPUT_X_MIN                   = 1; // unit is tile number on the field, the left most one is 0, always count relative to the alliance's driver station
    int        DRIVER_INPUT_Y_MAX                   = 6; // unit is tile number on the field, the top most one is 6, always count relative to the alliance's driver station
    int        DRIVER_INPUT_Y_MIN                   = 1; // unit is tile number on the field, the bottom most one is 0, always count relative to the alliance's driver station
    double     CONVERSION_OFFSET                    = 3.5; // unit is tile number on the field
    int        INCHES_PER_TILE                      = 24;
    boolean    ALLOW_DRIVER_INPUT_ROBOT_COORDINATE  = true;
    double     TIME_DISPLAY_DRIVER_INPUT_COORDINATE = 2.5;
    double     TIME_DISPLAY_DRIVER_INPUT_ACCEPTANCE = 2;
    int        NUMBER_OF_FAVORITE_COORDINATES       = 4;
    int        BLUE_TEAM_FAVORITE_COORDINATES[][]   = {{4, 1}, {4, 6}, {1, 3}, {6, 3}};
    int        RED_TEAM_FAVORITE_COORDINATES[][]    = {{3, 1}, {3, 6}, {6, 3}, {1, 3}};

    // NavX micro PID controller parameters
    double     TOLERANCE_DEGREES                    = 0.75;
    double     MIN_MOTOR_OUTPUT_VALUE               = -1.0;
    double     MAX_MOTOR_OUTPUT_VALUE               = 1.0;
    double     YAW_PID_P                            = 0.015;
    double     YAW_PID_I                            = 0.0;
    double     YAW_PID_D                            = 0.015;
    double     NAVX_CONNECTION_TIMEOUT              = 3.0;
    double     NAVX_PID_LOW_OUTPUT_THRESHOLD        = 0.03 * CargoBotConstants.SPEED_RATIO ; // 1.00 is max output
    double     MULTIPLER_WHEN_NAVX_LOW_OUTPUT       = 4.0;
    long       SLEEP_MS                             = 120; // millisecond
    int        WAIT_FOR_UPDATE_TIMEOUT_MS           = 500; // millisecond
    boolean    ZERO_YAW_FOR_TELEOP                  = false; // we don't zero yaw (reset) because we want to continue to use the zero reference from autonomous opmode.
    double     TIME_DISPLAY_NAVX_TIMEOUT_MSG        = 3.0; // unit: seconds


    // TeleOp
    double     TELEOP_TOP_SPEED                     = 0.3;
    boolean    ALLOW_ROTATE_TO_NEAREST_90           = true;
    double     CONTROL_STICK_THRESHOLD              = 0.15;
    double     TIMEOUT_WAIT_FOR_NAVX_DATA           = 0.2;  // unit is seconds
    boolean    ALLOW_ZERO_YAW_IN_PRACTICE_MODE      = true; // since the practice session can be longer than 2:30 minutes, use this flag to allow manually zero yaw.
    double     CONTROL_STICK_THRESHOLD_FOR_ZERO_YAW = 0.05;
    boolean    USE_Y_BUTTON_FOR_LOW_SPEED_MODE      = true;
    double     RED_VORTEX_ANGLE_OFFSET_DEGREES      = 270;  // unit is degrees
    double     VORTEX_ARM_RADIUS                    = 15.74 * mmPerInch;
    double     TIME_DISPLAY_VORTEX_LOCATION_CHANGE  = 2.0;  // unit is seconds

    // side beacon pressor
    boolean    SIDE_BEACON_PRESSOR_SERVO_PRESENT    = true;
    double     sideServoMin                         = 0.00;     // servo limit
    double     sideServoMax                         = 1.00;     // servo limit

    // test bot
    boolean    TEST_ROBOT_WITH_EXTRA_GEARS          = true;
}
