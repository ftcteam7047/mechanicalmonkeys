package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;

/**
 * Created by chrischen on 10/21/17.
 */

@TeleOp(name = "CargoBot Teleop Advanced")
//@Disabled


public class CargoBotTeleopAdvanced extends OpMode {

    HardwareMecanumCargoBot robot = new HardwareMecanumCargoBot();
    // NavX micro
    private final int NAVX_DIM_I2C_PORT = 5;
    private AHRS navxDevice = null;
    private navXPIDController yawTurnPIDController = null;
    private navXPIDController yawDrivePIDController = null;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    double yawKp = MMShooterBotConstants.YAW_PID_P;
    boolean isNavxDeviceConnected = false;
    boolean timeoutNavxConnection = false;
    boolean isNavxMicroDataTimeout = false;
    double timeNavxDataTestTimeout = 0.0;
    private boolean calibration_complete = false;
    double Kp = 0.005;

    // rotate to nearest 90 degrees
    RotateTo90DegreeState navxRotateState = RotateTo90DegreeState.IDLE;
    double navxRotationSetPoint = 0.0;
    boolean isRotatingToNearest90Degree = false;
    boolean isNavxRotateInitialized = false;
    boolean lastGamepad1LeftBumper = false;
    boolean lastGamepad1RightBumper = false;
    double  timeRotateToNearest90DegreeRequested = 0.0;

    private ElapsedTime period  = new ElapsedTime();

    boolean isRelicGripperButtonPressed;
    boolean isGrabberButtonPressed;
    boolean isBlockVisible;
    boolean isLifterButtonPressed;

    int target;
    int offset;

    int lastTick = 0;
    int lastEncoderPos = 0;
    double tStart = 0;
    boolean isMotorStalled = false;


    // low speed mode
    boolean isLowSpeedMode = false;
    boolean lastGamepad1YButton = false;

    float leftStickX = 0;
    float rightStickX = 0;
    float leftStickY = 0;
    float rightStickY = 0;

    private enum ENUM_GYRO_TURN {
        NOT_ARRIVED,
        ARRIVED
    }

    private enum ENUM_NAVX_GYRO_TURN {
        NOT_ARRIVED,
        ARRIVED,
        TIMEOUT
    }

    @Override
    public void init() {
        robot.init(hardwareMap);

        isRelicGripperButtonPressed = false;
        isGrabberButtonPressed = false;
        isBlockVisible = false;
        isLifterButtonPressed = false;

        // data persistence: block lift motor position
        // the file is for saving and restoring the position of the block lift if the robot is powered down
        if (robot.fileHandler.readFromFile("offset.txt", robot.context).equals("error")){
            offset = 0;
        } else {
            offset = robot.fileHandler.stringToInt(robot.fileHandler.readFromFile("offset.txt", robot.context));
        }

        // Run using encoder will force the motor to run at constant speed
        // Run at constant speed: This mode controls the speed of the motor
        // such that it maintains an average speed. If the motor is set to 85% power in this mode,
        // the controller will monitor the encoder to give an appropriate power to the motor
        // such that the motor rotates at 85% of its maximum average speed. Average is a key word here.
        // A motor set to 100% power in this mode will not rotate as fast as a motor set to 100% power in Run at constant power.
        // This is to allow the motor to catch up if it falls behind. If the motor experiences more resistance, more power will be applied to catch up to where it would have been if it never slowed down.
        // Think of this like how you actually drive.
        // Going up a hill, you will press further on the gas and going down the hill you will let up on the gas.
        // This mode requires encoders.
        // The top average speed in this mode is less than 100% in run at constant power because the motor needs to conserve the highest amounts of power to catch up if it experiences resistance.

        // Encoder reset if necessary
        // Run motor at constant speed. (see above for details)
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to NavX-micro device
        navxDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        // Create a PID Controller which uses the Yaw Angle as input. */
        yawTurnPIDController = new navXPIDController(navxDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawTurnPIDController.setContinuous(true);
        yawTurnPIDController.setOutputRange(MMShooterBotConstants.MIN_MOTOR_OUTPUT_VALUE, MMShooterBotConstants.MAX_MOTOR_OUTPUT_VALUE);
        yawTurnPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, MMShooterBotConstants.TOLERANCE_DEGREES);
        //yawTurnPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D); // let the app control this

        // Create a PID Controller which uses the Yaw Angle as input.
        yawDrivePIDController = new navXPIDController(navxDevice,
                navXPIDController.navXTimestampedDataSource.YAW);
        yawDrivePIDController.setContinuous(true);
        yawDrivePIDController.setOutputRange(MMShooterBotConstants.MIN_MOTOR_OUTPUT_VALUE, MMShooterBotConstants.MAX_MOTOR_OUTPUT_VALUE);
        yawDrivePIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, MMShooterBotConstants.TOLERANCE_DEGREES);

        initNavxMicroCalibration(MMShooterBotConstants.NAVX_CONNECTION_TIMEOUT);

        try {
            navxTestDataTimeout();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */
    @Override
    public void init_loop() {
        if (!isNavxMicroDataTimeout){
            telemetry.addData(">", "TeleOp Ready. Press Start.");
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData(">", "TeleOp Started");
        telemetry.update();
    }


    @Override
    public void loop() {
        driveController();
        rotateToNearest90DegreeController();
        //relicGripperController();
        grabberController();
        blockLiftController();
        telemetry.update();
    }


    public void driveController() {
        if (!isRotatingToNearest90Degree){
            checkForLowSpeedModeInput();
            // assign gamepad stick positions to interim variables
            // so we can do post processing on them
            leftStickX = gamepad1.left_stick_x;
            rightStickX = gamepad1.right_stick_x;
            leftStickY = gamepad1.left_stick_y;
            rightStickY = gamepad1.right_stick_y;
            if (abs(leftStickX) < CargoBotConstants.CONTROLLER_DEAD_ZONE &&
                    abs(rightStickX) < CargoBotConstants.CONTROLLER_DEAD_ZONE) {
                // Limit the top speed of the robot in low speed mode
                if (isLowSpeedMode) {
                    double dleftStickY = leftStickY;
                    double drightStickY = rightStickY;
                    leftStickY = (float) Range.clip(dleftStickY, -CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED, CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED);
                    rightStickY = (float) Range.clip(drightStickY, -CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED, CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED);
                }

                // Optional: pushing both sticks in the same direction result in same motor power on left and right
                if (CargoBotConstants.EQUALIZE_MOTOR_POWER){
                    float equalizedPower = equalizeMotorPower(leftStickY, rightStickY);
                    if (abs(equalizedPower) > 0){
                        leftStickY = equalizedPower;
                        rightStickY = equalizedPower;
                    }
                }

                // Optional: map stick y to a different curve (sinusoidal wave) to reduce jerk
                if (CargoBotConstants.REMAP_CONTROLLER_Y && !isLowSpeedMode){
                    leftStickY = mapStickY(leftStickY);
                    rightStickY = mapStickY(rightStickY);
                }

                // Optioinal: set motor zero power during tank drive
                if (CargoBotConstants.SET_MOTOR_ZERO_POWER){
                    if (!isLowSpeedMode){
                        setChassisMotorZeroBehavior(eZeroBehavior.FLOAT);
                    }
                }
                // Typical tank drive
                robot.frontLeftDrive.setPower(leftStickY);
                robot.frontRightDrive.setPower(rightStickY);
                robot.rearLeftDrive.setPower(leftStickY);
                robot.rearRightDrive.setPower(rightStickY);
            } else {
                // Optioinal: set motor zero power other than tank drive
                if (CargoBotConstants.SET_MOTOR_ZERO_POWER){
                    setChassisMotorZeroBehavior(eZeroBehavior.BRAKE);
                }

                // check if left and right sticks move in the same direction
                // and within a top-bottom band around the y-axis
                if (rightStickX * leftStickX > 0) {

                    if (abs(leftStickY) < CargoBotConstants.CONTROLLER_DEAD_ZONE &&
                            abs(rightStickY) < CargoBotConstants.CONTROLLER_DEAD_ZONE) {
                        float sideMovement = 0;
                        if (rightStickX > 0) {
                            sideMovement = max(leftStickX, rightStickX);
                        } else {
                            sideMovement = min(leftStickX, rightStickX);
                        }
                        // Limit the top speed of the robot in low speed mode
                        if (isLowSpeedMode) {
                            sideMovement = (float) Range.clip(sideMovement, -CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED, CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED);
                        }
                        robot.frontLeftDrive.setPower(-sideMovement);
                        robot.frontRightDrive.setPower(sideMovement);
                        robot.rearLeftDrive.setPower(sideMovement);
                        robot.rearRightDrive.setPower(-sideMovement);

                    } else {
                        if (CargoBotConstants.DO_DIAGONAL){
                            float diagMovement = 0;
                            float x = 0;
                            float y = 0;
                            if (rightStickX > 0) {
                                x = max(leftStickX, rightStickX);

                            } else {
                                x = min(leftStickX, rightStickX);
                            }
                            if (rightStickY > 0) {
                                y = -max(leftStickY, rightStickY);

                            } else {
                                y = -min(leftStickY, rightStickY);
                            }

                            //telemetry.addData("x", x);
                            //telemetry.addData("y", y);

                            if ((y >= (x - sqrt(2.0) * CargoBotConstants.DIAGONAL_HALF_BAND_WIDTH)) &&
                                    (y <= (x + sqrt(2.0) * CargoBotConstants.DIAGONAL_HALF_BAND_WIDTH))) {

                                diagMovement = (float) sqrt(x * x + y * y);
                                // Limit the top speed of the robot in low speed mode
                                if (isLowSpeedMode) {
                                    diagMovement = (float) Range.clip(diagMovement, -CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED, CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED);
                                }
                                if (x > 0 && y > 0) {
                                    robot.frontLeftDrive.setPower(-diagMovement);
                                    robot.frontRightDrive.setPower(0);
                                    robot.rearLeftDrive.setPower(0);
                                    robot.rearRightDrive.setPower(-diagMovement);
                                    //telemetry.addData("diag", "upper right");
                                } else if (x < 0 && y < 0) {
                                    robot.frontLeftDrive.setPower(diagMovement);
                                    robot.frontRightDrive.setPower(0);
                                    robot.rearLeftDrive.setPower(0);
                                    robot.rearRightDrive.setPower(diagMovement);
                                    //telemetry.addData("diag", "lower left");
                                }

                            } else if ((y >= (-x - sqrt(2.0) * CargoBotConstants.DIAGONAL_HALF_BAND_WIDTH)) &&
                                    (y <= (-x + sqrt(2.0) * CargoBotConstants.DIAGONAL_HALF_BAND_WIDTH))){
                                diagMovement = (float) sqrt(x * x + y * y);
                                // Limit the top speed of the robot in low speed mode
                                if (isLowSpeedMode) {
                                    diagMovement = (float) Range.clip(diagMovement, -CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED, CargoBotConstants.TELEOP_SLOW_MODE_TOP_SPEED);
                                }
                                if (x < 0 && y > 0) {
                                    robot.frontLeftDrive.setPower(0);
                                    robot.frontRightDrive.setPower(-diagMovement);
                                    robot.rearLeftDrive.setPower(-diagMovement);
                                    robot.rearRightDrive.setPower(0);
                                    //telemetry.addData("diag", "upper left");
                                } else if (x > 0 && y < 0) {
                                    robot.frontLeftDrive.setPower(0);
                                    robot.frontRightDrive.setPower(diagMovement);
                                    robot.rearLeftDrive.setPower(diagMovement);
                                    robot.rearRightDrive.setPower(0);
                                    //telemetry.addData("diag", "lower right");
                                }
                            }
                        }
                    }
                }
                //telemetry.update();
            }

        }
    }

//    public void relicGripperController() {
//        if (isRelicGripperButtonPressed) {
//            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
//                isRelicGripperButtonPressed = false;
//            }
//        }
//
//        if (gamepad1.right_bumper
//                && robot.relicGripPosition == robot.relicGripPosition.CLOSE
//                && !isRelicGripperButtonPressed) {
//            isRelicGripperButtonPressed = true;
//            robot.relicGripPosition = robot.relicGripPosition.OPEN;
//        } else if (gamepad1.left_bumper
//                && robot.relicGripPosition == robot.relicGripPosition.OPEN
//                && !isRelicGripperButtonPressed) {
//            isRelicGripperButtonPressed = true;
//            robot.relicGripPosition = robot.relicGripPosition.CLOSE;
//        }
//        // TODO: do this when ready
//        switch (robot.relicGripPosition) {
//            case CLOSE:
//                robot.relicServo.setPosition(CargoBotConstants.RELIC_ARM_CLOSE);
//                break;
//            case OPEN:
//                robot.relicServo.setPosition(CargoBotConstants.RELIC_ARM_OPEN);
//                break;
//        }
//    }

    public void grabberController() {
        if (isGrabberButtonPressed) {
            if (!gamepad1.a && !gamepad1.b) {
                isGrabberButtonPressed = false;
            }
        }

        if (gamepad1.a && !isGrabberButtonPressed) {
            if (robot.grabberPosition == robot.grabberPosition.OPEN) {
                robot.grabberPosition = robot.grabberPosition.CLOSE;
                isGrabberButtonPressed = true;
            } else if (robot.grabberPosition == robot.grabberPosition.CLOSE) {
                robot.grabberPosition = robot.grabberPosition.OPEN;
                isGrabberButtonPressed = true;
            } else if (robot.grabberPosition == robot.grabberPosition.WIDE_OPEN) {
                robot.grabberPosition = robot.grabberPosition.CLOSE;
                isGrabberButtonPressed = true;
            }
        }

        if (gamepad1.b && !isBlockVisible) {
            robot.grabberPosition = robot.grabberPosition.WIDE_OPEN;
        }

        switch (robot.grabberPosition) {
            case OPEN:
                robot.lowerLeftServo.setPosition(CargoBotConstants.LEFT_OPEN);
                robot.lowerRightServo.setPosition(CargoBotConstants.RIGHT_OPEN);
                robot.upperLeftServo.setPosition(CargoBotConstants.LEFT_OPEN);
                robot.upperRightServo.setPosition(CargoBotConstants.RIGHT_OPEN);
                break;
            case CLOSE:
                robot.lowerLeftServo.setPosition(CargoBotConstants.LEFT_CLOSE);
                robot.lowerRightServo.setPosition(CargoBotConstants.RIGHT_CLOSE);
                robot.upperLeftServo.setPosition(CargoBotConstants.LEFT_CLOSE);
                robot.upperRightServo.setPosition(CargoBotConstants.RIGHT_CLOSE);
                break;
            case WIDE_OPEN:
                robot.lowerLeftServo.setPosition(CargoBotConstants.LEFT_WIDE_OPEN);
                robot.lowerRightServo.setPosition(CargoBotConstants.RIGHT_WIDE_OPEN);
                robot.upperLeftServo.setPosition(CargoBotConstants.LEFT_WIDE_OPEN);
                robot.upperRightServo.setPosition(CargoBotConstants.RIGHT_WIDE_OPEN);
                break;
        }
    }

    public void blockLiftController() {
        // TODO: implement stall detection/timeout near GRAB(bottom) and PLACE (top) positions
        // initialization, considering offset
        if (robot.liftPosition == robot.liftPosition.INIT_POSITION){
            // first determine lift position, then determine the target position depending on dpad up/down
            int grab = (int) CargoBotConstants.GRAB_DISTANCE_FROM_START;
            int move = (int) CargoBotConstants.MOVE_DISTANCE_FROM_START;
            int stack = (int) CargoBotConstants.STACK_DISTANCE_FROM_START;
            int place = (int) CargoBotConstants.PLACE_DISTANCE_FROM_START;

            if (offset < grab){
                // only accept dpad up
                if (gamepad1.dpad_up){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.GRAB;
                        isLifterButtonPressed = true;
                    }
                }
            } else if (offset >= grab && offset < move){
                // can accept dpad up or down
                if (gamepad1.dpad_up){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.MOVE;
                        isLifterButtonPressed = true;
                    }
                } else if (gamepad1.dpad_down){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.GRAB;
                        isLifterButtonPressed = true;
                    }
                }

            } else if (offset >= move && offset < stack) {
                // can accept dpad up or down
                if (gamepad1.dpad_up){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.STACK;
                        isLifterButtonPressed = true;
                    }
                } else if (gamepad1.dpad_down){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.MOVE;
                        isLifterButtonPressed = true;
                    }
                }

            } else if (offset >= stack && offset < place) {
                // can accept dpad up or down
                if (gamepad1.dpad_up){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.PLACE;
                        isLifterButtonPressed = true;
                    }
                } else if (gamepad1.dpad_down){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.STACK;
                        isLifterButtonPressed = true;
                    }
                }

            } else {
                // offset >= place, only accept dpad down
                if (gamepad1.dpad_down){
                    if (!isLifterButtonPressed){
                        robot.liftPosition = robot.liftPosition.PLACE;
                        isLifterButtonPressed = true;
                    }
                }
            }
        }
        // normal operation
        if (gamepad1.dpad_up) {
            if (robot.liftPosition == robot.liftPosition.GRAB && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.MOVE;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to MOVE");
            } else if (robot.liftPosition == robot.liftPosition.MOVE && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.STACK;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to STACK");
            } else if (robot.liftPosition == robot.liftPosition.STACK && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.PLACE;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to PLACE");
            }
            isMotorStalled = false;
        } else if (gamepad1.dpad_down) {
            if (robot.liftPosition == robot.liftPosition.PLACE && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.STACK;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to STACK");
            } else if (robot.liftPosition == robot.liftPosition.STACK && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.MOVE;
                telemetry.addData("lift status", "to MOVE");
                isLifterButtonPressed = true;
            } else if (robot.liftPosition == robot.liftPosition.MOVE && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.GRAB;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to GRAB");
            }
            isMotorStalled = false;
        }

        switch (robot.liftPosition) {
            case GRAB:
                target = (int) CargoBotConstants.GRAB_DISTANCE_FROM_START - offset;
                break;
            case MOVE:
                target = (int) CargoBotConstants.MOVE_DISTANCE_FROM_START - offset;
                break;
            case STACK:
                target = (int) CargoBotConstants.STACK_DISTANCE_FROM_START - offset;
                break;
            case PLACE:
                target = (int) CargoBotConstants.PLACE_DISTANCE_FROM_START - offset;
                break;
            case INIT_POSITION:
                // robot is just initialized, don't move the lifter without user input
                break;
        }

        // don't move the lifter without user input
        if (robot.liftPosition != HardwareMecanumCargoBot.LiftPosition.INIT_POSITION){
            if (!isMotorStalled) {
                robot.blockLift.setTargetPosition(target);
                robot.blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (CargoBotConstants.SET_LIFT_MOTOR_HIGH_SPEED){
                    // always lift up/down at high speed
                    robot.blockLift.setPower(CargoBotConstants.LIFT_HI_SPEED);
                } else {
                    // only STACK and PLACE get high speed
                    if ((robot.liftPosition == HardwareMecanumCargoBot.LiftPosition.STACK) ||
                            (robot.liftPosition == HardwareMecanumCargoBot.LiftPosition.PLACE)){
                        robot.blockLift.setPower(CargoBotConstants.LIFT_HI_SPEED);
                    } else{
                        robot.blockLift.setPower(CargoBotConstants.LIFT_SPEED);
                    }
                }


            }
            //telemetry.addData("lift encoder pos", robot.blockLift.getCurrentPosition());
            //telemetry.update();
            if (!robot.blockLift.isBusy()) {
                turnOffLiftMotor();
                reportLiftLogicalPos();
            } else {
                detectNeverRest60MotorStall();
            }
        }

        if (isLifterButtonPressed) {
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                isLifterButtonPressed = false;
            }
        }
    }

    private void initNavxMicroCalibration(double timeoutS) {
        // if the connection to navx hardware is good
        double startTime = getRuntime();
        double elapsedTime = 0.0;

        while (!navxDevice.isConnected() && !timeoutNavxConnection) {
            // wait for device to be connected or time out
            elapsedTime = getRuntime() - startTime;
            if (elapsedTime >= timeoutS) {
                timeoutNavxConnection = true;
            }
            telemetry.addData("navX-Micro", "Connecting");
            telemetry.update();
        }
        if (!timeoutNavxConnection) {
            isNavxDeviceConnected = true;
        }
        if (isNavxDeviceConnected) {
            while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
                calibration_complete = !navxDevice.isCalibrating();
                if (!calibration_complete) {
                    telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                    telemetry.update();
                }
            }
            telemetry.addData("navX-Micro", "Done Calibration");
            telemetry.update();

            if (MMShooterBotConstants.ZERO_YAW_FOR_TELEOP) {
                navxDevice.zeroYaw();
            }
        } else {
            telemetry.addData("navX-Micro", "Check Hardware Connection");
            telemetry.update();
        }
    }

    private void navxTestDataTimeout() throws InterruptedException {
        double angle = 0;
        // set the parameters before enabling the PID controller
        yawTurnPIDController.setSetpoint(angle);
        yawTurnPIDController.setPID(Kp, MMShooterBotConstants.YAW_PID_I, MMShooterBotConstants.YAW_PID_D);
        yawTurnPIDController.enable(true);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        Thread.sleep(MMShooterBotConstants.SLEEP_MS);
        // this instruction blocks the thread, and won't return immediately
        // return true if new data is available
        // return false if it times out.
        if (yawTurnPIDController.waitForNewUpdate(yawPIDResult, MMShooterBotConstants.WAIT_FOR_UPDATE_TIMEOUT_MS)) {
            // no issues, data is available
            isNavxMicroDataTimeout = false;
        } else {
            // data timeout occurs
            isNavxMicroDataTimeout = true;
            timeNavxDataTestTimeout = getRuntime();
        }

        // navx data timeout, warn the driver to stop the app, and re-activate the configuration. Then re-start the app.
        while ((isNavxMicroDataTimeout) && ((getRuntime() - timeNavxDataTestTimeout) < MMShooterBotConstants.TIME_DISPLAY_NAVX_TIMEOUT_MSG)) {
            telemetry.addData(">", "Stop the app and exit. Cycle robot power. Then re-start.");    //
            telemetry.update();
        }

    }

    private enum RotateTo90DegreeState {
        IDLE,
        ROTATE_CCW,
        ROTATE_CW
    }

    private void rotateToNearest90DegreeController() {
        if (isNavxDeviceConnected) {
            switch (navxRotateState) {
                case IDLE:
                    // clear the flag so the "rotate-to-angle" method can accept new request
                    isNavxRotateInitialized = false;
                    // don't accept the command if the driver is still controlling the left/right stick (with some threshold)
                    if ((Math.abs(gamepad1.left_stick_y) <= MMShooterBotConstants.CONTROL_STICK_THRESHOLD) &&
                            (Math.abs(gamepad1.right_stick_y) <= MMShooterBotConstants.CONTROL_STICK_THRESHOLD)) {
                        if ((gamepad1.left_bumper) && !lastGamepad1LeftBumper) {
                            navxRotationSetPoint = calculateSetPointRotateToNearest90CCW();
                            navxRotateState = RotateTo90DegreeState.ROTATE_CCW;
                        } else if ((gamepad1.right_bumper) && !lastGamepad1RightBumper) {
                            navxRotationSetPoint = calculateSetPointRotateToNearest90CW();
                            navxRotateState = RotateTo90DegreeState.ROTATE_CW;
                        }
                    }
                    break;

                case ROTATE_CCW:
                    // abort if the button is pressed again
                    if ((gamepad1.left_bumper) && !lastGamepad1LeftBumper) {
                        turnOffDriveMotors();
                        navxRotateState = RotateTo90DegreeState.IDLE;
                    } else {
                        // keep rotating until it arrives at the target, or timeout wating for navx data
                        if ((navxRotateToAngle(navxRotationSetPoint, yawKp) == ENUM_NAVX_GYRO_TURN.ARRIVED) ||
                                (navxRotateToAngle(navxRotationSetPoint, yawKp) == ENUM_NAVX_GYRO_TURN.TIMEOUT)) {
                            navxRotateState = RotateTo90DegreeState.IDLE;
                        }
                    }
                    break;

                case ROTATE_CW:
                    // abort if the button is pressed again
                    if ((gamepad1.right_bumper) && !lastGamepad1RightBumper) {
                        turnOffDriveMotors();
                        navxRotateState = RotateTo90DegreeState.IDLE;
                    } else {
                        // keep rotating until it arrives at the target, or timeout wating for navx data
                        if ((navxRotateToAngle(navxRotationSetPoint, yawKp) == ENUM_NAVX_GYRO_TURN.ARRIVED) ||
                                (navxRotateToAngle(navxRotationSetPoint, yawKp) == ENUM_NAVX_GYRO_TURN.TIMEOUT)) {
                            navxRotateState = RotateTo90DegreeState.IDLE;
                        }
                    }
                    break;

                default:
                    turnOffDriveMotors();
                    navxRotateState = RotateTo90DegreeState.IDLE;
                    break;
            }

            // store button state for comparison in the next loop
            lastGamepad1LeftBumper = gamepad1.left_bumper;
            lastGamepad1RightBumper = gamepad1.right_bumper;


        } else {
            navxRotateState = RotateTo90DegreeState.IDLE;
        }
        // let other motor control program know we are in the process of rotation, to avoid conflicts in motor controls
        if ((navxRotateState == navxRotateState.ROTATE_CCW) ||
                (navxRotateState == navxRotateState.ROTATE_CW)) {
            isRotatingToNearest90Degree = true;
        } else {
            isRotatingToNearest90Degree = false;
        }
    }

    public void turnOffDriveMotors() {
        // Stop all motion;
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnOffLiftMotor(){
        robot.blockLift.setPower(0.0);
        robot.blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private ENUM_NAVX_GYRO_TURN navxRotateToAngle(double angle, double Kp) {
        ENUM_NAVX_GYRO_TURN retValue = ENUM_NAVX_GYRO_TURN.NOT_ARRIVED;
        double angleNormalized = -angle; // reverse the angle's direction: since positive is for CCW, negative is for CW

        if (!isNavxRotateInitialized) {
            // set the parameters before enabling the PID controller
            yawTurnPIDController.setSetpoint(angleNormalized);
            yawTurnPIDController.setPID(Kp, MMShooterBotConstants.YAW_PID_I, MMShooterBotConstants.YAW_PID_D);
            yawTurnPIDController.enable(true);
            timeRotateToNearest90DegreeRequested = getRuntime();
            // set to true so that init only runs once per request
            isNavxRotateInitialized = true;
        }

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        DecimalFormat df = new DecimalFormat("#.##");

        if (yawTurnPIDController.isNewUpdateAvailable(yawPIDResult)) {

            if (yawPIDResult.isOnTarget()) {
                setChassisMotorZeroBehavior(eZeroBehavior.BRAKE);
                turnOffDriveMotors();
                retValue = ENUM_NAVX_GYRO_TURN.ARRIVED;
                if (MMShooterBotConstants.displayNavXTelemetryTeleOp) {
                    telemetry.addData("PIDOutput", df.format(0.00));
                }
            } else {
                double output = yawPIDResult.getOutput();

                // amplify the output of PID controller as it gets close to the target
                // otherwise, it would not finish turning in time
                if (Math.abs(output) <= MMShooterBotConstants.NAVX_PID_LOW_OUTPUT_THRESHOLD) {
                    output = output * MMShooterBotConstants.MULTIPLER_WHEN_NAVX_LOW_OUTPUT;
                }

                robot.frontLeftDrive.setPower(-output);
                robot.frontRightDrive.setPower(output);
                robot.rearLeftDrive.setPower(-output);
                robot.rearRightDrive.setPower(output);
                if (MMShooterBotConstants.displayNavXTelemetryTeleOp) {
                    telemetry.addData("PIDOutput", df.format(output) + ", " +
                            df.format(-output));
                }
            }

            if (MMShooterBotConstants.displayNavXTelemetryTeleOp) {
                telemetry.addData("Yaw", df.format(-navxDevice.getYaw())); // add negative to convert to robot's heading
            }
        } else {
            // check for timeout waiting for navx data to update
            // note: this is not a timeout for the rotation, but for data availability.
            if ((getRuntime() - timeRotateToNearest90DegreeRequested) >= MMShooterBotConstants.TIMEOUT_WAIT_FOR_NAVX_DATA) {
                // give up and return time out status
                retValue = ENUM_NAVX_GYRO_TURN.TIMEOUT;
            }

        }
        return retValue;
    }

    private double calculateSetPointRotateToNearest90CW() {

        double setPoint = 0;
        // determine what angle is the nearest 90 in the CW direction
        double currentHeading = -navxDevice.getYaw(); // adding negative to the value so that for the robot, CCW is positive, CW is negative.

        if (currentHeading >= 0) {
            if (currentHeading < 90) {
                // heading is 0 to 90
                setPoint = 0;

            } else if ((currentHeading >= 90) && (currentHeading < 180)) {
                // 90 <= heading < 180
                setPoint = 90;
            }
        } else {
            if (currentHeading > -90) {
                // heading is 0 to -90
                setPoint = -90;

            } else if ((currentHeading <= -90) && (currentHeading > -180)) {
                // -180 < heading <= -90
                setPoint = -179.9;
            }
        }
        return setPoint;
    }

    private double calculateSetPointRotateToNearest90CCW() {

        double setPoint = 0;
        // determine what angle is the nearest 90 in the CCW direction
        double currentHeading = -navxDevice.getYaw(); // adding negative to the value so that for the robot, CCW is positive, CW is negative.

        if (currentHeading > 0) {
            if (currentHeading <= 90) {
                //  0 < heading <= 90
                setPoint = 90;

            } else if ((currentHeading > 90) && (currentHeading <= 180)) {
                // 90 < heading <= 180
                setPoint = 179.9;
            }
        } else {
            if (currentHeading > -90) {
                // -90 < heading <= 0
                setPoint = 0;

            } else if ((currentHeading <= -90) && (currentHeading > -180)) {
                // -180 < heading <= -90
                setPoint = -90;
            }
        }
        return setPoint;
    }

    private void checkForLowSpeedModeInput() {

        // determine we should use y button or right stick button to toggle low speed mode
        if (CargoBotConstants.USE_Y_BUTTON_FOR_LOW_SPEED_MODE){
            if ((gamepad1.y) && !lastGamepad1YButton) {
                isLowSpeedMode = !isLowSpeedMode;
            }
            // store the current state for comparison in the next loop
            lastGamepad1YButton = gamepad1.y;
        }
        // tell the driver with the current speed mode
        if (isLowSpeedMode) {
            telemetry.addData("Speed", "Low");
        } else {
            telemetry.addData("Speed", "Normal");
        }
    }

    private void reportLiftLogicalPos(){
        switch (robot.liftPosition){
            case GRAB:
                telemetry.addData("lift status", "@ GRAB");
                break;
            case MOVE:
                telemetry.addData("lift status", "@ MOVE");
                break;
            case STACK:
                telemetry.addData("lift status", "@ STACK");
                break;
            case PLACE:
                telemetry.addData("lift status", "@ PLACE");
                break;
            case INIT_POSITION:
                telemetry.addData("lift status", "@ INIT_POSITION");
                break;
        }
    }

    private int getRuntimeInTicks(double tStart, double tick){
        double runtime = getRuntime() - tStart;

        Double numTicks = runtime/tick;
        numTicks.intValue();

        return numTicks.intValue();
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private void detectNeverRest60MotorStall(){
        // create time base for checking if encoder count is not advancing much, which implies motor stall
        int tick = 0;
        // for every new request, set the last encoder position to the current position
        // as the basis for comparison
        // clear the motor stall flag to allow movement to other positions
        if (isLifterButtonPressed) {
            lastEncoderPos = robot.blockLift.getCurrentPosition();
            tStart = getRuntime();
            lastTick = getRuntimeInTicks(tStart, CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD);
            //isMotorStalled = false;
        }

        tick = getRuntimeInTicks(tStart, CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD);
        if (tick > lastTick) {
            // update for comparison in the next loop
            lastTick = tick;
            double time = CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD;
            int motorStallThreshold = (int) (getNeverest60MotorEncoderCountWithTime(time, CargoBotConstants.LIFT_SPEED) * CargoBotConstants.MOTOR_STALL_RATIO);
//            telemetry.addData("last encoder", lastEncoderPos);
//            telemetry.addData("current pos", robot.blockLift.getCurrentPosition());
//            telemetry.addData("delta pos", abs(lastEncoderPos - robot.blockLift.getCurrentPosition()));
//            telemetry.addData("motor stall thres", motorStallThreshold);

            // check the difference in encoder count at every tick
            if (abs(lastEncoderPos - robot.blockLift.getCurrentPosition()) < motorStallThreshold) {
                turnOffLiftMotor();
                isMotorStalled = true;
            }
            // update for comparison in the next loop
            lastEncoderPos = robot.blockLift.getCurrentPosition();

        }
    }

    /*
     * Calculate the theoretical encoder counts/pulses per tick using AndyMark's data
     * https://www.andymark.com/NeveRest-60-Gearmotor-p/am-3103.htm
     * Data is measured at Voltage: 12 volt DC
     * No Load Free Speed, at gearbox output shaft: 105 RPM
     */
    private int getNeverest60MotorEncoderCountWithTime(double tick, double motorPower) {
        // limit the motorPower to [-1.0, 1.0]
        motorPower = Range.clip(motorPower, -CargoBotConstants.MOTOR_TOP_SPEED, CargoBotConstants.MOTOR_TOP_SPEED);
        int maxEncoderPulsesPerSec = (int) Math.round(motorPower * CargoBotConstants.ANDYMARK_60_MAX_COUNT_PER_SEC * getBatteryVoltage() / CargoBotConstants.MOTOR_VOLTAGE);
        int maxEncoderPulsesPerTick = (int) Math.round(maxEncoderPulsesPerSec * tick);

        return maxEncoderPulsesPerTick;
    }

    private float mapStickY(float input){
        float output = 0;
        if (input > 0){
            float theta = (float) (input * Math.PI + CargoBotConstants.PI_MULTIPLIER * Math.PI);
            float cosineWave = (float) (Math.cos(theta));
            float scale = 0.5f;
            float offset = 1.0f;
            output = (cosineWave + offset) * scale;
        } else {
            input = abs(input);
            float theta = (float) (input * Math.PI + CargoBotConstants.PI_MULTIPLIER * Math.PI);
            float cosineWave = (float) (Math.cos(theta));
            float scale = 0.5f;
            float offset = 1.0f;
            output = -(cosineWave + offset) * scale;
        }

        return output;
    }

    private float equalizeMotorPower(float left, float right){
        // both left and right are positive or negative
        float output = 0;
        if ((left * right) > 0) {
            if (left > 0){
                output = max(left, right);
            } else {
                output = min(left, right);
            }
        }
        return output;
    }
    enum eZeroBehavior{
        BRAKE,
        FLOAT
    }
    private void setChassisMotorZeroBehavior(eZeroBehavior mode){
        if (mode == eZeroBehavior.BRAKE){
            robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (mode == eZeroBehavior.FLOAT){
            robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        navxDevice.close();
        // stop all dc motors and return the servo motor to its neutral position
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        // TODO: set other motor power to zero
        robot.blockLift.setPower(0);
        // save the block lift position for next operation
        robot.fileHandler.writeToFile("offset.txt", Integer.toString(offset + robot.blockLift.getCurrentPosition()), robot.context);
    }

}
