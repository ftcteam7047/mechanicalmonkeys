package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;

/**
 * Created by chrischen on 11/29/17.
 * Test program to control 4 DC motors
 * 1 DC motor for primary intake: motor controller AH00QHU7, 1 = front
 * 2 DC motors for secondary intake: motor controller AL00VVM3, 1 = left, 2 = right
 * 1 DC motor for lift ( timing belt axle diameter = 0.465 inch)
 * 1 MakeBlock servo for rotating the ramp/placing the block
 */

@TeleOp(name = "MM Version2 Test 2")
//@Disabled

public class MMVersion2Test2 extends OpMode {

    Servo testServo;
    //
    DcMotor frontIntakeMotor;
    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;
    DcMotor blockLift;

    MMFileHandler fileHandler = new MMFileHandler();
    Context context;

    // servo variables
    double targetPos = 0.0;
    boolean lastGamepad1A = false;
    boolean lastGamepad1B = false;
    boolean lastGamepad1X = false;

    double currentServoPos = 0.0;
    boolean aIsPressed = false;
    boolean bIsPressed = false;
    boolean xIsPressed = false;

    // servo parameters
    double downTarget = 0.82;
    double upTarget = 0.0;
    double flatTarget = 0.62;
    double timeIncrement = 0.025;
    double posIncrement = 0.02;

    // dcMotor parameters
    double frontIntakeMotorPower = 0.5;
    double leftIntakeMotorPower = 0.85;
    double rightIntakeMotorPower = 0.85;
    double lrIntakeMotorDeltaPower = 0.1;

    enum intakeDir {
        NORMAL,
        REVERSE
    }

    private enum RotateTo90DegreeState {
        IDLE,
        ROTATE_CCW,
        ROTATE_CW
    }

    // rotate to nearest 90 degrees
    RotateTo90DegreeState navxRotateState = RotateTo90DegreeState.IDLE;
    double navxRotationSetPoint = 0.0;
    boolean isRotatingToNearest90Degree = false;
    boolean isNavxRotateInitialized = false;
    boolean lastGamepad1LeftBumper = false;
    boolean lastGamepad1RightBumper = false;
    double  timeRotateToNearest90DegreeRequested = 0.0;

    // low speed mode
    boolean isLowSpeedMode = false;
    boolean lastGamepad1YButton = false;

    float leftStickX = 0;
    float rightStickX = 0;
    float leftStickY = 0;
    float rightStickY = 0;

    // enum for block lift
    public enum LiftPosition {
        LOW,
        HIGH,
        INIT_POSITION
    }

    // lift motor variables
    LiftPosition liftPosition;
    boolean isLifterButtonPressed;
    int offset;
    boolean isMotorStalled = false;
    int target;
    int lastTick = 0;
    int lastMotorStallTick = 0;
    double tStart = 0;
    double tMotorStart = 0;
    int lastEncoderPos = 0;

    // for time interval
    private ElapsedTime period  = new ElapsedTime();

    @Override
    public void init() {
        context = hardwareMap.appContext;
        testServo = hardwareMap.servo.get("testServo");
        testServo.setPosition(downTarget);
        frontIntakeMotor = hardwareMap.dcMotor.get("frontIntakeMotor");
        leftIntakeMotor = hardwareMap.dcMotor.get("leftIntakeMotor");
        rightIntakeMotor = hardwareMap.dcMotor.get("rightIntakeMotor");
        // set intake motor direction
        setIntakeMotorDir(intakeDir.NORMAL);
        // reset motor encoder
        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // maintain constant speed
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initLiftMotor();
        initDriveMotors();
    }

    @Override
    public void loop() {
        servoController();
        intakeController();
        driveController();
        blockLiftController();
        try {
            waitForTick(2); // every 2ms, sample the encoder
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData(">", "TeleOp Started");
        telemetry.update();
        tStart = getRuntime();
    }

    private void initDriveMotors(){
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        rearLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        rearRightDrive = hardwareMap.dcMotor.get("rearRightDrive");

        // Set motor initial direction
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors Neverest 40
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors Neverest 40
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors Neverest 40
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors Neverest 40

        // Encoder reset if necessary
        // Run motor at constant speed. (see above for details)
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void intakeController() {
        if (gamepad2.a) {
            // normal intake
            setIntakeMotorDir(intakeDir.NORMAL);
            activateIntakeMotors();
        } else if (gamepad2.y) {
            // reverse intake
            setIntakeMotorDir(intakeDir.REVERSE);
            activateIntakeMotors();
        } else {
            turnOffIntakeMotors();
        }

    }

    private void servoController(){
        if (gamepad1.a && !lastGamepad1A) {
            // open
            aIsPressed = true;
            bIsPressed = false;
            xIsPressed = false;
            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1A = gamepad1.a;

        if (gamepad1.b && !lastGamepad1B) {
            // close
            aIsPressed = false;
            bIsPressed = true;
            xIsPressed = false;
            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1B = gamepad1.b;

        if (gamepad1.x && !lastGamepad1X) {
            // flat
            aIsPressed = false;
            bIsPressed = false;
            xIsPressed = true;
            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1X = gamepad1.x;


        if (aIsPressed){
            moveToDown();
        } else if (bIsPressed){
            moveToUp();
        } else if (xIsPressed){
            moveToFlat();
        }
    }

    private void activateIntakeMotors(){
        // activate primary intake motor
        frontIntakeMotor.setPower(frontIntakeMotorPower);
        // use dpad left / right to add power to left or right intake motor
        if (gamepad1.dpad_left) {
            leftIntakeMotor.setPower(leftIntakeMotorPower + lrIntakeMotorDeltaPower);
            rightIntakeMotor.setPower(rightIntakeMotorPower);
        } else if (gamepad1.dpad_right) {
            leftIntakeMotor.setPower(leftIntakeMotorPower);
            rightIntakeMotor.setPower(rightIntakeMotorPower + lrIntakeMotorDeltaPower);
        } else {
            // activate equal power on both left and right intake motors
            leftIntakeMotor.setPower(leftIntakeMotorPower);
            rightIntakeMotor.setPower(rightIntakeMotorPower);
        }
    }

    private void turnOffIntakeMotors(){
        frontIntakeMotor.setPower(0);
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        // make sure intake motors maintain constant speed
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnOffDriveMotors(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        // make sure the drive motors maintain constant speed
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setIntakeMotorDir(intakeDir dir){
        if (dir == intakeDir.NORMAL){
            frontIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
            leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
            rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            frontIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
            leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
            rightIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

    }

    private int getRuntimeInTicks(double startTime, double tick){
        double runtime = getRuntime() - startTime;

        Double numTicks = runtime/tick;
        return numTicks.intValue();
    }

    private void moveToDown(){
        int tick = 0;
        tick = getRuntimeInTicks(tStart, timeIncrement);

        if (tick > lastTick) {
            int deltaTick = tick - lastTick;
            lastTick = tick;
            targetPos = testServo.getPosition() + deltaTick * posIncrement;
            if (targetPos >= downTarget) {
                targetPos = downTarget;
            }
            testServo.setPosition(targetPos);
        }
        if (downTarget == testServo.getPosition()){
            aIsPressed = false;
        }
    }
    private void moveToUp(){
        int tick = 0;
        tick = getRuntimeInTicks(tStart, timeIncrement);

        if (tick > lastTick) {
            int deltaTick = tick - lastTick;
            lastTick = tick;
            targetPos = testServo.getPosition() - deltaTick * posIncrement;
            if (targetPos <= upTarget) {
                targetPos = upTarget;
            }
            testServo.setPosition(targetPos);
        }
        if (upTarget == testServo.getPosition()){
            bIsPressed = false;
        }
    }

    private void moveToFlat(){
        int tick = 0;
        tick = getRuntimeInTicks(tStart, timeIncrement);

        if (tick > lastTick) {
            int deltaTick = tick - lastTick;
            lastTick = tick;
            currentServoPos = testServo.getPosition();
            if ((currentServoPos - deltaTick * posIncrement) > flatTarget){
                targetPos = currentServoPos - deltaTick * posIncrement;
            } else if ((currentServoPos + deltaTick * posIncrement) < flatTarget){
                targetPos = currentServoPos + deltaTick * posIncrement;
            } else {
                targetPos = flatTarget;
            }

            testServo.setPosition(targetPos);
        }
        if (flatTarget == testServo.getPosition()){
            xIsPressed = false;
        }
    }

    private void checkForLowSpeedModeInput() {
        if ((gamepad1.y) && !lastGamepad1YButton) {
            isLowSpeedMode = !isLowSpeedMode;
        }
        // store the current state for comparison in the next loop
        lastGamepad1YButton = gamepad1.y;

        // tell the driver with the current speed mode
        if (isLowSpeedMode) {
            telemetry.addData("Speed", "Low");
        } else {
            telemetry.addData("Speed", "Normal");
        }
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
                frontLeftDrive.setPower(leftStickY);
                frontRightDrive.setPower(rightStickY);
                rearLeftDrive.setPower(leftStickY);
                rearRightDrive.setPower(rightStickY);
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
                        frontLeftDrive.setPower(-sideMovement);
                        frontRightDrive.setPower(sideMovement);
                        rearLeftDrive.setPower(sideMovement);
                        rearRightDrive.setPower(-sideMovement);

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
                                    frontLeftDrive.setPower(-diagMovement);
                                    frontRightDrive.setPower(0);
                                    rearLeftDrive.setPower(0);
                                    rearRightDrive.setPower(-diagMovement);
                                    //telemetry.addData("diag", "upper right");
                                } else if (x < 0 && y < 0) {
                                    frontLeftDrive.setPower(diagMovement);
                                    frontRightDrive.setPower(0);
                                    rearLeftDrive.setPower(0);
                                    rearRightDrive.setPower(diagMovement);
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
                                    frontLeftDrive.setPower(0);
                                    frontRightDrive.setPower(-diagMovement);
                                    rearLeftDrive.setPower(-diagMovement);
                                    rearRightDrive.setPower(0);
                                    //telemetry.addData("diag", "upper left");
                                } else if (x > 0 && y < 0) {
                                    frontLeftDrive.setPower(0);
                                    frontRightDrive.setPower(diagMovement);
                                    rearLeftDrive.setPower(diagMovement);
                                    rearRightDrive.setPower(0);
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
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (mode == eZeroBehavior.FLOAT){
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void blockLiftController() {
        // initialization, considering offset
        if (liftPosition == liftPosition.INIT_POSITION) {
            // first determine lift position, then determine the target position depending on dpad up/down
            int low = (int) CargoBotConstants.LOW_DISTANCE_FROM_START;
            int high = (int) CargoBotConstants.HIGH_DISTANCE_FROM_START;

            if (offset < low) {
                // only accept dpad up
                if (gamepad1.dpad_up) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.LOW;
                        isLifterButtonPressed = true;
                    }
                }
            } else if (offset >= low && offset < high) {
                // can accept dpad up or down
                if (gamepad1.dpad_up) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.HIGH;
                        isLifterButtonPressed = true;
                    }
                } else if (gamepad1.dpad_down) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.LOW;
                        isLifterButtonPressed = true;
                    }
                }

            } else {
                // offset >= high, only accept dpad down
                if (gamepad1.dpad_down) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.HIGH;
                        isLifterButtonPressed = true;
                    }
                }
            }
        }
        // normal operation
        if (gamepad1.dpad_up) {
            if (liftPosition == liftPosition.LOW && !isLifterButtonPressed) {
                liftPosition = liftPosition.HIGH;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to HIGH");
            }
            isMotorStalled = false;
        } else if (gamepad1.dpad_down) {
            if (liftPosition == liftPosition.HIGH && !isLifterButtonPressed) {
                liftPosition = liftPosition.LOW;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to LOW");
            }
            isMotorStalled = false;
        }

        switch (liftPosition) {
            case LOW:
                target = (int) CargoBotConstants.LOW_DISTANCE_FROM_START - offset;
                break;
            case HIGH:
                target = (int) CargoBotConstants.HIGH_DISTANCE_FROM_START - offset;
                break;
            case INIT_POSITION:
                // robot is just initialized, don't move the lifter without user input
                break;
        }

        // don't move the lifter without user input
        if (liftPosition != LiftPosition.INIT_POSITION) {
            if (!isMotorStalled) {
                blockLift.setTargetPosition(target);
                blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (CargoBotConstants.SET_LIFT_MOTOR_HIGH_SPEED) {
                    // always lift up/down at high speed
                    blockLift.setPower(CargoBotConstants.LIFT_HI_SPEED);
                } else {
                    blockLift.setPower(CargoBotConstants.LIFT_SPEED);
                }
            }

            //telemetry.addData("lift encoder pos", blockLift.getCurrentPosition());
            //telemetry.update();
            if (!blockLift.isBusy()) {
                turnOffLiftMotor();
                reportLiftLogicalPos();
            } else {
                detectNeverRest20MotorStall();
            }
        }

        if (isLifterButtonPressed) {
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                isLifterButtonPressed = false;
            }
        }
    }

    private void turnOffLiftMotor() {
        blockLift.setPower(0.0);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void reportLiftLogicalPos() {
        switch (liftPosition) {
            case LOW:
                telemetry.addData("lift status", "@ LOW");
                break;
            case HIGH:
                telemetry.addData("lift status", "@ HIGH");
                break;
            case INIT_POSITION:
                telemetry.addData("lift status", "@ INIT_POSITION");
                break;
        }
    }

    /*
 * Calculate the theoretical encoder counts/pulses per tick using AndyMark's data
 * http://www.andymark.com/NeveRest-20-12V-Gearmotor-p/am-3102.htm
 * Data is measured at Voltage: 12 volt DC
 * No Load Free Speed, at gearbox output shaft: 275-315 RPM
 */
    private int getNeverest20MotorEncoderCountWithTime(double tick, double motorPower) {
        // limit the motorPower to [-1.0, 1.0]
        motorPower = Range.clip(motorPower, -CargoBotConstants.MOTOR_TOP_SPEED, CargoBotConstants.MOTOR_TOP_SPEED);
        int maxEncoderPulsesPerSec = (int) Math.round(motorPower * CargoBotConstants.ANDYMARK_20_MAX_COUNT_PER_SEC * getBatteryVoltage() / CargoBotConstants.MOTOR_VOLTAGE);
        int maxEncoderPulsesPerTick = (int) Math.round(maxEncoderPulsesPerSec * tick);

        return maxEncoderPulsesPerTick;
    }

    private void detectNeverRest20MotorStall() {
        // create time base for checking if encoder count is not advancing much, which implies motor stall
        int tick = 0;
        // for every new request, set the last encoder position to the current position
        // as the basis for comparison
        // clear the motor stall flag to allow movement to other positions
        if (isLifterButtonPressed) {
            lastEncoderPos = blockLift.getCurrentPosition();
            tMotorStart = getRuntime();
            lastMotorStallTick = getRuntimeInTicks(tMotorStart, CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD);
        }

        tick = getRuntimeInTicks(tMotorStart, CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD);
        if (tick > lastMotorStallTick) {
            // update for comparison in the next loop
            lastMotorStallTick = tick;
            double time = CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD;
            double motorSpeed;
            if (CargoBotConstants.SET_LIFT_MOTOR_HIGH_SPEED) {
                motorSpeed = CargoBotConstants.LIFT_HI_SPEED;
            } else {
                motorSpeed = CargoBotConstants.LIFT_SPEED;
            }
            int motorStallThreshold = (int) (getNeverest20MotorEncoderCountWithTime(time, motorSpeed) * CargoBotConstants.MOTOR_STALL_RATIO);
//            telemetry.addData("last encoder", lastEncoderPos);
//            telemetry.addData("current pos", blockLift.getCurrentPosition());
//            telemetry.addData("delta pos", abs(lastEncoderPos - blockLift.getCurrentPosition()));
//            telemetry.addData("motor stall thres", motorStallThreshold);

            // check the difference in encoder count at every tick
            if (abs(lastEncoderPos - blockLift.getCurrentPosition()) < motorStallThreshold) {
                turnOffLiftMotor();
                isMotorStalled = true;
            }
            // update for comparison in the next loop
            lastEncoderPos = blockLift.getCurrentPosition();

        }
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

    private void initLiftMotor(){
        liftPosition = LiftPosition.LOW;
        context = hardwareMap.appContext;
        if (fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, context).equals("error")) {
            offset = 0;
        } else {
            offset = fileHandler.stringToInt(fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, context));
        }

        blockLift = hardwareMap.dcMotor.get("blockLiftVer2");

        blockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // configure motor direction
        blockLift.setDirection(DcMotor.Direction.REVERSE);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    private void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        turnOffIntakeMotors();
        turnOffDriveMotors();
        blockLift.setPower(0);
        fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + blockLift.getCurrentPosition()), context);
    }
}
