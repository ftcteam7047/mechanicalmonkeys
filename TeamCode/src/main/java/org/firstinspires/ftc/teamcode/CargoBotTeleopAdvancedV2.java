package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.DecimalFormat;

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

@TeleOp(name = "CargoBotAdvanced V2")
//@Disabled

public class CargoBotTeleopAdvancedV2 extends OpMode {

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
    double flatTarget = 0.66;
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

    // get back on the platform
    double lastGamepad1LeftTrigger = 0.0;
    double lastGamepad1RightTrigger = 0.0;
    double currentHeading = 0;
    enum enumPlatFormState {
        START,
        RUN,
        END,
    }
    enumPlatFormState platformState = enumPlatFormState.END;
    boolean isGettingOnPlatform = false;

    // NavX micro
    private final int NAVX_DIM_I2C_PORT = 1;
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
    boolean isNavxDriveInitiated = false;
    navXPIDController.PIDResult yawPIDResult;
    double navxDriveStartTime = 0;

    // for activating intake motor while the ramp is moving to flat or up
    boolean shouldRampMoveToDownDueToLift = false;
    boolean shouldRampMoveToFlatDueToLift = false;
    boolean isRampUpEvent = false;
    boolean isLiftAtLowPosition = false;
    boolean lastGamepadDown = false;
    boolean lastGamepadUp = false;
    double lifterDownStartTime = 0.0;
    double lifterUpStartTime = 0.0;
    boolean shouldActivateIntakeDueToLifting = false;

    // rotate to nearest 90 degrees
    private enum ENUM_NAVX_GYRO_TURN {
        NOT_ARRIVED,
        ARRIVED,
        TIMEOUT
    }

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

        // get a reference to NavX-micro device
        navxDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        // init gyros
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

    @Override
    public void loop() {
        servoController();
        intakeController();
        driveController();
        blockLiftController();
        // rotate to nearest 90 is not used
        // back on to platform is not used
//        try {
//            checkCommandToGetOnToPlatform();
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        rotateToNearest90DegreeController();

        try {
            waitForTick(2); // every 2ms, sample the encoder
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
        if (gamepad2.a || shouldActivateIntakeDueToLifting) {
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
            // ramp down
            aIsPressed = true;
            bIsPressed = false;
            xIsPressed = false;
            // allow driver input to override the request by block lift controller
            shouldRampMoveToFlatDueToLift = false;
            shouldRampMoveToDownDueToLift = false;

            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1A = gamepad1.a;

        if (gamepad1.b && !lastGamepad1B) {
            // ramp up
            aIsPressed = false;
            bIsPressed = true;
            xIsPressed = false;
            // allow driver input to override the request by block lift controller
            shouldRampMoveToFlatDueToLift = false;
            shouldRampMoveToDownDueToLift = false;

            lastTick = getRuntimeInTicks(tStart, timeIncrement);
            isRampUpEvent = true;
        } else {
            isRampUpEvent = false;
        }
        // store a copy for comparison in the next loop
        lastGamepad1B = gamepad1.b;

        if (gamepad1.x && !lastGamepad1X) {
            // ramp flat
            aIsPressed = false;
            bIsPressed = false;
            xIsPressed = true;
            // allow driver input to override the request by block lift controller
            shouldRampMoveToFlatDueToLift = false;
            shouldRampMoveToDownDueToLift = false;

            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1X = gamepad1.x;

        // execute the servo movement based on driver input (button A/B/X)
        // or intended lift target (d-pad up/down)
        if (aIsPressed || shouldRampMoveToDownDueToLift) {
            moveRampToDown();
        } else if (bIsPressed) {
            moveRampToUp();
        } else if (xIsPressed || shouldRampMoveToFlatDueToLift) {
            moveRampToFlat();
        }
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


    private void activateIntakeMotors(){
        // activate primary intake motor
        frontIntakeMotor.setPower(frontIntakeMotorPower);
        // activate equal power on both left and right intake motors
        leftIntakeMotor.setPower(leftIntakeMotorPower);
        rightIntakeMotor.setPower(rightIntakeMotorPower);
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
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private void moveRampToDown(){
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
        // cancel the request when the servo is at the target
        if (downTarget == testServo.getPosition()){
            aIsPressed = false;
            shouldRampMoveToDownDueToLift = false;
        }
    }
    private void moveRampToUp(){
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
        // cancel the request when the servo is at the target
        if (upTarget == testServo.getPosition()){
            bIsPressed = false;
        }
    }

    private void moveRampToFlat(){
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
        // cancel the request when the servo is at the target
        if (flatTarget == testServo.getPosition()){
            xIsPressed = false;
            shouldRampMoveToFlatDueToLift = false;
        }
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

                frontLeftDrive.setPower(-output);
                frontRightDrive.setPower(output);
                rearLeftDrive.setPower(-output);
                rearRightDrive.setPower(output);
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

    public void driveController(){
        if (!isRotatingToNearest90Degree && !isGettingOnPlatform){
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
                        // Limit the top strafing speed of the robot in low speed mode
                        if (isLowSpeedMode) {
                            sideMovement = (float) Range.clip(sideMovement, -CargoBotConstants.TELEOP_SLOW_MODE_STRAFING_TOP_SPEED, CargoBotConstants.TELEOP_SLOW_MODE_STRAFING_TOP_SPEED);
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
                if (gamepad2.dpad_up) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.LOW;
                        isLifterButtonPressed = true;
                    }
                }
            } else if (offset >= low && offset < high) {
                // can accept dpad up or down
                if (gamepad2.dpad_up) {
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
        if (gamepad2.dpad_up) {
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


        setLifterStartTime();

        // don't move the lifter without user input
        if (liftPosition != LiftPosition.INIT_POSITION) {
            if (!isMotorStalled) {
                if ((getRuntime() - lifterUpStartTime) < CargoBotConstants.INTAKE_MOTOR_ACTIVE_TIME) {
                    shouldActivateIntakeDueToLifting = true;
                } else {
                    shouldActivateIntakeDueToLifting = false;
                    blockLift.setTargetPosition(target);
                    blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (CargoBotConstants.SET_LIFT_MOTOR_HIGH_SPEED) {
                        // always lift up/down at high speed
                        blockLift.setPower(CargoBotConstants.LIFT_HI_SPEED);
                    } else {
                        blockLift.setPower(CargoBotConstants.LIFT_SPEED);
                    }
                }
            }

            //telemetry.addData("lift encoder pos", blockLift.getCurrentPosition());
            //telemetry.update();
            if (!blockLift.isBusy()) {
                turnOffLiftMotor();
                reportLiftLogicalPos();
            } else {

                //activate intake motor for a short burst to help blocks clear the obstacle
                if (!shouldActivateIntakeDueToLifting){
                    if (liftPosition == LiftPosition.HIGH) {
                        if (!isLiftNearHighPosition() && !isMotorStalled && !isLifterUpTimeout()) {
                            // inform servo controller that we want to move to flat position
                            shouldRampMoveToFlatDueToLift = true;
                        }
                    }
                }
                if (liftPosition == LiftPosition.LOW) {
                    if (!isLiftNearLowPosition() && !isMotorStalled && !isLifterDownTimeout()) {
                        // inform servo controller that we want to move to down position
                        shouldRampMoveToDownDueToLift = true;
                        shouldActivateIntakeDueToLifting = false;
                    }
                }
                if (!shouldActivateIntakeDueToLifting) {
                    detectNeverRest20MotorStall();
                }
            }
        }

        if (isLifterButtonPressed) {
            if (!gamepad1.dpad_down && !gamepad2.dpad_up) {
                isLifterButtonPressed = false;
            }
        }

    }

    private void setLifterStartTime(){
        // record the time when lifter down button has been pressed
        if (!lastGamepadDown && gamepad1.dpad_down){
            lifterDownStartTime = getRuntime();
        }
        // record the time when lifter up button has been pressed
        if (!lastGamepadUp && gamepad2.dpad_up){
            lifterUpStartTime = getRuntime();
        }
        // for comparison in the next loop
        lastGamepadDown = gamepad1.dpad_down;
        lastGamepadUp = gamepad2.dpad_up;
    }

    private boolean isLifterUpTimeout(){
        // account for the time to activate intake motor
        return ((getRuntime() - lifterUpStartTime) > (CargoBotConstants.LIFT_MOTOR_TIMEOUT + CargoBotConstants.INTAKE_MOTOR_ACTIVE_TIME));
    }
    private boolean isLifterDownTimeout(){
        return ((getRuntime() - lifterDownStartTime) > CargoBotConstants.LIFT_MOTOR_TIMEOUT);
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
            int deltaTicks = tick - lastMotorStallTick;
            double time = CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD * deltaTicks;
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


    /* switch to high speed
     * advance a defined distance onto the platform
     * brake
     */
    private void getOnPlatform() throws InterruptedException {
        boolean driveStatus = false;
        switch (platformState){
            case START:
                // switch back to normal speed
                isLowSpeedMode = false;
                // get robot heading to be used for navxDrive
                currentHeading = -navxDevice.getYaw(); // adding negative to the value so that for the robot, CCW is positive, CW is negative.
                // inform other controller the status
                isGettingOnPlatform = true;
                platformState = enumPlatFormState.RUN;
                break;

            case RUN:
                driveStatus = navxDrive(CargoBotConstants.BACKUP_ONTO_PLATFORM_SPEED,
                        CargoBotConstants.BACKUP_ONTO_PLATFORM_DISTANCE,
                        calculateTimeout(CargoBotConstants.BACKUP_ONTO_PLATFORM_DISTANCE, CargoBotConstants.BACKUP_ONTO_PLATFORM_SPEED),
                        currentHeading);

                if (driveStatus) {
                    platformState = enumPlatFormState.END;
                }
                break;

            case END:
                isLowSpeedMode = true;
                // inform other controller the status
                isGettingOnPlatform = false;
                break;
        }
    }

    private double calculateTimeout(double distance, double speed) {
        double timeoutS = 0.0;
        timeoutS = distance/speed * MMShooterBotConstants.TIMEOUT_SCALING_FACTOR;

        // make sure we return a positive value
        return Math.abs(timeoutS);
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

    private boolean navxDrive( double speed,
                              double distance,
                              double timeoutS,
                              double angle) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        boolean driveComplete = false;


        if (!isNavxDriveInitiated) {
            isNavxDriveInitiated = true;
            // negation for mecanum drive arrangement
            distance = -distance;

            // set up navx stuff
            double angleNormalized = -angle; // reverse the angle's direction: since positive is for CCW, negative is for CW
            // set the parameters before enabling the PID controller
            yawDrivePIDController.enable(true);
            yawDrivePIDController.setSetpoint(angleNormalized);
            yawDrivePIDController.setPID(Kp, MMShooterBotConstants.YAW_PID_I, MMShooterBotConstants.YAW_PID_D);

            yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

            Thread.sleep(MMShooterBotConstants.SLEEP_MS);

            navxDriveStartTime = getRuntime();

            setChassisMotorZeroBehavior(eZeroBehavior.BRAKE);
        }


        // according to documentation, this instruction blocks the thread, and won't return immediately
        // it returns true if new data is available; false if it times out.
        if (yawDrivePIDController.waitForNewUpdate(yawPIDResult, MMShooterBotConstants.WAIT_FOR_UPDATE_TIMEOUT_MS)) {
            // proceed to keep using data from navx-micro

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * MMShooterBotConstants.COUNTS_PER_INCH);
            newLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            newLeftTarget = rearLeftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rearRightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontLeftDrive.setTargetPosition(newLeftTarget);
            frontRightDrive.setTargetPosition(newRightTarget);
            rearLeftDrive.setTargetPosition(newLeftTarget);
            rearRightDrive.setTargetPosition(newRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            // for debugging only, activate it when necessary
            //RobotLog.vv(MMShooterBotConstants.GYRO_DRIVE_TAG, ",run time after reset =,%5.2f, seconds", runtime.seconds());

            // keep looping while we are still active, and all motors are running.
            if (frontLeftDrive.isBusy() && frontRightDrive.isBusy()
                    && rearLeftDrive.isBusy() && rearRightDrive.isBusy() &&
                    ((getRuntime() - navxDriveStartTime) < timeoutS)) {
                if (yawDrivePIDController.isNewUpdateAvailable(yawPIDResult)) {
                    if (yawPIDResult.isOnTarget()) {
                        frontLeftDrive.setPower(speed);
                        frontRightDrive.setPower(speed);
                        rearLeftDrive.setPower(speed);
                        rearRightDrive.setPower(speed);
//                            telemetry.addData("PIDOutput", df.format(speed) + ", " +
//                                    df.format(speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        // if driving in reverse, the motor correction also needs to be reversed
                        if (distance < 0)
                            output *= -1.0;

                        frontLeftDrive.setPower(speed + output);
                        frontRightDrive.setPower(speed - output);
                        rearLeftDrive.setPower(speed + output);
                        rearRightDrive.setPower(speed - output);
//                            telemetry.addData("PIDOutput", df.format(limit(speed + output)) + ", " +
//                                    df.format(limit(speed - output)));
                    }
//                        telemetry.addData("Yaw", df.format(navxDevice.getYaw()));
                }
//                    telemetry.update();

                // for debugging only, activate it when necessary
                //RobotLog.vv(MMShooterBotConstants.GYRO_DRIVE_TAG, ",run time =,%5.2f, seconds", runtime.seconds());
            } else {
                // stop the momentum of the robot
                turnOffDriveMotors();
                isNavxDriveInitiated = false;
                yawDrivePIDController.enable(false);
                driveComplete = true;
            }
        } else {
            // time out occurs
            RobotLog.vv("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            isNavxMicroDataTimeout = true;
            isNavxDriveInitiated = false;
            yawDrivePIDController.enable(false);
            driveComplete = true;
        }

        return driveComplete;
    }

    private void checkCommandToGetOnToPlatform() throws InterruptedException {
        // don't get back on to platform unless robot is relatively stationary, controlled by driver 1
        if ((Math.abs(gamepad1.left_stick_y) <= CargoBotConstants.CONTROL_STICK_THRESHOLD_FOR_BACKUP_TO_PLATFORM) &&
                (Math.abs(gamepad1.right_stick_y) <= CargoBotConstants.CONTROL_STICK_THRESHOLD_FOR_BACKUP_TO_PLATFORM)) {

            // left trigger & right trigger fully pressed simultaneously, controlled by driver 1
            if (((gamepad1.left_trigger == MMShooterBotConstants.FULLY_PRESSED) && (lastGamepad1LeftTrigger != MMShooterBotConstants.FULLY_PRESSED)
                    && (gamepad1.right_trigger == MMShooterBotConstants.FULLY_PRESSED))
                    ||
                    ((gamepad1.right_trigger == MMShooterBotConstants.FULLY_PRESSED) && (lastGamepad1RightTrigger != MMShooterBotConstants.FULLY_PRESSED)
                            && (gamepad1.left_trigger == MMShooterBotConstants.FULLY_PRESSED))) {
                if (platformState == enumPlatFormState.END) {
                    platformState = enumPlatFormState.START;
                } else if (platformState == enumPlatFormState.RUN) {
                    // abort
                    platformState = enumPlatFormState.END;
                    turnOffDriveMotors();
                    isNavxDriveInitiated = false;
                    yawDrivePIDController.enable(false);
                }
            }

            // store current left and right trigger for comparison in the next loop
            lastGamepad1LeftTrigger = gamepad1.left_trigger;
            lastGamepad1RightTrigger = gamepad1.right_trigger;
        }
        getOnPlatform();
    }

    private boolean isLiftNearHighPosition(){
        boolean retVal = false;
        int highTarget = (int) CargoBotConstants.HIGH_DISTANCE_FROM_START - offset;
        int currentLiftEncoder = blockLift.getCurrentPosition();

        // if lift is close enough to the target
        if (abs(highTarget - currentLiftEncoder) < CargoBotConstants.LIFT_TARGET_TOLERANCE){
            retVal = true;
        }
        return retVal;
    }

    private boolean isLiftNearLowPosition(){
        boolean retVal = false;
        int lowTarget = (int) CargoBotConstants.LOW_DISTANCE_FROM_START - offset;
        int currentLiftEncoder = blockLift.getCurrentPosition();

        // if lift is close enough to the target
        if (abs(lowTarget - currentLiftEncoder) < CargoBotConstants.LIFT_TARGET_TOLERANCE){
            retVal = true;
        }
        return retVal;
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
        navxDevice.close();
        turnOffIntakeMotors();
        turnOffDriveMotors();
        blockLift.setPower(0);
        fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + blockLift.getCurrentPosition()), context);
    }
}
