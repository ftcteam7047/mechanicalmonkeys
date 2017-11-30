package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by chrischen on 11/29/17.
 * Test program to control 4 DC motors
 * 1 DC motor for primary intake: motor controller AH00QHU7, 1 = front
 * 2 DC motors for secondary intake: motor controller AL00VVM3, 1 = left, 2 = right
 * 1 DC motor for lift ( timing belt axle diameter = 0.444 inch)
 * 1 MakeBlock servo for rotating the ramp/placing the block
 */

@TeleOp(name = "MM Version2 Test")
//@Disabled

public class MMVersion2Test extends OpMode {

    Servo testServo;
    DcMotor frontIntakeMotor;
    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;
    MMFileHandler fileHandler = new MMFileHandler();
    int offset;
    Context context;

    // servo variables
    int target;
    double targetPos = 0.0;
    boolean lastGamepad1A = false;
    boolean lastGamepad1B = false;
    double tStart = 0;
    int lastTick = 0;
    double currentServoPos = 0.0;
    boolean aIsPressed = false;
    boolean bIsPressed = false;

    // servo parameters
    double openTarget = 1.0;
    double closeTarget = 0.0;
    double timeIncrement = 0.025;
    double posIncrement = 0.02;

    // dcMotor parameters
    double frontIntakeMotorPower = 0.5;
    double leftIntakeMotorPower = 0.75;
    double rightIntakeMotorPower = 0.75;
    double lrIntakeMotorDeltaPower = 0.1;

    enum intakeDir {
        NORMAL,
        REVERSE
    }

    @Override
    public void init() {
        context = hardwareMap.appContext;
        testServo = hardwareMap.servo.get("testServo");
        testServo.setPosition(0.0);
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

    }

    @Override
    public void loop() {
        servoController();
        intakeController();
    }

    private void intakeController() {
        if (gamepad1.x) {
            // normal intake
            setIntakeMotorDir(intakeDir.NORMAL);
            activateIntakeMotors();
        } else if (gamepad1.y) {
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
            tStart = getRuntime();
            aIsPressed = true;
            bIsPressed = false;
            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1A = gamepad1.a;

        if (gamepad1.b && !lastGamepad1B) {
            // close
            tStart = getRuntime();
            aIsPressed = false;
            bIsPressed = true;
            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1B = gamepad1.b;

        if (aIsPressed){
            moveToOpen();
        } else if (bIsPressed){
            moveToClose();
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

    private int getRuntimeInTicks(double tStart, double tick){
        double runtime = getRuntime() - tStart;

        Double numTicks = runtime/tick;
        numTicks.intValue();

        return numTicks.intValue();
    }

    private void moveToOpen(){
        int tick = 0;
        tick = getRuntimeInTicks(tStart, timeIncrement);

        if (tick > lastTick) {
            lastTick = tick;
            targetPos = testServo.getPosition() + posIncrement;
            if (targetPos >= openTarget) {
                targetPos = openTarget;
            }
            testServo.setPosition(targetPos);
        }
        if (openTarget == testServo.getPosition()){
            aIsPressed = false;
        }
    }
    private void moveToClose(){
        int tick = 0;
        tick = getRuntimeInTicks(tStart, timeIncrement);

        if (tick > lastTick) {
            lastTick = tick;
            targetPos = testServo.getPosition() - posIncrement;
            if (targetPos <= closeTarget) {
                targetPos = closeTarget;
            }
            testServo.setPosition(targetPos);
        }
        if (closeTarget == testServo.getPosition()){
            bIsPressed = false;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        turnOffIntakeMotors();
        // TODO: turn off lift motor
    }
}
