package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by chrischen on 11/29/17.
 * Apply pseudo PWM algorithm to control the speed of the servo
 */

@TeleOp(name = "Servo Test")
@Disabled

public class TestServo extends OpMode {

    Servo testServo;
    MMFileHandler fileHandler = new MMFileHandler();
    int offset;
    Context context;
    int target;
    double timeIncrement = 0.025;
    double posIncrement = 0.02;
    double targetPos = 0.0;
    boolean lastGamepad1A = false;
    boolean lastGamepad1B = false;
    double tStart = 0;
    int lastTick = 0;
    double currentServoPos = 0.0;
    double downTarget = 0.85;
    double upTarget = 0.0;
    boolean aIsPressed = false;
    boolean bIsPressed = false;

    @Override
    public void init() {

        context = hardwareMap.appContext;
        testServo = hardwareMap.servo.get("testServo");
        testServo.setPosition(downTarget);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        tStart = getRuntime();
    }

    @Override
    public void loop() {
        servoController();
    }

    private void servoController(){

        if (gamepad1.a && !lastGamepad1A) {
            // open
            tStart = getRuntime();
            currentServoPos = testServo.getPosition();
            aIsPressed = true;
            bIsPressed = false;
            lastTick = getRuntimeInTicks(tStart, timeIncrement);
        }
        // store a copy for comparison in the next loop
        lastGamepad1A = gamepad1.a;

        if (gamepad1.b && !lastGamepad1B) {
            // close
            tStart = getRuntime();
            currentServoPos = testServo.getPosition();
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
            if (targetPos >= downTarget) {
                targetPos = downTarget;
            }
            testServo.setPosition(targetPos);
        }
        if (downTarget == testServo.getPosition()){
            aIsPressed = false;
        }
    }
    private void moveToClose(){
        int tick = 0;
        tick = getRuntimeInTicks(tStart, timeIncrement);

        if (tick > lastTick) {
            lastTick = tick;
            targetPos = testServo.getPosition() - posIncrement;
            if (targetPos <= upTarget) {
                targetPos = upTarget;
            }
            testServo.setPosition(targetPos);
        }
        if (upTarget == testServo.getPosition()){
            bIsPressed = false;
        }
    }

    @Override
    public void stop() {

    }
}
