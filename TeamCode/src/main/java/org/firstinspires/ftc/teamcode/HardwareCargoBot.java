package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by michaelchen on 10/7/17.
 */

public class HardwareCargoBot {
    // declaration of all robot devices
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public Servo relicServo;
    public Servo gripServoLL;
    public Servo gripServoLR;
    public Servo gripServoUL;
    public Servo gripServoUR;

    // enum for relic arm
    public enum RelicGripPosition {
        CLOSE, OPEN
    }
    RelicGripPosition relicGripPosition;

    // enum for block grabber
    enum GrabberPosition {
        OPEN,
        CLOSE,
        STOW
    }
    GrabberPosition grabberPosition;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
//    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareCargoBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors and Servos
        leftDrive = hwMap.dcMotor.get("leftDrive");
        rightDrive = hwMap.dcMotor.get("rightDrive");
        relicServo = hwMap.servo.get("relicServo");
        gripServoLL = hwMap.servo.get("LLServo");
        gripServoLR = hwMap.servo.get("LRServo");
        gripServoUL = hwMap.servo.get("ULServo");
        gripServoUR = hwMap.servo.get("URServo");

        // Set motor initial direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors Neverest 60
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors Neverest 60

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all servo positions
        relicServo.setPosition(0.0);
        relicGripPosition = RelicGripPosition.CLOSE;

        gripServoLL.setPosition(0.0);
        gripServoLR.setPosition(1.0);
        gripServoUL.setPosition(1.0);
        gripServoUR.setPosition(0.0);
        grabberPosition = GrabberPosition.OPEN;


//        // Set drive motors to run with encoders.
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // Set up motor's maximum tick of the encoder based on its brand and part number
//        int pulsesPerSecond = MMShooterBotConstants.ANDYMARK_40_MAX_TICK_PER_SEC;
//        // TODO to set max speed, configure motor with motor profile within the robot configuration
//        //leftDrive.setMaxSpeed(pulsesPerSecond);
//        //rightDrive.setMaxSpeed(pulsesPerSecond);

    }

//    /***
//     *
//     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
//     * periodic tick.  This is used to compensate for varying processing times for each cycle.
//     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
//     *
//     * @param periodMs  Length of wait cycle in mSec.
//     * @throws InterruptedException
//     */
//    public void waitForTick(long periodMs) throws InterruptedException {
//
//        long  remaining = periodMs - (long)period.milliseconds();
//
//        // sleep for the remaining portion of the regular cycle period.
//        if (remaining > 0)
//            Thread.sleep(remaining);
//
//        // Reset the cycle clock for the next pass.
//        period.reset();
//    }
//
//    public double getVBatt() {
//        double retVal = 0.0;
//
//        // make sure the shooter motor controller exists
//        if (shooterMotorController != null) {
//            retVal = shooterMotorController.getVoltage();
//        }
//        return retVal;
//    }
}
