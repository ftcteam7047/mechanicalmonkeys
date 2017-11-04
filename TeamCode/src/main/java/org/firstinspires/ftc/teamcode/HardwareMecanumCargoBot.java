package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by chrischen on 10/21/17.
 */

public class HardwareMecanumCargoBot {
    // initialize fileHandler
    MMFileHandler fileHandler = new MMFileHandler();
    Context context;

    // declaration of all robot drive motors
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor rearLeftDrive;
    public DcMotor rearRightDrive;

    // declaration of all non drive motors
    public DcMotor blockLift;

    // declaration of all robot servos
    public Servo ballArm;
    // TODO: relic servo
    //public Servo relicServo;
    public Servo lowerLeftServo;
    public Servo lowerRightServo;
    public Servo upperLeftServo;
    public Servo upperRightServo;

    // enum for relic arm
    public enum RelicGripPosition {
        CLOSE,
        OPEN
    }
    RelicGripPosition relicGripPosition;

    // enum for block grabber
    public enum GrabberPosition {
        OPEN,
        CLOSE,
        STOW
    }
    GrabberPosition grabberPosition;

    // enum for block lift
    public enum LiftPosition {
        GRAB,
        MOVE,
        STACK,
        PLACE,
        INIT_POSITION
    }
    LiftPosition liftPosition;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
//    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanumCargoBot(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // fileHandler context
        context = hwMap.appContext;

        // Define and Initialize Motors and Servos
        frontLeftDrive = hwMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hwMap.dcMotor.get("frontRightDrive");
        rearLeftDrive = hwMap.dcMotor.get("rearLeftDrive");
        rearRightDrive = hwMap.dcMotor.get("rearRightDrive");
        blockLift = hwMap.dcMotor.get("blockLift");
        //relicServo = hwMap.servo.get("relicServo");
        ballArm = hwMap.servo.get("ballArm");
        lowerLeftServo = hwMap.servo.get("lls");
        lowerRightServo = hwMap.servo.get("lrs");
        upperLeftServo = hwMap.servo.get("uls");
        upperRightServo = hwMap.servo.get("urs");


        // Set motor initial direction
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors Neverest 40
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors Neverest 40
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors Neverest 40
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors Neverest 40

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        // Encoder reset: this step seems to be necessary to allow RUN_TO_POSITION to work
        // otherwise, motor speed may not be consistent.
        // Watch out for inconsistent motor speed in the autonomous mode, if the following is not performed
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosition = LiftPosition.INIT_POSITION;
        blockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all servos to initial position
        ballArm.setPosition(CargoBotConstants.BALL_ARM_UP);

        // TODO: relic servos
        //relicServo.setPosition(0.0);
        relicGripPosition = RelicGripPosition.CLOSE;

        lowerLeftServo.setPosition(0.5);
        lowerRightServo.setPosition(0.5);
        upperLeftServo.setPosition(0.5);
        upperRightServo.setPosition(0.5);
        grabberPosition = GrabberPosition.OPEN;

//        // Set drive motors to run with encoders.
//        front:eftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // Set up motor's maximum tick of the encoder based on its brand and part number
//        int pulsesPerSecond = MMShooterBotConstants.ANDYMARK_40_MAX_TICK_PER_SEC;
//        // TODO to set max speed, configure motor with motor profile within the robot configuration
//        //frontLeftDrive.setMaxSpeed(pulsesPerSecond);
//        //frontRightDrive.setMaxSpeed(pulsesPerSecond);

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
