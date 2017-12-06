package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by michaelchen on 10/3/17.
 *
 * button a toggles from open to close
 * button b stows
 * if stowed, button a opens
 * on real robot, sensor will detect block and stop robot from stowing
 * this is demonstrated by using y
 */

@TeleOp(name = "Concept: GripperArmTest", group = "Concept")
@Disabled

public class GripperArmTest extends OpMode {

    MMFileHandler fileHandler = new MMFileHandler();
    Context context;
    boolean isLifterButtonPressed;
    DcMotor blockLift;
    int target;
    int offset;

    public enum LiftPosition {
        GRAB,
        MOVE,
        STACK,
        PLACE
    }
    LiftPosition liftPosition;

    Servo lServo;
    Servo rServo;
    Servo lHServo;
    Servo rHServo;

    boolean isBlockVisible;
    boolean isGripperButtonPressed;
    boolean isVisibilityButtonPressed;

    enum gripperPosition {
        OPEN,
        CLOSE,
        STOW
    }

    gripperPosition position;

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void init() {
        lServo = hardwareMap.servo.get("leftServo");
        rServo = hardwareMap.servo.get("rightServo");
        lHServo = hardwareMap.servo.get("hlservo");
        rHServo = hardwareMap.servo.get("hrservo");
        lServo.setPosition(0.0);
        rServo.setPosition(1.0);
        lHServo.setPosition(1.0);
        rHServo.setPosition(0.0);
        position = gripperPosition.OPEN;
        isBlockVisible = false;
        isVisibilityButtonPressed = false;

        liftPosition = LiftPosition.GRAB;
        context = hardwareMap.appContext;
        if (fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, context).equals("error")){
            offset = 0;
        } else {
            offset = fileHandler.stringToInt(fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, context));
        }

        blockLift = hardwareMap.dcMotor.get("blockLift");

        blockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blockLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            if (liftPosition == liftPosition.GRAB && !isLifterButtonPressed) {
                liftPosition = liftPosition.MOVE;
                isLifterButtonPressed = true;
            } else if (liftPosition == liftPosition.MOVE && !isLifterButtonPressed) {
                liftPosition = liftPosition.STACK;
                isLifterButtonPressed = true;
            } else if (liftPosition == liftPosition.STACK && !isLifterButtonPressed) {
                liftPosition = liftPosition.PLACE;
                isLifterButtonPressed = true;
            }
        } else if (gamepad1.dpad_down) {
            if (liftPosition == liftPosition.PLACE && !isLifterButtonPressed) {
                liftPosition = liftPosition.STACK;
                isLifterButtonPressed = true;
            } else if (liftPosition == liftPosition.STACK && !isLifterButtonPressed) {
                liftPosition = liftPosition.MOVE;
                isLifterButtonPressed = true;
            } else if (liftPosition == liftPosition.MOVE && !isLifterButtonPressed) {
                liftPosition = liftPosition.GRAB;
                isLifterButtonPressed = true;
            }
        }

        switch (liftPosition) {
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
        }

        blockLift.setTargetPosition(target);
        blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blockLift.setPower(CargoBotConstants.LIFT_SPEED);
        telemetry.addData("pos", blockLift.getCurrentPosition());
        telemetry.update();
        if (!blockLift.isBusy()) {
            blockLift.setPower(0.0);
            blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (isLifterButtonPressed) {
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                isLifterButtonPressed = false;
            }
        }

        if (gamepad1.a && !isGripperButtonPressed) {
            if (position == gripperPosition.OPEN) {
                position = gripperPosition.CLOSE;
                isGripperButtonPressed = true;
            } else if (position == gripperPosition.CLOSE) {
                position = gripperPosition.OPEN;
                isGripperButtonPressed = true;
            } else if (position == gripperPosition.STOW) {
                position = gripperPosition.OPEN;
                isGripperButtonPressed = true;
            }

        }

        if (gamepad1.b && !isBlockVisible) {
            position = gripperPosition.STOW;
        }

        if (isGripperButtonPressed) {
            if (!gamepad1.a) {
                isGripperButtonPressed = false;
            }
        }

        if (gamepad1.y && !isVisibilityButtonPressed) {
            if (isBlockVisible == false) {
                isBlockVisible = true;
                isVisibilityButtonPressed = true;
            } else if(isBlockVisible == true) {
                isBlockVisible = false;
                isVisibilityButtonPressed = true;
            }
        }

        if(isVisibilityButtonPressed) {
            if(!gamepad1.y) {
                isVisibilityButtonPressed = false;
            }
        }

        switch (position) {
            case OPEN:
                lServo.setPosition(0.0);
                rServo.setPosition(1.0);
                lHServo.setPosition(1.0);
                rHServo.setPosition(0.1);
                break;
            case CLOSE:
                lServo.setPosition(0.5);
                rServo.setPosition(0.5);
                lHServo.setPosition(0.5);
                rHServo.setPosition(0.5);
                break;
            case STOW:
                lServo.setPosition(1.0);
                rServo.setPosition(0.0);
                lHServo.setPosition(0.0);
                rHServo.setPosition(1.0);
        }

        if(isBlockVisible) {
            telemetry.addData("is block visible:","YES");
        } else {
            telemetry.addData("is block visible", "NO");
        }
        telemetry.update();

        leftDrive.setPower(gamepad1.left_stick_y * 0.4);
        rightDrive.setPower(-gamepad1.right_stick_y * 0.4);

    }

    public void stop() {
        fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + blockLift.getCurrentPosition()), context);
    }
    
}
