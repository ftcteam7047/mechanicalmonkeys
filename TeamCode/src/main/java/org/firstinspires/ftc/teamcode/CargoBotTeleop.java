package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by michaelchen on 10/7/17.
 */

@TeleOp(name = "CargoBotTeleop")
@Disabled


public class CargoBotTeleop extends OpMode {

    HardwareCargoBot robot = new HardwareCargoBot();

    boolean isRelicGripperButtonPressed;
    boolean isGrabberButtonPressed;
    boolean isBlockVisible;
    boolean isLifterButtonPressed;

    int target;
    int offset;

    @Override
    public void init() {

        robot.init(hardwareMap);

        isRelicGripperButtonPressed = false;
        isGrabberButtonPressed = false;
        isBlockVisible = false;
        isLifterButtonPressed = false;

        if (robot.fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, robot.context).equals("error")){
            offset = 0;
        } else {
            offset = robot.fileHandler.stringToInt(robot.fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, robot.context));
        }
    }

    @Override
    public void loop() {

        driveController();
        relicGripperController();
        grabberController();
        blockLiftController();
    }

    @Override
    public void stop() {
        robot.fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + robot.blockLift.getCurrentPosition()), robot.context);
    }

    public void relicGripperController() {
        if (isRelicGripperButtonPressed) {
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                isRelicGripperButtonPressed = false;
            }
        }

        if (gamepad1.right_bumper
                && robot.relicGripPosition == robot.relicGripPosition.CLOSE
                && !isRelicGripperButtonPressed) {
            isRelicGripperButtonPressed = true;
            robot.relicGripPosition = robot.relicGripPosition.OPEN;
        } else if (gamepad1.left_bumper
                && robot.relicGripPosition == robot.relicGripPosition.OPEN
                && !isRelicGripperButtonPressed) {
            isRelicGripperButtonPressed = true;
            robot.relicGripPosition = robot.relicGripPosition.CLOSE;
        }

        switch (robot.relicGripPosition) {
            case CLOSE:
                robot.relicServo.setPosition(CargoBotConstants.RELIC_ARM_CLOSE);
                break;
            case OPEN:
                robot.relicServo.setPosition(CargoBotConstants.RELIC_ARM_OPEN);
                break;
        }
    }

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
            } else if (robot.grabberPosition == robot.grabberPosition.STOW) {
                robot.grabberPosition = robot.grabberPosition.OPEN;
                isGrabberButtonPressed = true;
            }
        }

        if (gamepad1.b && !isBlockVisible) {
            robot.grabberPosition = robot.grabberPosition.STOW;
        }

        switch (robot.grabberPosition) {
            case OPEN:
                robot.gripServoLL.setPosition(0.0);
                robot.gripServoLR.setPosition(1.0);
                robot.gripServoUL.setPosition(1.0);
                robot.gripServoUR.setPosition(0.0);
                break;
            case CLOSE:
                robot.gripServoLL.setPosition(CargoBotConstants.GRABBER_CLOSE);
                robot.gripServoLR.setPosition(CargoBotConstants.GRABBER_CLOSE);
                robot.gripServoUL.setPosition(CargoBotConstants.GRABBER_CLOSE);
                robot.gripServoUR.setPosition(CargoBotConstants.GRABBER_CLOSE);
                break;
            case STOW:
                robot.gripServoLL.setPosition(1.0);
                robot.gripServoLR.setPosition(0.0);
                robot.gripServoUL.setPosition(0.0);
                robot.gripServoUR.setPosition(1.0);
                break;
        }
    }

    public void driveController() {
        robot.leftDrive.setPower(gamepad1.left_stick_y);
        robot.rightDrive.setPower(gamepad1.right_stick_y);
    }

    public void blockLiftController() {
        if (gamepad1.dpad_up) {
            if (robot.liftPosition == robot.liftPosition.GRAB && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.MOVE;
                isLifterButtonPressed = true;
            } else if (robot.liftPosition == robot.liftPosition.MOVE && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.STACK;
                isLifterButtonPressed = true;
            } else if (robot.liftPosition == robot.liftPosition.STACK && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.PLACE;
                isLifterButtonPressed = true;
            }
        } else if (gamepad1.dpad_down) {
            if (robot.liftPosition == robot.liftPosition.PLACE && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.STACK;
                isLifterButtonPressed = true;
            } else if (robot.liftPosition == robot.liftPosition.STACK && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.MOVE;
                isLifterButtonPressed = true;
            } else if (robot.liftPosition == robot.liftPosition.MOVE && !isLifterButtonPressed) {
                robot.liftPosition = robot.liftPosition.GRAB;
                isLifterButtonPressed = true;
            }
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
        }

        robot.blockLift.setTargetPosition(target);
        robot.blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.blockLift.setPower(CargoBotConstants.LIFT_SPEED);
        telemetry.addData("pos", robot.blockLift.getCurrentPosition());
        telemetry.update();
        if (!robot.blockLift.isBusy()) {
            robot.blockLift.setPower(0.0);
            robot.blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (isLifterButtonPressed) {
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                isLifterButtonPressed = false;
            }
        }
    }
}
