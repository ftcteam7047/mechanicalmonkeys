package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by michaelchen on 10/7/17.
 */

@TeleOp(name = "CargoBotTeleop")
//@Disabled


public class CargoBotTeleop extends OpMode {

    HardwareCargoBot cargoBot = new HardwareCargoBot();

    boolean isRelicGripperButtonPressed;
    boolean isGrabberButtonPressed;
    boolean isBlockVisible;

    @Override
    public void init() {

        cargoBot.init(hardwareMap);

        isRelicGripperButtonPressed = false;
        isGrabberButtonPressed = false;
        isBlockVisible = false;
    }

    @Override
    public void loop() {

        driveController();
        relicGripperController();
        grabberController();

    }

    public void relicGripperController() {
        if (isRelicGripperButtonPressed) {
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                isRelicGripperButtonPressed = false;
            }
        }

        if (gamepad1.right_bumper
                && cargoBot.relicGripPosition == cargoBot.relicGripPosition.CLOSE
                && !isRelicGripperButtonPressed) {
            isRelicGripperButtonPressed = true;
            cargoBot.relicGripPosition = cargoBot.relicGripPosition.OPEN;
        } else if (gamepad1.left_bumper
                && cargoBot.relicGripPosition == cargoBot.relicGripPosition.OPEN
                && !isRelicGripperButtonPressed) {
            isRelicGripperButtonPressed = true;
            cargoBot.relicGripPosition = cargoBot.relicGripPosition.CLOSE;
        }

        switch (cargoBot.relicGripPosition) {
            case CLOSE:
                cargoBot.relicServo.setPosition(CargoBotConstants.RELIC_ARM_CLOSE);
                break;
            case OPEN:
                cargoBot.relicServo.setPosition(CargoBotConstants.RELIC_ARM_OPEN);
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
            if (cargoBot.grabberPosition == cargoBot.grabberPosition.OPEN) {
                cargoBot.grabberPosition = cargoBot.grabberPosition.CLOSE;
                isGrabberButtonPressed = true;
            } else if (cargoBot.grabberPosition == cargoBot.grabberPosition.CLOSE) {
                cargoBot.grabberPosition = cargoBot.grabberPosition.OPEN;
                isGrabberButtonPressed = true;
            } else if (cargoBot.grabberPosition == cargoBot.grabberPosition.STOW) {
                cargoBot.grabberPosition = cargoBot.grabberPosition.OPEN;
                isGrabberButtonPressed = true;
            }
        }

        if (gamepad1.b && !isBlockVisible) {
            cargoBot.grabberPosition = cargoBot.grabberPosition.STOW;
        }

        switch (cargoBot.grabberPosition) {
            case OPEN:
                cargoBot.gripServoLL.setPosition(0.0);
                cargoBot.gripServoLR.setPosition(1.0);
                cargoBot.gripServoUL.setPosition(1.0);
                cargoBot.gripServoUR.setPosition(0.0);
                break;
            case CLOSE:
                cargoBot.gripServoLL.setPosition(CargoBotConstants.GRABBER_CLOSE);
                cargoBot.gripServoLR.setPosition(CargoBotConstants.GRABBER_CLOSE);
                cargoBot.gripServoUL.setPosition(CargoBotConstants.GRABBER_CLOSE);
                cargoBot.gripServoUR.setPosition(CargoBotConstants.GRABBER_CLOSE);
                break;
            case STOW:
                cargoBot.gripServoLL.setPosition(1.0);
                cargoBot.gripServoLR.setPosition(0.0);
                cargoBot.gripServoUL.setPosition(0.0);
                cargoBot.gripServoUR.setPosition(1.0);
                break;
        }
    }

    public void driveController() {
        cargoBot.leftDrive.setPower(gamepad1.left_stick_y);
        cargoBot.rightDrive.setPower(gamepad1.right_stick_y);
    }
}
