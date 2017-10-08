package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
//@Disabled

public class GripperArmTest extends OpMode {

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

    @Override
    public void init() {
        lServo = hardwareMap.servo.get("leftServo");
        rServo = hardwareMap.servo.get("rightServo");
        lHServo = hardwareMap.servo.get("hlservo");
        rHServo = hardwareMap.servo.get("hrservo");
        lServo.setPosition(0.0);
        rServo.setPosition(1.0);
        lHServo.setPosition(0.75);
        rHServo.setPosition(0.25);
        position = gripperPosition.OPEN;
        isBlockVisible = false;
        isVisibilityButtonPressed = false;
    }

    @Override
    public void loop() {
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
                lHServo.setPosition(0.75);
                rHServo.setPosition(0.25);
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
                lHServo.setPosition(0.1);
                rHServo.setPosition(1.0);
        }

        if(isBlockVisible) {
            telemetry.addData("is block visible:","YES");
        } else {
            telemetry.addData("is block visible", "NO");
        }
        telemetry.update();

    }
}
