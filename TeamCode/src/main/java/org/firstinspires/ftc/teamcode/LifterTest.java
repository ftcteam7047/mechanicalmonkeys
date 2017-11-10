package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by michaelchen on 10/9/17.
 */

@TeleOp(name = "lifterTest")
@Disabled

public class LifterTest extends OpMode {

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

    @Override
    public void init() {
        liftPosition = LiftPosition.GRAB;
        context = hardwareMap.appContext;
        if (fileHandler.readFromFile("offset.txt", context).equals("error")){
            offset = 0;
        } else {
            offset = fileHandler.stringToInt(fileHandler.readFromFile("offset.txt", context));
        }

        blockLift = hardwareMap.dcMotor.get("blockLift");

        blockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }
    public void stop() {
        fileHandler.writeToFile("offset.txt", Integer.toString(offset + blockLift.getCurrentPosition()), context);
    }
}
