package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by michaelchen on 9/25/17.
 */

@TeleOp(name = "Concept: TestRelicControl", group = "Concept")
@Disabled

public class TestRelicControl extends OpMode {

    Servo servo;

    private enum Position{
        RETRACT, EXTEND
    }

    Position position;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("Servo");
        servo.setPosition(0.0);
        position = Position.RETRACT;
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper && position == Position.RETRACT) {
            position = Position.EXTEND;
        } else if(gamepad1.left_bumper && position == Position.EXTEND) {
            position = Position.RETRACT;
        }

        switch(position) {
            case RETRACT:
                servo.setPosition(0.0);
                break;
            case EXTEND:
                servo.setPosition(0.4);
                break;
        }
        while(gamepad1.left_bumper || gamepad1.right_bumper) {

        }
    }
}
