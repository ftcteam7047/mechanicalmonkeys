package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by michaelchen on 9/30/17.
 */

@TeleOp(name = "Concept: TestCargoBotChainDrive", group = "Concept")
@Disabled

public class TestCargoBotChainDrive extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    @Override
    public void loop() {
        leftDrive.setPower(gamepad1.left_stick_y);
        rightDrive.setPower(-gamepad1.right_stick_y);
    }
}
