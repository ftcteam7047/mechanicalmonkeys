package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * Created by chrischen on 10/21/17.
 */

@TeleOp(name = "MecanumTestBotTeleop")
//@Disabled


public class MecanumTestBotTeleop extends OpMode {

    HardwareMecanumTestBot mecanumBot = new HardwareMecanumTestBot();



    @Override
    public void init() {
        mecanumBot.init(hardwareMap);

        // Run using encoder will force the motor to run at constant speed
        // Run at constant speed: This mode controls the speed of the motor
        // such that it maintains an average speed. If the motor is set to 85% power in this mode,
        // the controller will monitor the encoder to give an appropriate power to the motor
        // such that the motor rotates at 85% of its maximum average speed. Average is a key word here.
        // A motor set to 100% power in this mode will not rotate as fast as a motor set to 100% power in Run at constant power.
        // This is to allow the motor to catch up if it falls behind. If the motor experiences more resistance, more power will be applied to catch up to where it would have been if it never slowed down.
        // Think of this like how you actually drive.
        // Going up a hill, you will press further on the gas and going down the hill you will let up on the gas.
        // This mode requires encoders.
        // The top average speed in this mode is less than 100% in run at constant power because the motor needs to conserve the highest amounts of power to catch up if it experiences resistance.

        // Encoder reset if necessary
        // Run motor at constant speed. (see above for details)
        mecanumBot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumBot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumBot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumBot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumBot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumBot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumBot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumBot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        driveController();
    }


    public void driveController() {
        if (abs(gamepad1.left_stick_x) < 0.25 && abs(gamepad1.right_stick_x) < 0.25) {
            mecanumBot.frontLeftDrive.setPower(gamepad1.left_stick_y);
            mecanumBot.frontRightDrive.setPower(gamepad1.right_stick_y);
            mecanumBot.rearLeftDrive.setPower(gamepad1.left_stick_y);
            mecanumBot.rearRightDrive.setPower(gamepad1.right_stick_y);
        } else {
            // check if left and right sticks move in the same direction
            if (gamepad1.right_stick_x * gamepad1.left_stick_x > 0) {

                float sideMovement = 0;
                if (gamepad1.right_stick_x > 0) {
                    sideMovement = max(gamepad1.left_stick_x, gamepad1.right_stick_x);
                } else {
                    sideMovement = min(gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
                mecanumBot.frontLeftDrive.setPower(-sideMovement);
                mecanumBot.frontRightDrive.setPower(sideMovement);
                mecanumBot.rearLeftDrive.setPower(sideMovement);
                mecanumBot.rearRightDrive.setPower(-sideMovement);
            }
        }
    }
}
