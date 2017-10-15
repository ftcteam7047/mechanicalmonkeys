package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.content.Context;
import android.view.ViewDebug;

/**
 * Created by michaelchen on 10/8/17.
 */

@TeleOp(name = "Encoder Test")
//@Disabled

public class EncoderTest extends OpMode {

    DcMotor test;
    MMFileHandler fileHandler = new MMFileHandler();
    int offset;
    Context context;
    int target;

    @Override
    public void init() {

        context = hardwareMap.appContext;
        test = hardwareMap.dcMotor.get("test");
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        test.setPower(0.0);
        //
        if (fileHandler.readFromFile("offset.txt", context).equals("error")){
            offset = 0;
        } else {
            offset = fileHandler.stringToInt(fileHandler.readFromFile("offset.txt", context));
        }
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            target = 1000 - offset;
            test.setTargetPosition(target);
            test.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            test.setPower(0.1);
            while (test.isBusy()) {
                telemetry.addData("pos", test.getCurrentPosition());
                telemetry.update();
            }
        }
        if (gamepad1.b) {
            target = 0 - offset;
            test.setTargetPosition(target);
            test.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            test.setPower(0.1);
            while (test.isBusy()) {
                telemetry.addData("pos", test.getCurrentPosition());
                telemetry.update();
            }
        }



        test.setPower(0.0);
        telemetry.addData("pos", test.getCurrentPosition());
        telemetry.update();

        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop() {
        fileHandler.writeToFile("offset.txt", Integer.toString(offset + test.getCurrentPosition()), context);
    }
}
