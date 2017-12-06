package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * Created by chrischen on 12/1/17.
 */

@TeleOp(name = "lifterTest Ver2")
//@Disabled

public class LifterTestVer2 extends OpMode {

    MMFileHandler fileHandler = new MMFileHandler();
    Context context;
    boolean isLifterButtonPressed;
    DcMotor blockLift;
    int target;
    int offset;
    boolean isMotorStalled = false;

    // enum for block lift
    public enum LiftPosition {
        LOW,
        HIGH,
        INIT_POSITION
    }

    LiftPosition liftPosition;
    int lastTick = 0;
    int lastEncoderPos = 0;
    double tStart = 0;

    @Override
    public void init() {
        initLiftMotor();
    }

    @Override
    public void loop() {
        blockLiftController();
    }

    public void blockLiftController() {
        // TODO: implement stall detection/timeout near LOW(bottom) and HIGH (top) positions
        // initialization, considering offset
        if (liftPosition == liftPosition.INIT_POSITION) {
            // first determine lift position, then determine the target position depending on dpad up/down
            int low = (int) CargoBotConstants.LOW_DISTANCE_FROM_START;
            int high = (int) CargoBotConstants.HIGH_DISTANCE_FROM_START;

            if (offset < low) {
                // only accept dpad up
                if (gamepad1.dpad_up) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.LOW;
                        isLifterButtonPressed = true;
                    }
                }
            } else if (offset >= low && offset < high) {
                // can accept dpad up or down
                if (gamepad1.dpad_up) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.HIGH;
                        isLifterButtonPressed = true;
                    }
                } else if (gamepad1.dpad_down) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.LOW;
                        isLifterButtonPressed = true;
                    }
                }

            } else {
                // offset >= high, only accept dpad down
                if (gamepad1.dpad_down) {
                    if (!isLifterButtonPressed) {
                        liftPosition = liftPosition.HIGH;
                        isLifterButtonPressed = true;
                    }
                }
            }
        }
        // normal operation
        if (gamepad1.dpad_up) {
            if (liftPosition == liftPosition.LOW && !isLifterButtonPressed) {
                liftPosition = liftPosition.HIGH;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to HIGH");
            }
            isMotorStalled = false;
        } else if (gamepad1.dpad_down) {
            if (liftPosition == liftPosition.HIGH && !isLifterButtonPressed) {
                liftPosition = liftPosition.LOW;
                isLifterButtonPressed = true;
                telemetry.addData("lift status", "to LOW");
            }
            isMotorStalled = false;
        }

        switch (liftPosition) {
            case LOW:
                target = (int) CargoBotConstants.LOW_DISTANCE_FROM_START - offset;
                break;
            case HIGH:
                target = (int) CargoBotConstants.HIGH_DISTANCE_FROM_START - offset;
                break;
            case INIT_POSITION:
                // robot is just initialized, don't move the lifter without user input
                break;
        }

        // don't move the lifter without user input
        if (liftPosition != LiftPosition.INIT_POSITION) {
            if (!isMotorStalled) {
                blockLift.setTargetPosition(target);
                blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (CargoBotConstants.SET_LIFT_MOTOR_HIGH_SPEED) {
                    // always lift up/down at high speed
                    blockLift.setPower(CargoBotConstants.LIFT_HI_SPEED);
                } else {
                    blockLift.setPower(CargoBotConstants.LIFT_SPEED);
                }
            }

            //telemetry.addData("lift encoder pos", blockLift.getCurrentPosition());
            //telemetry.update();
            if (!blockLift.isBusy()) {
                turnOffLiftMotor();
                reportLiftLogicalPos();
            } else {
                detectNeverRest20MotorStall();
            }
        }

        if (isLifterButtonPressed) {
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                isLifterButtonPressed = false;
            }
        }
    }


    private void turnOffLiftMotor() {
        blockLift.setPower(0.0);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void reportLiftLogicalPos() {
        switch (liftPosition) {
            case LOW:
                telemetry.addData("lift status", "@ LOW");
                break;
            case HIGH:
                telemetry.addData("lift status", "@ HIGH");
                break;
            case INIT_POSITION:
                telemetry.addData("lift status", "@ INIT_POSITION");
                break;
        }
    }


    private void detectNeverRest20MotorStall() {
        // create time base for checking if encoder count is not advancing much, which implies motor stall
        int tick = 0;
        // for every new request, set the last encoder position to the current position
        // as the basis for comparison
        // clear the motor stall flag to allow movement to other positions
        if (isLifterButtonPressed) {
            lastEncoderPos = blockLift.getCurrentPosition();
            tStart = getRuntime();
            lastTick = getRuntimeInTicks(tStart, CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD);
        }

        tick = getRuntimeInTicks(tStart, CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD);
        if (tick > lastTick) {
            // update for comparison in the next loop
            lastTick = tick;
            double time = CargoBotConstants.MOTOR_STALL_CHECKING_PERIOD;
            double motorSpeed;
            if (CargoBotConstants.SET_LIFT_MOTOR_HIGH_SPEED) {
                motorSpeed = CargoBotConstants.LIFT_HI_SPEED;
            } else {
                motorSpeed = CargoBotConstants.LIFT_SPEED;
            }
            int motorStallThreshold = (int) (getNeverest20MotorEncoderCountWithTime(time, motorSpeed) * CargoBotConstants.MOTOR_STALL_RATIO);
//            telemetry.addData("last encoder", lastEncoderPos);
//            telemetry.addData("current pos", blockLift.getCurrentPosition());
//            telemetry.addData("delta pos", abs(lastEncoderPos - blockLift.getCurrentPosition()));
//            telemetry.addData("motor stall thres", motorStallThreshold);

            // check the difference in encoder count at every tick
            if (abs(lastEncoderPos - blockLift.getCurrentPosition()) < motorStallThreshold) {
                turnOffLiftMotor();
                isMotorStalled = true;
            }
            // update for comparison in the next loop
            lastEncoderPos = blockLift.getCurrentPosition();

        }
    }

    private int getRuntimeInTicks(double tStart, double tick) {
        double runtime = getRuntime() - tStart;

        Double numTicks = runtime / tick;
        numTicks.intValue();

        return numTicks.intValue();
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /*
     * Calculate the theoretical encoder counts/pulses per tick using AndyMark's data
     * http://www.andymark.com/NeveRest-20-12V-Gearmotor-p/am-3102.htm
     * Data is measured at Voltage: 12 volt DC
     * No Load Free Speed, at gearbox output shaft: 275-315 RPM
     */
    private int getNeverest20MotorEncoderCountWithTime(double tick, double motorPower) {
        // limit the motorPower to [-1.0, 1.0]
        motorPower = Range.clip(motorPower, -CargoBotConstants.MOTOR_TOP_SPEED, CargoBotConstants.MOTOR_TOP_SPEED);
        int maxEncoderPulsesPerSec = (int) Math.round(motorPower * CargoBotConstants.ANDYMARK_20_MAX_COUNT_PER_SEC * getBatteryVoltage() / CargoBotConstants.MOTOR_VOLTAGE);
        int maxEncoderPulsesPerTick = (int) Math.round(maxEncoderPulsesPerSec * tick);

        return maxEncoderPulsesPerTick;
    }

    private void initLiftMotor(){
        liftPosition = LiftPosition.LOW;
        context = hardwareMap.appContext;
        if (fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, context).equals("error")) {
            offset = 0;
        } else {
            offset = fileHandler.stringToInt(fileHandler.readFromFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, context));
        }

        blockLift = hardwareMap.dcMotor.get("blockLiftVer2");

        blockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // configure motor direction
        blockLift.setDirection(DcMotor.Direction.REVERSE);
    }

    public void stop() {
        fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + blockLift.getCurrentPosition()), context);
    }
}
