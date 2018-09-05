/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.content.ContextWrapper;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
import android.util.AtomicFile;
import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static java.lang.Math.abs;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.sin;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="MM Autonomous", group ="Red")
//@Disabled
public class MMAutonomous extends LinearOpMode {

    HardwareMecanumCargoBot robot = new HardwareMecanumCargoBot();

    enum ALLIANCE_COLOR {
        RED,
        BLUE
    }

    ALLIANCE_COLOR alliance = ALLIANCE_COLOR.RED;


    private ElapsedTime runtime = new ElapsedTime();

    // configure motor encoder
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeverRest 40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    ENCODER_DRIVE state = ENCODER_DRIVE.START;
    boolean driveStatus = false;

    enum OPMODE_STEPS {
        STEP1,
        STEP2,
        STEP3,
        STEP4,
        STEP5,
        STEP6,
        STEP7,
        STEP8
    }
    OPMODE_STEPS opmodeState = OPMODE_STEPS.STEP1;

    private final int NAVX_DIM_I2C_PORT = 1;
    private AHRS navxDevice = null;
    private navXPIDController yawTurnPIDController = null;
    private navXPIDController yawDrivePIDController = null;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    double yawKp = MMShooterBotConstants.YAW_PID_P;
    boolean isNavxDeviceConnected = false;
    boolean timeoutNavxConnection = false;
    boolean isNavxMicroDataTimeout = false;
    double timeNavxDataTestTimeout = 0.0;
    private boolean calibration_complete = false;
    double Kp                   = 0.005;
    double distanceFromWall = 0.0;
    double colorOfFloor = 0.0;
    float hsvValues[] = {0F,0F,0F};

    private boolean isDoneRunningAuto = false;

    int target;
    int offset = 0;
    int OnRobotStop = 0;

    double timeRotationRequested;

    boolean isNavxRotateInitialized = false;
    Servo ArmServo;

    @Override public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        initializeNavx();
        ArmServo = hardwareMap.servo.get("Servo1");
        ArmServo.setPosition(0.9);

        while (!isStarted()) {
            telemetry.addData(">", "Robot Ready. Press Start.");
            telemetry.update();
            idle();
        }
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..


        waitForStart();
        // in case the robot is idle in init for a while
        navxDevice.zeroYaw();

        try {
//            while (!isDoneRunningAuto && !Thread.currentThread().isInterrupted()) {
              while (!Thread.currentThread().isInterrupted()) {

                // set up counters to be use for object detection
                int loopCounter = 0;

                while (!isDoneRunningAuto && opModeIsActive()) {
                    loopCounter++;
                    linearOpModeSteps();
                    reportNavxDriveTimeout();
                    idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
                }
            }
            telemetry.addData("End of While Loop","");
            telemetry.update();
        }
        catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } finally {
            onRobotStopOrInterrupt();
            telemetry.addData("Autonomous", "Interrupted");
            telemetry.update();
        }

    }

    private void initializeNavx() throws InterruptedException {
        //this initializes the navx gyro
        navxDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        // Create a PID Controller which uses the Yaw Angle as input. */
        yawTurnPIDController = new navXPIDController(navxDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        //yawTurnPIDController.setSetpoint(TARGET_ANGLE_DEGREES); let the app control this
        yawTurnPIDController.setContinuous(true);
        yawTurnPIDController.setOutputRange(MMShooterBotConstants.MIN_MOTOR_OUTPUT_VALUE, MMShooterBotConstants.MAX_MOTOR_OUTPUT_VALUE);
        yawTurnPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, MMShooterBotConstants.TOLERANCE_DEGREES);
        //yawTurnPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D); // let the app control this

        // Create a PID Controller which uses the Yaw Angle as input. */
        yawDrivePIDController = new navXPIDController(navxDevice,
                navXPIDController.navXTimestampedDataSource.YAW);
        yawDrivePIDController.setContinuous(true);
        yawDrivePIDController.setOutputRange(MMShooterBotConstants.MIN_MOTOR_OUTPUT_VALUE, MMShooterBotConstants.MAX_MOTOR_OUTPUT_VALUE);
        yawDrivePIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, MMShooterBotConstants.TOLERANCE_DEGREES);


        initNavxMicroCalibration(MMShooterBotConstants.NAVX_CONNECTION_TIMEOUT);

        // test if navx will timeout because of data is unavailable
        // if it times out, display the warning for 3 seconds to tell the driver to re-activate the configuration
        navxTestDataTimeout();
    }

    int getRuntimeInSeconds(double tStart){
        double runtime = getRuntime() - tStart;
        Double runtime_d = Math.floor(runtime);
        return runtime_d.intValue();
    }

    int getRuntimeInTicks(double tStart, double tick){
        double runtime = getRuntime() - tStart;

        Double numTicks = runtime/tick;
        numTicks.intValue();

        return numTicks.intValue();
    }

    enum ENCODER_DRIVE {
        START,
        LOOP,
        END
    }

    private enum ENUM_NAVX_GYRO_TURN {
        NOT_ARRIVED,
        ARRIVED,
        TIMEOUT
    }

    /*
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
    public boolean encoderDrive(double speed,
                                double leftInches, double rightInches,
                                double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        boolean driveComplete = false;

        // inverse direction due to motor configuration
        leftInches = -leftInches;
        rightInches = -rightInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            switch (state){
                case START:
                    // Determine new target position, and pass to motor controller
                    newLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    robot.frontLeftDrive.setTargetPosition(newLeftTarget);
                    robot.frontRightDrive.setTargetPosition(newRightTarget);
                    robot.rearLeftDrive.setTargetPosition(newLeftTarget);
                    robot.rearRightDrive.setTargetPosition(newRightTarget);

                    // Turn On RUN_TO_POSITION
                    robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.frontLeftDrive.setPower(Math.abs(speed));
                    robot.frontRightDrive.setPower(Math.abs(speed));
                    robot.rearLeftDrive.setPower(Math.abs(speed));
                    robot.rearRightDrive.setPower(Math.abs(speed));

                    // go to next state
                    state = ENCODER_DRIVE.LOOP;
                    break;

                case LOOP:
                    // keep looping while we are still active, and there is time left, and both motors are running.
                    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                    // its target position, the motion will stop.  This is "safer" in the event that the robot will
                    // always end the motion as soon as possible.
                    // However, if you require that BOTH motors have finished their moves before the robot continues
                    // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    if (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                        telemetry.addData("Path2",  "Running at %7d :%7d",
                                robot.frontLeftDrive.getCurrentPosition(),
                                robot.frontRightDrive.getCurrentPosition());
                        telemetry.update();
                    } else {
                        // go to next state
                        state = ENCODER_DRIVE.END;
                    }
                    break;

                case END:
                    // Stop all motion;
                    robot.frontLeftDrive.setPower(0);
                    robot.frontRightDrive.setPower(0);
                    robot.rearLeftDrive.setPower(0);
                    robot.rearRightDrive.setPower(0);

                    // Turn off RUN_TO_POSITION
                    robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    driveComplete = true;

                    // go back to initial state
                    state = ENCODER_DRIVE.START;
                    break;
            }

        }
        return driveComplete;
    }

    private void linearOpModeSteps() throws InterruptedException {

      //  telemetry.addData("I am in LinearOpMode","");

        switch (opmodeState) {
            case STEP1:  // Go straight 50 inches

                driveStatus = navxDrive(0.5,50.0,
                        calculateTimeout(50.0,0.5),
                        0);
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP2;
                }
                break;
            case STEP2: // Turn 90 degree
        //        telemetry.addData("Step:", opmodeState.toString());


                driveStatus = navxRotateToAngle(90, yawKp * 0.8);


                    if (driveStatus) {
                        opmodeState = OPMODE_STEPS.STEP3;
                    }
                break;

            case STEP3:  // Go Straight 50 inches

                 driveStatus = navxDrive(0.5,50.0,
                         calculateTimeout(50.0,0.5),
                        90);

                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP4;
                }
                break;


            case STEP4:  // Turn 180 degree
                driveStatus = navxRotateToAngle(180, yawKp * 0.8);

                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP5;
                }
                break;

            case STEP5:  // Check the distance of robot from wall. If < 75, move back

               distanceFromWall = robot.rangeSensor.getDistance(DistanceUnit.INCH);

               if (distanceFromWall < 75.0) {

                   driveStatus = navxDrive(0.5, distanceFromWall - 75.0,
                           calculateTimeout(distanceFromWall - 75.0,0.5),
                           180);

               }

                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP6;
                }
                break;


            case STEP6: // Check the color off the floor and display Hue value

                // convert the RGB values to HSV values.
                Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);

                telemetry.addData("Color of Floor", hsvValues[0]);
                telemetry.update();


                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP7;
                }
                break;

            case STEP7: // Bring the servo down and drive forward 5 inches

                ArmServo.setPosition(0.3);
                driveStatus = navxDrive(0.5, 5.0,
                        calculateTimeout(5.0,0.5),
                        180);
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP8;
                }
                break;

            case STEP8:
                // TODO: ensure when the last step is complete, call onRobotStopOrInterrupt() to terminate properly.
                 telemetry.addData("opmode state =",opmodeState);
                onRobotStopOrInterrupt();
                telemetry.addData("Autonomous", "Complete");
                telemetry.update();
                break;
        }
    }

    private void initNavxMicroCalibration(double timeoutS) {
        // if the connection to navx hardware is good
        double startTime = getRuntime();
        double elapsedTime = 0.0;

        while (!navxDevice.isConnected() && !timeoutNavxConnection) {
            // wait for device to be connected or time out
            elapsedTime = getRuntime() - startTime;
            if (elapsedTime >= timeoutS) {
                timeoutNavxConnection = true;
            }
            telemetry.addData("navX-Micro", "Connecting");
            telemetry.update();
            idle();
        }
        if (!timeoutNavxConnection) {
            isNavxDeviceConnected = true;
        }
        if (isNavxDeviceConnected) {
            while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
                calibration_complete = !navxDevice.isCalibrating();
                if (!calibration_complete) {
                    telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                    telemetry.update();
                }
            }
            navxDevice.zeroYaw();
        } else {
            telemetry.addData("navX-Micro", "Check Hardware Connection");
            telemetry.update();
        }
    }

    private void navxTestDataTimeout() throws InterruptedException{
        double angle = 0;
        // set the parameters before enabling the PID controller
        yawTurnPIDController.setSetpoint(angle);
        yawTurnPIDController.setPID(Kp, MMShooterBotConstants.YAW_PID_I, MMShooterBotConstants.YAW_PID_D);
        yawTurnPIDController.enable(true);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        Thread.sleep(MMShooterBotConstants.SLEEP_MS);
        // this instruction blocks the thread, and won't return immediately
        // return true if new data is available
        // return false if it times out.
        if (yawTurnPIDController.waitForNewUpdate(yawPIDResult, MMShooterBotConstants.WAIT_FOR_UPDATE_TIMEOUT_MS)) {
            // no issues, data is available
            isNavxMicroDataTimeout = false;
        } else {
            // data timeout occurs
            isNavxMicroDataTimeout = true;
            timeNavxDataTestTimeout = getRuntime();
        }

        // navx data timeout, warn the driver to stop the app, and re-activate the configuration. Then re-start the app.
        while ((isNavxMicroDataTimeout) && ((getRuntime() - timeNavxDataTestTimeout) < MMShooterBotConstants.TIME_DISPLAY_NAVX_TIMEOUT_MSG)) {
            telemetry.addData(">", "Stop the app. Re-activate ftc7047shooterBot configuration. Then re-start.");    //
            telemetry.update();
        }
    }

    private boolean navxRotateToAngle(double angle, double Kp) throws InterruptedException{
        ENUM_NAVX_GYRO_TURN rotationStatus = ENUM_NAVX_GYRO_TURN.NOT_ARRIVED;
        double angleNormalized = -angle; // reverse the angle's direction: since positive is for CCW, negative is for CW

        if (!isNavxRotateInitialized) {
            // set the parameters before enabling the PID controller
            // set tolerance for angle
            yawTurnPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, CargoBotConstants.ANGLE_TOLERANCE);
            yawTurnPIDController.setSetpoint(angleNormalized);
            yawTurnPIDController.setPID(Kp, MMShooterBotConstants.YAW_PID_I, MMShooterBotConstants.YAW_PID_D);
            yawTurnPIDController.enable(true);
            timeRotationRequested = getRuntime();
            // set to true so that init only runs once per request
            isNavxRotateInitialized = true;
        }

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        DecimalFormat df = new DecimalFormat("#.##");
        // this step is necessary for this method to work
        Thread.sleep(MMShooterBotConstants.SLEEP_MS);

        if (yawTurnPIDController.isNewUpdateAvailable(yawPIDResult)) {

            if (yawPIDResult.isOnTarget()) {
                robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turnOffDriveMotors();
                rotationStatus = ENUM_NAVX_GYRO_TURN.ARRIVED;
                if (MMShooterBotConstants.displayNavXTelemetryTeleOp) {
                    telemetry.addData("PIDOutput", df.format(0.00));
                }
            } else {
                double output = yawPIDResult.getOutput();
                // scale the output so that the robot is not turning too fast
                output = output * CargoBotConstants.SPEED_RATIO;

                // amplify the output of PID controller as it gets close to the target
                // otherwise, it would not finish turning in time
                if (Math.abs(output) <= MMShooterBotConstants.NAVX_PID_LOW_OUTPUT_THRESHOLD) {
                    output = output * MMShooterBotConstants.MULTIPLER_WHEN_NAVX_LOW_OUTPUT;
                }

                robot.frontLeftDrive.setPower(-output);
                robot.frontRightDrive.setPower(output);
                robot.rearLeftDrive.setPower(-output);
                robot.rearRightDrive.setPower(output);
                if (MMShooterBotConstants.displayNavXTelemetryTeleOp) {
                    telemetry.addData("PIDOutput", df.format(output) + ", " +
                            df.format(-output));
                }
            }

            if (MMShooterBotConstants.displayNavXTelemetryTeleOp) {
                telemetry.addData("Yaw", df.format(-navxDevice.getYaw())); // add negative to convert to robot's heading
            }
        } else {
            // check for timeout waiting for navx data to update
            // note: this is not a timeout for the rotation, but for data availability.
            if ((getRuntime() - timeRotationRequested) >= MMShooterBotConstants.TIMEOUT_WAIT_FOR_NAVX_DATA) {
                // give up and return time out status
                rotationStatus = ENUM_NAVX_GYRO_TURN.TIMEOUT;
            }

        }
        if (rotationStatus == ENUM_NAVX_GYRO_TURN.ARRIVED) {
            isNavxRotateInitialized = false;
            return true;
        } else {
            return false;
        }
    }

    public boolean navxDrive( double speed,
                              double distance,
                              double timeoutS,
                              double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        boolean driveComplete = false;
        // negation for mecanum drive arrangement
        distance = -distance;

        // set up navx stuff
        double angleNormalized = -angle; // reverse the angle's direction: since positive is for CCW, negative is for CW
        // set the parameters before enabling the PID controller
        yawDrivePIDController.enable(true);
        yawDrivePIDController.setSetpoint(angleNormalized);
        yawDrivePIDController.setPID(Kp, MMShooterBotConstants.YAW_PID_I, MMShooterBotConstants.YAW_PID_D);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        Thread.sleep(MMShooterBotConstants.SLEEP_MS);
        // according to documentation, this instruction blocks the thread, and won't return immediately
        // it returns true if new data is available; false if it times out.
        if (yawDrivePIDController.waitForNewUpdate(yawPIDResult, MMShooterBotConstants.WAIT_FOR_UPDATE_TIMEOUT_MS)) {
            // proceed to keep using data from navx-micro
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int)(distance * MMShooterBotConstants.COUNTS_PER_INCH);
                newLeftTarget = robot.frontLeftDrive.getCurrentPosition() + moveCounts;
                newRightTarget = robot.frontRightDrive.getCurrentPosition() + moveCounts;
                newLeftTarget = robot.rearLeftDrive.getCurrentPosition() + moveCounts;
                newRightTarget = robot.rearRightDrive.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.frontLeftDrive.setTargetPosition(newLeftTarget);
                robot.frontRightDrive.setTargetPosition(newRightTarget);
                robot.rearLeftDrive.setTargetPosition(newLeftTarget);
                robot.rearRightDrive.setTargetPosition(newRightTarget);

                robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);

                runtime.reset();
                // for debugging only, activate it when necessary
                //RobotLog.vv(MMShooterBotConstants.GYRO_DRIVE_TAG, ",run time after reset =,%5.2f, seconds", runtime.seconds());

                // keep looping while we are still active, and either motors are running.
                while (opModeIsActive() && robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy()
                        && robot.rearLeftDrive.isBusy() && robot.rearRightDrive.isBusy() &&
                        (runtime.seconds() < timeoutS)){
                    if (yawDrivePIDController.isNewUpdateAvailable(yawPIDResult)) {
                        if (yawPIDResult.isOnTarget()) {
                            robot.frontLeftDrive.setPower(speed);
                            robot.frontRightDrive.setPower(speed);
                            robot.rearLeftDrive.setPower(speed);
                            robot.rearRightDrive.setPower(speed);
//                            telemetry.addData("PIDOutput", df.format(speed) + ", " +
//                                    df.format(speed));
                        } else {
                            double output = yawPIDResult.getOutput();
                            // if driving in reverse, the motor correction also needs to be reversed
                            if (distance < 0)
                                output *= -1.0;

                            robot.frontLeftDrive.setPower(speed - output);
                            robot.frontRightDrive.setPower(speed + output);
                            robot.rearLeftDrive.setPower(speed - output);
                            robot.rearRightDrive.setPower(speed + output);
//                            telemetry.addData("PIDOutput", df.format(limit(speed + output)) + ", " +
//                                    df.format(limit(speed - output)));
                        }
//                        telemetry.addData("Yaw", df.format(navxDevice.getYaw()));
                    }
//                    telemetry.update();

                    // for debugging only, activate it when necessary
                    //RobotLog.vv(MMShooterBotConstants.GYRO_DRIVE_TAG, ",run time =,%5.2f, seconds", runtime.seconds());
                }

                // stop the momentum of the robot
                turnOffDriveMotors();
                driveComplete = true;
            }

        } else {
            // time out occurs, fall back to use modern robotics gyro and change the mission route
            RobotLog.vv("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            isNavxMicroDataTimeout = true;
        }
        yawDrivePIDController.enable(false);
        return driveComplete;
    }

    public void turnOffDriveMotors() {
        // Stop all motion;
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        // It is critical to stop and reset the encoder before the next run
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double calculateTimeout(double distance, double speed) {
        double timeoutS = 0.0;
        timeoutS = distance/speed * MMShooterBotConstants.TIMEOUT_SCALING_FACTOR;
        timeoutS = timeoutS * CargoBotConstants.TIMEOUT_CORRECTION_FACTOR;
        // make sure we return a positive value;
        // and add 1 second margin
        return (Math.abs(timeoutS) + 1);
    }


    private void onRobotStopOrInterrupt(){

       isDoneRunningAuto = true;
       OnRobotStop++;
        if (navxDevice != null) {

            navxDevice.close();
            telemetry.addData("After NacxDevice-->Close #", OnRobotStop);
            telemetry.update();
            sleep(2000);
        }

        turnOffDriveMotors();
        // save the block lift position for next operation
        // robot.fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + robot.blockLift.getCurrentPosition()), robot.context);
        // save the alliance color for teleop
          robot.fileHandler.writeToFile("alliance_color.txt", CargoBotConstants.pathToAllianceColor, "red", robot.context);
    }
    private void reportNavxDriveTimeout() {
        if (isNavxMicroDataTimeout){
            telemetry.addData("navxDrive", "Yaw PID waitForNewUpdate() TIMEOUT");
            telemetry.update();
        }
    }
}
