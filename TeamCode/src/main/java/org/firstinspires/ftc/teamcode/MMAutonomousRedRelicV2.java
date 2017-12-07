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
import android.os.Environment;
import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@Autonomous(name="Red Relic V2", group ="Red")
//@Disabled
public class MMAutonomousRedRelicV2 extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    // Configuration values for tiny-yolo-voc. Note that the graph is not included with TensorFlow and
    // must be manually placed in the assets/ directory by the user.
    // Graphs and models downloaded from http://pjreddie.com/darknet/yolo/ may be converted e.g. via
    // DarkFlow (https://github.com/thtrieu/darkflow). Sample command:
    // ./flow --model cfg/tiny-yolo-voc.cfg --load bin/tiny-yolo-voc.weights --savepb --verbalise
    private static final String YOLO_MODEL_FILE = "file:///android_asset/optimized_8_bit_quantized_416_v5.pb";
    private static final int YOLO_INPUT_SIZE = 416;
    private static final String YOLO_INPUT_NAME = "input";
    private static final String YOLO_OUTPUT_NAMES = "output";
    private static final int YOLO_BLOCK_SIZE = 32;

    // Configuration values for the prepackaged multibox model.
    private static final int MB_INPUT_SIZE = 224;
    private static final int MB_IMAGE_MEAN = 128;
    private static final float MB_IMAGE_STD = 128;
    private static final String MB_INPUT_NAME = "ResizeBilinear";
    private static final String MB_OUTPUT_LOCATIONS_NAME = "output_locations/Reshape";
    private static final String MB_OUTPUT_SCORES_NAME = "output_scores/Reshape";
    private static final String MB_MODEL_FILE = "file:///android_asset/multibox_model.pb";
    private static final String MB_LOCATION_FILE =
            "file:///android_asset/multibox_location_priors.txt";
    private static final int TF_OD_API_INPUT_SIZE = 300;
    private static final String TF_OD_API_MODEL_FILE =
            "file:///android_asset/ssd_mobilenet_v1_android_export.pb";
    private static final String TF_OD_API_LABELS_FILE = "file:///android_asset/coco_labels_list.txt";


    // Which detection model to use: by default uses Tensorflow Object Detection API frozen
    // checkpoints.  Optionally use legacy Multibox (trained using an older version of the API)
    // or YOLO.
    private enum DetectorMode {
        TF_OD_API, MULTIBOX, YOLO;
    }
    private static final DetectorMode MODE = DetectorMode.YOLO;

    // Minimum detection confidence to track a detection.
    private static final float MINIMUM_CONFIDENCE_TF_OD_API = 0.6f;
    private static final float MINIMUM_CONFIDENCE_MULTIBOX = 0.1f;
    private static final float MINIMUM_CONFIDENCE_YOLO = 0.15f;
    private Classifier detector;
    ArrayList<String> detectionResult = new ArrayList<String>();
    private static final int MINIMUM_NUM_LOOPS = 50; // approximately 2 frame/sec, since 1 loop per 10ms
    int cropSize;
    float minimumConfidence;
    private enum BALL_PREDICTION_RESULT {
        UNKNOWN,
        RED_ON_LEFT,
        BLUE_ON_LEFT
    }
    BALL_PREDICTION_RESULT prediction = BALL_PREDICTION_RESULT.UNKNOWN;
    String predictionString = "unknown";
    boolean isProcessingFrame = false;
    Context appContext = null;
    private static final double PERIOD_PER_TICK = 0.5; // in seconds
    boolean isFirstLoop = true;


    HardwareMecanumCargoBotV2 robot = new HardwareMecanumCargoBotV2();

    enum ALLIANCE_COLOR {
        RED,
        BLUE
    }

    ALLIANCE_COLOR alliance = ALLIANCE_COLOR.RED;

    enum STARTING_POSITION {
        RELIC,
        TIP
    }

    public enum LiftPosition {
        GRAB,
        MOVE,
        STACK,
        PLACE
    }
    LiftPosition liftPosition = LiftPosition.GRAB;

    enum GripperPosition {
        OPEN,
        CLOSE,
        WIDE_OPEN
    }

    GripperPosition gripperPosition = GripperPosition.CLOSE;

    STARTING_POSITION startingPosition = STARTING_POSITION.RELIC;

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
    boolean liftStatus = false;

    enum OPMODE_STEPS {
        STEP1,
        STEP2,
        STEP3,
        STEP4,
        STEP5,
        STEP6,
        STEP7,
        STEP8,
        STEP9,
        STEP10,
        STEP11,
        STEP12
    }
    OPMODE_STEPS opmodeState = OPMODE_STEPS.STEP1;

    private final int NAVX_DIM_I2C_PORT = 5;
    private AHRS navxDevice = null;
    private navXPIDController yawTurnPIDController = null;
    private navXPIDController yawDrivePIDController = null;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    double yawKp = MMShooterBotConstants.YAW_PID_P;
    boolean isNavxDeviceConnected = false;
    boolean timeoutNavxConnection = false;
    boolean didRetryIngnterceptingWhiteLine = false;
    boolean isNavxMicroDataTimeout = false;
    double timeNavxDataTestTimeout = 0.0;
    private boolean calibration_complete = false;
    double Kp                   = 0.005;
	
	private boolean isDoneRunningAuto = false;

    int target;
    int offset = 0;

    ExecutorService cameraExecutorService = Executors.newFixedThreadPool(1);

    ArrayList<BALL_PREDICTION_RESULT> predictions = new ArrayList<BALL_PREDICTION_RESULT>();

    double drivingOffPlatformOffset;

    double timeRotationRequested;

    boolean isNavxRotateInitialized = false;

    double stepFiveStartTime;

    double tX = 0;
    double tY = 0;
    double tZ = 0;

    // Extract the rotational components of the target relative to the robot (phone)
    double rX = 0;
    double rY = 0;
    double rZ = 0;

    boolean isRobotPositionAvailable = false;

    @Override public void runOpMode() throws InterruptedException {
        try {
            while (!isDoneRunningAuto && !Thread.currentThread().isInterrupted()) {

                robot.init(hardwareMap);

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

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

                // OR...  Do Not Activate the Camera Monitor View, to save power
                // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
                parameters.vuforiaLicenseKey = "AaukgGT/////AAAAGWkWAT0TG0LEo/kIYGHRbONkJEGTvNn6Q1Urj56aIj3ewuw2apA6gNe8KvZxdfL42j2CQo+4CAp0xeKL1zXwrc7wJjNeyavmXD0dUI32L8ssp5QiCAtPXF7uwqLm+Q/1vWD5w5NpD+2l1yCpsZgwhF0IKBlgJmwTZpouOzjSMcGFz5H3ERq/n8b405blFDI4unS6C1BMUIQVhQgFZe2PUrslIMeopgc3MyHIzW5nNhwX161k99VxXCdMPp+qKwP0N4Wj+aVwdGSXzwnX4pwzLdDTh4IsQNHlNaWzT/mv8hIQ703Cxf/L5DiI8KBnLjG9jOtlPXxu9NULXMFZf9qBB8tDnqY5EvkF+gt8n0BHUmNq";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                // create context to be used with threads
                Context context;
                context = hardwareMap.appContext;
                appContext = context;

                // initialize tensor flow object detection
                 initDetection(context);

                // time management
                int tick = 0;
                int lastTick = 0;
                /**
                 * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
                 * in this data set: all three of the VuMarks in the game were created from this one template,
                 * but differ in their instance id information.
                 * @see VuMarkInstanceId
                 */
                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                // Wait for the game to start (Display Gyro value), and reset gyro before we move..
                while (!isStarted()) {
                    telemetry.addData(">", "Robot Ready. Press Start.");
                    telemetry.update();
                    idle();
                }

                waitForStart();
                // in case the robot is idle in init for a while
                navxDevice.zeroYaw();

                double tStart = getRuntime();
                relicTrackables.activate();

                // set up frame format to be used for object detection
                vuforia.setFrameQueueCapacity(1);
                Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

                // set up counters to be use for object detection
                int loopCounter = 0;
                int frameCounter = 0;
//                ExecutorService cameraExecutorService = Executors.newFixedThreadPool(1);

                while (opModeIsActive()) {
                    VuforiaLocalizer.CloseableFrame frame = null;
                    tick = getRuntimeInTicks(tStart, PERIOD_PER_TICK);
                    if ((tick > lastTick) || isFirstLoop) {
                        // update for comparison in the next loop
                        lastTick = tick;
                        isFirstLoop = false;

                        frame = vuforia.getFrameQueue().take();
                        if (frame != null) {
                            Image img = getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
                            if (img != null) {
                                frameCounter++;
                                final Bitmap bm = getBitmapFromImage(img);

                                if (!isProcessingFrame) {
                                    cameraExecutorService.execute(new Runnable() {
                                        @Override
                                        public void run() {
                                            isProcessingFrame = true;
                                            processBitmap(bm);
                                            isProcessingFrame = false;
                                        }
                                    });
                                }


                                RobotLog.vv("detection status", ",%s,", detectionResult.toString());
                                telemetry.addData("detection status", detectionResult.toString());
                                telemetry.addData("prediction", predictionString);
                                telemetry.addData("frame counter", frameCounter);
                                telemetry.update();
//                        String absPath = saveToInternalStorage(bm, context, imgCounter);
//                        imgCounter++;
//                        telemetry.addData("vuforia image saved path", absPath);
//                        telemetry.update();
                            }

                        }
                    }
                    loopCounter++;


                    /**
                     * See if any of the instances of {@link relicTemplate} are currently visible.
                     * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                     * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                     * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                     */
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                        //telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                        //telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                        if (pose != null) {
                            VectorF trans = pose.getTranslation();
                            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                            // Extract the X, Y, and Z components of the offset of the target relative to the robot
                            tX = trans.get(0);
                            tY = trans.get(1);
                            tZ = trans.get(2);

                            // Extract the rotational components of the target relative to the robot
                            rX = rot.firstAngle;
                            rY = rot.secondAngle;
                            rZ = rot.thirdAngle;
                            isRobotPositionAvailable = true;
                        } else {
                            isRobotPositionAvailable = false;
                        }
                    } else {
                        isRobotPositionAvailable = false;
                        //telemetry.addData("VuMark", "not visible");
                    }

                    //telemetry.update();

                    // checks what vuMark is currently visible and sets a variable
                    // to keep track and locks it in if it is detected
                    if (vuMarkIdentified == VU_MARK_TYPE.UNKNOWN) {
                        switch (vuMark) {
                            case LEFT:
                                vuMarkIdentified = VU_MARK_TYPE.LEFT;
                                break;
                            case CENTER:
                                vuMarkIdentified = VU_MARK_TYPE.CENTER;
                                break;
                            case RIGHT:
                                vuMarkIdentified = VU_MARK_TYPE.RIGHT;
                                break;
                            case UNKNOWN:
                                break;
                        }
                    }

                    // step through linear opmode steps
                    linearOpModeSteps();

                    // run loops every 10 ms
                    //sleep(10);

                    idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
                }
            }
//            cameraExecutorService.shutdown();
        }
        catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } finally {
//            if (!cameraExecutorService.isShutdown()) {
//                cameraExecutorService.shutdown();
//            }
            onRobotStopOrInterrupt();

            telemetry.addData("Autonomous", "Interrupted");
            telemetry.update();
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat){
        long numImgs = frame.getNumImages();
        telemetry.addData("Acquird images in a frame", numImgs);
        for (int i = 0; i < numImgs; i++){
            if (frame.getImage(i).getFormat() == pixelFormat){
                return  frame.getImage(i);
            }
        }
        telemetry.addData("status ", "no images acquired");
        telemetry.update();
        return  null;
    }

    public Bitmap getBitmapFromImage(Image img){

        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());
        return bm;
    }

    private void initDetection(Context context) {

        if (MODE == DetectorMode.YOLO) {
            telemetry.addData("Status", "initializing YOLO Classifier");
            telemetry.update();
            detector =
                    TensorFlowYoloDetector.create(
                            context.getAssets(),
                            YOLO_MODEL_FILE,
                            YOLO_INPUT_SIZE,
                            YOLO_INPUT_NAME,
                            YOLO_OUTPUT_NAMES,
                            YOLO_BLOCK_SIZE);
            cropSize = YOLO_INPUT_SIZE;
            telemetry.addData("Status", "YOLO Classifier initialized");
            telemetry.update();
        } else if (MODE == DetectorMode.MULTIBOX) {
            detector =
                    TensorFlowMultiBoxDetector.create(
                            context.getAssets(),
                            MB_MODEL_FILE,
                            MB_LOCATION_FILE,
                            MB_IMAGE_MEAN,
                            MB_IMAGE_STD,
                            MB_INPUT_NAME,
                            MB_OUTPUT_LOCATIONS_NAME,
                            MB_OUTPUT_SCORES_NAME);
            cropSize = MB_INPUT_SIZE;
        } else {
            try {
                telemetry.addData("Status", "initializing the TF_OD_API_MODEL Classifier");
                telemetry.update();
                detector = TensorFlowObjectDetectionAPIModel.create(
                        context.getAssets(), TF_OD_API_MODEL_FILE, TF_OD_API_LABELS_FILE, TF_OD_API_INPUT_SIZE);
                cropSize = TF_OD_API_INPUT_SIZE;
                telemetry.addData("Status", "TF_OD_API_MODEL Classifier is initialized");
                telemetry.update();
            } catch (final IOException e) {
                //LOGGER.e("Exception initializing classifier!", e);
                telemetry.addData("Error", "Classifier TF_OD_API_MODEL could not be initialized");
                telemetry.update();
            }
        }
        minimumConfidence = MINIMUM_CONFIDENCE_TF_OD_API;
        switch (MODE) {
            case TF_OD_API:
                minimumConfidence = MINIMUM_CONFIDENCE_TF_OD_API;
                break;
            case MULTIBOX:
                minimumConfidence = MINIMUM_CONFIDENCE_MULTIBOX;
                break;
            case YOLO:
                minimumConfidence = MINIMUM_CONFIDENCE_YOLO;
                break;
        }

    }

    private String saveToInternalStorage(Bitmap bitmapImage, Context context, int sequenceNum) {
        ContextWrapper cw = new ContextWrapper(context);
        // path to /data/data/yourapp/app_data/imageDir
        File directory = cw.getDir("imageDir", Context.MODE_PRIVATE);

        File path = context.getFilesDir();
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File(sdCard.getAbsolutePath() + "/FTC/vuforia");
        dir.mkdirs();
        File filePath = null;
        String filename = "vuforia_frame" + Integer.toString(sequenceNum) + ".jpg";

        if (isExternalStorageReadable()) {
            filePath = new File(dir, filename);  // external storage on micro sd card
        } else {
            filePath = new File(path, filename); // app's local storage
        }

        FileOutputStream fos = null;
        try {
            fos = new FileOutputStream(filePath);
            // Use the compress method on the BitMap object to write image to the OutputStream
            bitmapImage.compress(Bitmap.CompressFormat.PNG, 100, fos);
            telemetry.addData("file status", "saving image from vuforia");
            telemetry.update();
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addData("file status", "error occurred");
        } finally {
            try {
                fos.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return directory.getAbsolutePath();
    }

    /* Checks if external storage is available to at least read */
    private boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
            return true;
        }
        return false;
    }


    public void processBitmap(final Bitmap input) {
        Bitmap b, b2;
        double startTime = 0.0;
        startTime = getRuntime();
        final ArrayList<String> result = new ArrayList<String>();

        b = BitmapUtils.resizeBitmap(input, cropSize);
        b2 = BitmapUtils.rotateBitmap(b);
        List<Classifier.Recognition> r = detector.recognizeImage(b2);

        for (Classifier.Recognition recog : r) {
            Log.d("Recognition", recog.toString());
            float scalar = 1.0f;
            if (!recog.getTitle().equals("???")) {
                if (recog.getConfidence() >= minimumConfidence) {
                    float leftEdge = recog.getLocation().left * scalar;
                    result.add(recog.getTitle() + ": " + recog.getConfidence().toString() + "leftEdge" + Float.toString(leftEdge));
                }
            }
        }
        final double endTime = getRuntime() - startTime;

        Activity ftcAppActivity = (Activity) appContext;
        ftcAppActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                result.add(Double.toString(endTime));
                detectionResult = searchDetectionResult(result);
                prediction = decodeDetectionResult(detectionResult);
                switch (prediction) {
                    case UNKNOWN:
                        predictionString = "unknown";
                        break;
                    case RED_ON_LEFT:
                        predictionString = "[Red: left  Blue: right]";
                        break;
                    case BLUE_ON_LEFT:
                        predictionString = "[Blue: left  Red: right]";
                        break;
                    default:
                        predictionString = "unknown";
                        break;
                }
            }
        });
    }


    private ArrayList<String> searchDetectionResult(ArrayList<String> results) {
        ArrayList<String> detectedObjects = new ArrayList<String>();

        for(String result : results) {
            //if(result.equals("keyboard") || result.equals("laptop") || result.equals("mouse") || result.equals("person")) {
            if (!result.equals("???")){
                detectedObjects.add(result);
            }
        }
        if (detectedObjects == null){
            detectedObjects.add("noKnownObjects");
        }
        return  detectedObjects;
    }

    private BALL_PREDICTION_RESULT decodeDetectionResult(ArrayList<String> results){
        BALL_PREDICTION_RESULT pred = BALL_PREDICTION_RESULT.UNKNOWN;
        Float ballLeftEdge[]=new Float[2];
        ArrayList<String> ballList = new ArrayList<String>();

        String object = "";
        String leftEdge = "";
        for (String result : results) {
            // parse the object
            if (result.contains(":")) {
                String[] temp1 = result.split(":");
                object = temp1[0];
            }
            // parse the object's left edge of its bounding box
            if (result.contains("leftEdge")){
                String[] temp2 = result.split("leftEdge");
                leftEdge = temp2[1];
            }

            if (object.equals("redball")) {
                if (!ballList.contains("redball")) {
                    ballList.add("redball");
                    // position 0 stores the left edge of redball
                    ballLeftEdge[0] = Float.parseFloat(leftEdge);
                }
            }
            if (object.equals("blueball")) {
                if (!ballList.contains("blueball")) {
                    ballList.add("blueball");
                    // position 1 stores the left edge of blueball
                    ballLeftEdge[1] = Float.parseFloat(leftEdge);
                }
            }

        }
        // if the list size is 2, then we have both red and blue result
        // now we are ready to compare them.
        // However, as long there is one prediction available, red or blue,
        // we can figure out which one is on the left.
        if (ballList.size() == 2) {
            if (ballLeftEdge[0] < ballLeftEdge[1]) {
                pred = BALL_PREDICTION_RESULT.RED_ON_LEFT;
            } else {
                pred = BALL_PREDICTION_RESULT.BLUE_ON_LEFT;
            }
        } else if (ballList.size() == 1) {
            if (ballList.get(0).equals("redball")) {
                // check redball's left edge relative to the frame
                if (ballLeftEdge[0] <= (cropSize * 1.0 / 2)) {
                    pred = BALL_PREDICTION_RESULT.RED_ON_LEFT;
                } else {
                    pred = BALL_PREDICTION_RESULT.BLUE_ON_LEFT;
                }
            } else {
                // check blueball's left edge relative to the frame
                if (ballLeftEdge[1] <= (cropSize * 1.0 / 2)) {
                    pred = BALL_PREDICTION_RESULT.BLUE_ON_LEFT;
                } else {
                    pred = BALL_PREDICTION_RESULT.RED_ON_LEFT;
                }
            }
        } else {
            pred = BALL_PREDICTION_RESULT.UNKNOWN;
        }

        return pred;
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

    enum VU_MARK_TYPE {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }
    VU_MARK_TYPE vuMarkIdentified = VU_MARK_TYPE.UNKNOWN;
    ArrayList<Double> yaws = new ArrayList<Double>();

    enum INTAKE_DIR {
        IN,
        OUT
    }

    enum ENUM_INTAKE_CONTROLLER_STATE {
        START,
        RUN,
        END
    }
    ENUM_INTAKE_CONTROLLER_STATE intakeControllerState = ENUM_INTAKE_CONTROLLER_STATE.START;
    double intakeMotorRunTime = 0.0;

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
        switch (opmodeState) {
            case STEP1:
                // Lowers ball arm
                robot.ballArm.setPosition(CargoBotConstants.BALL_ARM_DOWN);
                opmodeState = OPMODE_STEPS.STEP2;
                break;
            case STEP2:
                // Move backwards or forwards to knock off ball
                driveStatus = false;
                if (prediction != BALL_PREDICTION_RESULT.UNKNOWN) {
                    if (startingPosition == STARTING_POSITION.RELIC) {
                        if (alliance == ALLIANCE_COLOR.RED) {
                            if (prediction == BALL_PREDICTION_RESULT.RED_ON_LEFT) {
                                // move forward or turn right
                                //telemetry.addData("detection result", "red on left");
                                //sleep(5000);
                                predictions.add(prediction);
                                driveStatus = navxDrive(CargoBotConstants.BALL_SPEED,
                                        CargoBotConstants.BALL_DISTANCE,
                                        calculateTimeout(CargoBotConstants.BALL_DISTANCE,
                                                CargoBotConstants.BALL_SPEED),
                                        0);
                                drivingOffPlatformOffset = CargoBotConstants.DRIVE_OFF_PLATFORM_MORE_OFFSET;
//                                driveStatus = encoderDrive(CargoBotConstants.BALL_SPEED,
//                                        CargoBotConstants.BALL_DISTANCE,
//                                        CargoBotConstants.BALL_DISTANCE,
//                                        5);
                            } else {
                                // move back or turn left
                                //telemetry.addData("detection result", "red on right");
                                //sleep(5000);
                                predictions.add(prediction);
                                driveStatus = navxDrive(CargoBotConstants.BALL_SPEED,
                                        -CargoBotConstants.BALL_DISTANCE,
                                        calculateTimeout(CargoBotConstants.BALL_DISTANCE,
                                                CargoBotConstants.BALL_SPEED),
                                        0);
                                drivingOffPlatformOffset = CargoBotConstants.DRIVE_OFF_PLATFORM_LESS_OFFSET;
//                                driveStatus = encoderDrive(CargoBotConstants.BALL_SPEED,
//                                        -CargoBotConstants.BALL_DISTANCE,
//                                        -CargoBotConstants.BALL_DISTANCE,
//                                        5);
                            }
                        } else {
                            if (prediction == BALL_PREDICTION_RESULT.RED_ON_LEFT) {
                                // move forward or turn right
                                driveStatus = navxDrive(CargoBotConstants.BALL_SPEED,
                                        -CargoBotConstants.BALL_DISTANCE,
                                        calculateTimeout(CargoBotConstants.BALL_DISTANCE,
                                                CargoBotConstants.BALL_SPEED),
                                        0);
                                drivingOffPlatformOffset = CargoBotConstants.DRIVE_OFF_PLATFORM_MORE_OFFSET;
//                                driveStatus = encoderDrive(CargoBotConstants.BALL_SPEED,
//                                        -CargoBotConstants.BALL_DISTANCE,
//                                        -CargoBotConstants.BALL_DISTANCE,
//                                        5);
                            } else {
                                // move back or turn left
                                driveStatus = navxDrive(CargoBotConstants.BALL_SPEED,
                                        CargoBotConstants.BALL_DISTANCE,
                                        calculateTimeout(CargoBotConstants.BALL_DISTANCE,
                                                CargoBotConstants.BALL_SPEED),
                                        0);
                                drivingOffPlatformOffset = CargoBotConstants.DRIVE_OFF_PLATFORM_LESS_OFFSET;
//                                driveStatus = encoderDrive(CargoBotConstants.BALL_SPEED,
//                                        CargoBotConstants.BALL_DISTANCE,
//                                        CargoBotConstants.BALL_DISTANCE,
//                                        5);
                            }
                        }
                    }
                } else {
                    telemetry.addData("detection result", "unknown");
                }
                telemetry.addData("detection list", predictions.toString());
                telemetry.update();
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP3;
                    robot.ballArm.setPosition(CargoBotConstants.BALL_ARM_UP);
                }
                break;
            case STEP3:
                // Get off the platform and stop (to view VuMark)
                // positive going forward negative go backward
                // leaving platform always use negative for red and leaving platform for blue always use positive
                if (alliance == ALLIANCE_COLOR.RED) {
                    driveStatus = navxDrive(CargoBotConstants.DRIVING_OFF_PLATFORM_SPEED,
                            -CargoBotConstants.DRIVE_OFF_PLATFORM_DISTANCE_WITHOUT_OFFSET - drivingOffPlatformOffset,
                            calculateTimeout(CargoBotConstants.DRIVE_OFF_PLATFORM_DISTANCE_WITHOUT_OFFSET +
                                            drivingOffPlatformOffset,
                                    CargoBotConstants.DRIVING_OFF_PLATFORM_SPEED), 0);
                } else {
                    driveStatus = navxDrive(CargoBotConstants.DRIVING_OFF_PLATFORM_SPEED,
                            CargoBotConstants.DRIVE_OFF_PLATFORM_DISTANCE_WITHOUT_OFFSET + drivingOffPlatformOffset,
                            calculateTimeout(CargoBotConstants.DRIVE_OFF_PLATFORM_DISTANCE_WITHOUT_OFFSET +
                                            drivingOffPlatformOffset,
                                    CargoBotConstants.DRIVING_OFF_PLATFORM_SPEED), 0);
                }
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP4;
                }
                break;
            case STEP4:
                // Turn towards the VuMark
                if (alliance == ALLIANCE_COLOR.RED) {
                    driveStatus = navxRotateToAngle(CargoBotConstants.RED_TURN_ANGLE_V2, yawKp);
                } else {
                    driveStatus = navxRotateToAngle(CargoBotConstants.BLUE_TURN_ANGLE_V2, yawKp);
                }

                if (driveStatus) {
                    stepFiveStartTime = getRuntime();
                    opmodeState = OPMODE_STEPS.STEP5;
                }
                break;
            case STEP5:
                // wait for VuMark detection, no need to lower the ball arm since it's no longer in the way of the camera
                driveStatus = false;
                if (vuMarkIdentified != VU_MARK_TYPE.UNKNOWN) {
                    driveStatus = true;
                } else {
                    driveStatus = false;
                    // timeout and default to center
                    if ((getRuntime() - stepFiveStartTime) > CargoBotConstants.VU_MARK_DETECTION_TIMEOUT) {
                        vuMarkIdentified = VU_MARK_TYPE.CENTER;
                        driveStatus = true;
                    }
                }

                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP6;
                }
                break;
            case STEP6:
                // Turn away from VuMark
                driveStatus = navxRotateToAngle(0, yawKp);
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP7;
                }
                break;
            case STEP7:
                // move to the column specified by vuMark
                if (startingPosition == STARTING_POSITION.RELIC) {
                    switch (vuMarkIdentified) {
                        case RIGHT:
                            driveStatus = true;
                            break;
                        case CENTER:
                            driveStatus = navxDrive(CargoBotConstants.APPROACH_SPEED,
                                    -CargoBotConstants.MOVE_TO_CENTER_DISTANCE_RELIC,
                                    calculateTimeout(CargoBotConstants.MOVE_TO_CENTER_DISTANCE_RELIC,
                                            CargoBotConstants.APPROACH_SPEED), 0);
                            break;
                        case LEFT:
                            driveStatus = navxDrive(CargoBotConstants.APPROACH_SPEED,
                                    -CargoBotConstants.MOVE_TO_LEFT_DISTANCE_RELIC,
                                    calculateTimeout(CargoBotConstants.MOVE_TO_LEFT_DISTANCE_RELIC,
                                            CargoBotConstants.APPROACH_SPEED), 0);
                            break;
                    }
                }
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP8;
                }
                break;
            case STEP8:
                // turn to face column
                liftPosition = LiftPosition.MOVE;
                liftStatus = blockLiftController();
                driveStatus = navxRotateToAngle(CargoBotConstants.ANGLE_TO_FACE_BOX_RED_RELIC, yawKp);
                if (driveStatus && liftStatus) {
                    opmodeState = OPMODE_STEPS.STEP9;
                }
                break;
            case STEP9:
                // move into a column
                driveStatus = navxDrive(CargoBotConstants.APPROACH_SPEED,
                        CargoBotConstants.CRYPTO_BOX_DISTANCE_RED_RELIC,
                        calculateTimeout(CargoBotConstants.CRYPTO_BOX_DISTANCE_RED_RELIC, CargoBotConstants.APPROACH_SPEED),
                        CargoBotConstants.ANGLE_TO_FACE_BOX_RED_RELIC);
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP10;
                }
                break;
            case STEP10:
                // release block
                driveStatus = intakeController(CargoBotConstants.INTAKE_MOTOR_ACTIVATION_TIME, INTAKE_DIR.OUT);
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP11;
                }
                break;
            case STEP11:
                // drive backwards so robot is not in contact with the block
                driveStatus = navxDrive(CargoBotConstants.APPROACH_SPEED,
                        CargoBotConstants.BACKUP_DISTANCE,
                        calculateTimeout(CargoBotConstants.BACKUP_DISTANCE, CargoBotConstants.APPROACH_SPEED),
                        CargoBotConstants.ANGLE_TO_FACE_BOX_RED_RELIC);
                if (driveStatus) {
                    opmodeState = OPMODE_STEPS.STEP12;
                }
                break;
            case STEP12:
                // TODO: ensure when the last step is complete, call onRobotStopOrInterrupt() to terminate properly. For now, step 12 is the conclusion of autonomous mode.
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

                            robot.frontLeftDrive.setPower(speed + output);
                            robot.frontRightDrive.setPower(speed - output);
                            robot.rearLeftDrive.setPower(speed + output);
                            robot.rearRightDrive.setPower(speed - output);
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
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double calculateTimeout(double distance, double speed) {
        double timeoutS = 0.0;
        timeoutS = distance/speed * MMShooterBotConstants.TIMEOUT_SCALING_FACTOR;

        // make sure we return a positive value
        return Math.abs(timeoutS);
    }

    private boolean blockLiftController() {

        boolean isDone = false;

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

        robot.blockLift.setTargetPosition(target);
        robot.blockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.blockLift.setPower(CargoBotConstants.LIFT_SPEED);
        telemetry.addData("pos", robot.blockLift.getCurrentPosition());
        telemetry.update();
        if (!robot.blockLift.isBusy()) {
            robot.blockLift.setPower(0.0);
            robot.blockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isDone = true;
        }
        return isDone;
    }


    private boolean intakeController(double activationTime, INTAKE_DIR dir) {
        boolean isDone = false;

        switch (intakeControllerState){
            case START:
                // reset timer
                intakeMotorRunTime = getRuntime();
                setIntakeMotorDir(dir);
                intakeControllerState = ENUM_INTAKE_CONTROLLER_STATE.RUN;
                break;

            case RUN:
                // run the motors for the length of activation time
                if ((getRuntime() - intakeMotorRunTime) < activationTime){
                    activateIntakeMotors();
                } else {
                    intakeControllerState = ENUM_INTAKE_CONTROLLER_STATE.RUN;
                }
                break;

            case END:
                intakeControllerState = ENUM_INTAKE_CONTROLLER_STATE.START;
                isDone = true;
                break;
        }

        // TODO: create motor stall detection for each of the 3 intake motors, during RUN state
        return isDone;
    }

    private void activateIntakeMotors(){
        // activate primary intake motor
        robot.frontIntakeMotor.setPower(CargoBotConstants.FRONT_INTAKE_POWER);

        // activate equal power on both left and right intake motors
        robot.leftIntakeMotor.setPower(CargoBotConstants.LEFT_INTAKE_POWER);
        robot.rightIntakeMotor.setPower(CargoBotConstants.RIGHT_INTAKE_POWER);

    }

    private void turnOffIntakeMotors(){
        robot.frontIntakeMotor.setPower(0);
        robot.leftIntakeMotor.setPower(0);
        robot.rightIntakeMotor.setPower(0);

        // make sure intake motors maintain constant speed
        robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setIntakeMotorDir(INTAKE_DIR dir){
        if (dir == INTAKE_DIR.IN){
            robot.frontIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            robot.frontIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.rightIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

    }

    private void onRobotStopOrInterrupt(){
        isDoneRunningAuto = true;
        if (navxDevice != null) {
            navxDevice.close();
        }
        // save the block lift position for next operation
        robot.fileHandler.writeToFile("offset.txt", CargoBotConstants.pathToLiftMotorOffset, Integer.toString(offset + robot.blockLift.getCurrentPosition()), robot.context);
    }

    private void localizeRobot(double robotHeading){
        float tZCorrected = (float) (tZ * (1 + CargoBotConstants.TZ_CORRECTION_FACTOR));
        double rXRadians = (rX * Math.PI / 180);
        float projectedtZCorrected = tZCorrected * (float) cos(rXRadians);
        double robotHeadingRadians = (abs(robotHeading) * Math.PI / 180);
        float lateralDistFromCenterOfPic = projectedtZCorrected * (float) sin(robotHeadingRadians);
        float longitudinalDisFromCenterOfPic = projectedtZCorrected * (float) cos(robotHeadingRadians);
        float[] distanceArray = {lateralDistFromCenterOfPic, longitudinalDisFromCenterOfPic};
        telemetry.addData("distance array", Arrays.toString(distanceArray));
    }

    private double getMedianValue(double yaw){
        double median = 0;
        yaws.add(yaw);

        if (yaws.size() == 20){
            double[] yawArray = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0};
            for (int i = 0; i < 20; i++){
                yawArray[i] = yaws.get(i);
            }
            Arrays.sort(yawArray);

            if (yawArray.length % 2 == 0){
                median = (double)yawArray[yawArray.length/2] + (double)yawArray[yawArray.length/2 - 1]/2;

            } else {
                median =(double)yawArray[yawArray.length/2];
            }
        }

        return median;
    }
}
