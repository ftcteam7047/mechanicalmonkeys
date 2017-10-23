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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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

@Autonomous(name="Autonomous Test", group ="Concept")
//@Disabled
public class MMAutonomousTest extends LinearOpMode {

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
    HardwareMecanumTestBot mecanumBot = new HardwareMecanumTestBot();

    @Override public void runOpMode() throws InterruptedException {

        mecanumBot.init(hardwareMap);

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        double tStart = getRuntime();
        relicTrackables.activate();

        // set up frame format to be used for object detection
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        // set up counters to be use for object detection
        int loopCounter = 0;
        int frameCounter = 0;
        ExecutorService executorService= Executors.newFixedThreadPool(1);

        while (opModeIsActive()) {
            VuforiaLocalizer.CloseableFrame frame = null;
            tick = getRuntimeInTicks(tStart, PERIOD_PER_TICK);
            if (tick > lastTick) {
                // update for comparison in the next loop
                lastTick = tick;

                frame = vuforia.getFrameQueue().take();
                if (frame != null) {
                    Image img = getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
                    if (img != null){
                        frameCounter++;
                        final Bitmap bm = getBitmapFromImage(img);

                        if (!isProcessingFrame) {
                            executorService.execute(new Runnable() {
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
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();

            // run loops every 10 ms
            sleep(10);
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
}
