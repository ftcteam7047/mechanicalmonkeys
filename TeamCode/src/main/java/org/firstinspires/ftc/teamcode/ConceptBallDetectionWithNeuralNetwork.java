/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.content.ContextWrapper;
import android.graphics.Bitmap;
import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.DetectionAsyncTask.AsyncResponse;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Concept: Ball Detection w/ Neural Network", group ="Concept")
//@Disabled
public class ConceptBallDetectionWithNeuralNetwork extends LinearOpMode implements AsyncResponse {

    public static final String TAG = "Vuforia Sample";

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
    private static final int MINIMUM_NUM_LOOPS = 16142; // approximately 2 frame/sec
    int cropSize;
    float minimumConfidence;
    BALL_PREDICTION_RESULT prediction = BALL_PREDICTION_RESULT.UNKNOWN;
    String predictionString = "unknown";
    boolean isProcessingFrame = false;
    Context appContext = null;
    boolean runAsyncTask = false;

    @Override public void runOpMode() throws InterruptedException {
        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AaukgGT/////AAAAGWkWAT0TG0LEo/kIYGHRbONkJEGTvNn6Q1Urj56aIj3ewuw2apA6gNe8KvZxdfL42j2CQo+4CAp0xeKL1zXwrc7wJjNeyavmXD0dUI32L8ssp5QiCAtPXF7uwqLm+Q/1vWD5w5NpD+2l1yCpsZgwhF0IKBlgJmwTZpouOzjSMcGFz5H3ERq/n8b405blFDI4unS6C1BMUIQVhQgFZe2PUrslIMeopgc3MyHIzW5nNhwX161k99VxXCdMPp+qKwP0N4Wj+aVwdGSXzwnX4pwzLdDTh4IsQNHlNaWzT/mv8hIQ703Cxf/L5DiI8KBnLjG9jOtlPXxu9NULXMFZf9qBB8tDnqY5EvkF+gt8n0BHUmNq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */
        VuforiaTrackables visionTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17"); // was StonesAndChips

        // from the device database, unit is meter
        //<ImageTarget name="gears" size="0.280000 0.214343" />
        //<ImageTarget name="wheels" size="0.280000 0.214343" />
        //<ImageTarget name="tools" size="0.280000 0.214343" />
        //<ImageTarget name="legos" size="0.280000 0.214343" />

        VuforiaTrackable gearsTarget = visionTargets.get(3);
        gearsTarget.setName("gearsTarget");  // gears

        VuforiaTrackable toolsTarget = visionTargets.get(2);
        toolsTarget.setName("toolsTarget");  // tools

        VuforiaTrackable wheelsTarget  = visionTargets.get(1);
        wheelsTarget.setName("wheelsTarget");  // wheels

        VuforiaTrackable legosTarget  = visionTargets.get(0);
        legosTarget.setName("legosTarget");  // legos

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(visionTargets);

        VuforiaTrackableDefaultListener wheels = null;


        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float meterPerInch     = 0.0254f;
        float meterBotWidth       = 18 * meterPerInch;            // ... or whatever is right for your robot
        float meterFTCFieldWidth  = (12*12 - 2) * meterPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float meterTileWidth = 24 * meterPerInch;

        Context context;
        context = hardwareMap.appContext;
        appContext = context;

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */
        // Velocity Vortex set up:
        // gears target on the red wall: middle of 4th tile from the audience corner, each tile is 24 inch long
        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X, negative translation in Y.*/
                .translation(-meterFTCFieldWidth/2, -meterTileWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocationOnField);
        RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        // Velocity Vortex set up:
        // tools target on the red wall: middle of 2nd tile from the audience corner, each tile is 24 inch long
        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X, positive translation in Y. */
                .translation(-meterFTCFieldWidth/2, 1.5f * meterTileWidth, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));



       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        // Velocity Vortex set up:
        // wheels target on the blue wall: middle of 4th tile from the audience corner, each tile is 24 inch long
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in X, positive translation in Y.*/
                .translation(meterTileWidth/2, meterFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheelsTarget.setLocation(wheelsTargetLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

        // Velocity Vortex set up:
        // wheels target on the blue wall: middle of 4th tile from the audience corner, each tile is 24 inch long
        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a negative translation in X, positive translation in Y.*/
                .translation(-1.5f * meterTileWidth, meterFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legosTarget.setLocation(legosTargetLocationOnField);
        RobotLog.ii(TAG, "Legos Target=%s", format(legosTargetLocationOnField));
        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        //TODO: 9/20/16 change phoneLocationOnRobot when the exact location is available
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(meterBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)gearsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)wheelsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)toolsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legosTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        // initialize tensor flow object detection
        initDetection(context);
        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = gearsTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        visionTargets.activate();
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        int loopCounter = 0;
        int index = 0;
        int frameCounter = 0;
        ArrayList<DetectionAsyncTask> detectionAsyncTaskList = new ArrayList<DetectionAsyncTask>();
        int numOfAsyncTasks = 0;
        ExecutorService executorService= Executors.newFixedThreadPool(1);

        while (opModeIsActive()) {
            VuforiaLocalizer.CloseableFrame frame = null;
            if (loopCounter % MINIMUM_NUM_LOOPS == 0) {
                // only take a new frame every MINIMUM_NUM_LOOPS loops
                loopCounter = 0;

                frame = vuforia.getFrameQueue().take();
                if (frame != null) {
                    Image img = getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
                    if (img != null){
                        frameCounter++;
                        final Bitmap bm = getBitmapFromImage(img);

                        // beta
                        if(!isProcessingFrame) {
                            executorService.execute(new Runnable() {
                                @Override
                                public void run() {

                                    isProcessingFrame = true;
                                    //timer.tic();
                                    //timer.toc("Object Detection");
                                    processBitmap(bm);
                                    isProcessingFrame = false;
                                }
                            });
                        }

                        if (runAsyncTask){
                            if (numOfAsyncTasks <= 10) {
                                DetectionAsyncTask detectionAsyncTask = new DetectionAsyncTask(context, detector, cropSize, minimumConfidence);
                                //this to set delegate/listener back to this class
                                detectionAsyncTaskList.add(detectionAsyncTask);
                                detectionAsyncTaskList.get(index).delegate = this;
                                detectionAsyncTaskList.get(index).execute(bm);
                                index++;
                            }
                            numOfAsyncTasks = detectionAsyncTaskList.size();
                            for (DetectionAsyncTask asyncTask : detectionAsyncTaskList) {

                                if ((asyncTask.getStatus() == AsyncTask.Status.FINISHED) ||
                                        (asyncTask.isCancelled())) {
                                    numOfAsyncTasks--;
                                }
                            }
                            // if a new detection is finished, while the old one is still running, just cancel it to avoid stale result
                            for (int i = 0; i < detectionAsyncTaskList.size(); i++){
                                if (i > 0){
                                    if (detectionAsyncTaskList.get(i).getStatus() == AsyncTask.Status.FINISHED){
                                        if (detectionAsyncTaskList.get(i-1).getStatus() == AsyncTask.Status.RUNNING){
                                            detectionAsyncTaskList.get(i-1).cancel(true);
                                        }
                                    }
                                }
                            }
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
            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                //telemetry.addData("Pos", format(lastLocation));
            } else {
                //telemetry.addData("Pos", "Unknown");
            }
            //telemetry.update();
            idle();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
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
        // TODO: TRY Bitmap.Config.ARGB_8888
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

        // TODO: verify that this act as a callback
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

    @Override
    public void processFinish(ArrayList<String> output) {
        detectionResult = searchDetectionResult(output);
        prediction = decodeDetectionResult(detectionResult);
        switch (prediction){
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

    private enum BALL_PREDICTION_RESULT {
        UNKNOWN,
        RED_ON_LEFT,
        BLUE_ON_LEFT
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
}
