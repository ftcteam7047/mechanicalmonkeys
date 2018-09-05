package org.firstinspires.ftc.teamcode;


import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.os.AsyncTask;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Created by chrischen on 10/1/17.
 */

public class DetectionAsyncTask extends AsyncTask<Bitmap, Void, ArrayList<String>> {
    Context appContext;
    Paint rectPaint, textPaint;
    Classifier detector;
    int imageInputSize;
    float minimumConfidence;
    // internal time tracking
    private double startTime;

    public interface AsyncResponse {
        void processFinish(ArrayList<String> output);
    }
    public AsyncResponse delegate = null;

    // the constructor
    public DetectionAsyncTask(Context context, Classifier detector, int imageInputSize, float minimumConfidence) {

        this.appContext = context;
        this.detector = detector;
        this.imageInputSize = imageInputSize;
        this.minimumConfidence = minimumConfidence;
    }


    @Override
    protected void onPreExecute (){
        rectPaint = new Paint();
        startTime = getRuntime();
    }

    @Override
    protected ArrayList<String> doInBackground(Bitmap... params) {
        Bitmap drawBitmap,b,b2;
        Canvas canvas;

        ArrayList<String> result = new ArrayList<String>();
        //Log.d(TAG, "MyAsyncTask@doInBackground from another thread");
        b = BitmapUtils.resizeBitmap(params[0], imageInputSize);
        b2 = BitmapUtils.rotateBitmap(b);
        float scalar = 1.0f;
        if (detector!= null) {
            List<Classifier.Recognition> r = detector.recognizeImage(b2);
            //recognize = timer.toc("Recognize Image");
            //timer.tic();
//            drawBitmap = b2.copy(Bitmap.Config.ARGB_8888, true);
//            canvas = new Canvas(drawBitmap);
            for (Classifier.Recognition recog : r) {
                Log.d("Recognition", recog.toString());
                if (!recog.getTitle().equals("???")){
                    if (recog.getConfidence() >= minimumConfidence) {
                        float leftEdge = recog.getLocation().left * scalar;
                        result.add(recog.getTitle() + ": " + recog.getConfidence().toString() + "leftEdge" + Float.toString(leftEdge));
                    }
                }
//            telemetry.addData("detection", recogString);
//            telemetry.update();
//                float scalar = 1.0f;
//                canvas.drawRect(recog.getLocation().left * scalar, recog.getLocation().top * scalar,
//                        recog.getLocation().right * scalar, recog.getLocation().bottom * scalar, rectPaint);
//                String conf = Float.toString(recog.getConfidence());
//                canvas.drawText(recog.getTitle(), recog.getLocation().left * scalar, recog.getLocation().top * scalar, textPaint);
//                canvas.drawText(conf.substring(2, 4), recog.getLocation().left * scalar, (recog.getLocation().top + 25) * scalar, textPaint);
            }
        }

        return result;

    }
    @Override
    protected void onPostExecute(ArrayList<String> result) {
        super.onPostExecute(result);
        double elapsedTime = getRuntime() - startTime;
        result.add(Double.toString(elapsedTime));
        delegate.processFinish(result);
    }

    @Override
    protected void onCancelled(ArrayList<String> result) {
    }

    private double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }
}
