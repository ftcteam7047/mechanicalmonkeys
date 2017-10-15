package org.firstinspires.ftc.teamcode;

/**
 * Created by chrischen on 10/19/16.
 */

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class MMFileHandler extends OpMode {
    /* Constructor */
    public MMFileHandler(){

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    //    public void writeToFile(String filename, String data, Context context) {
//        try {
//            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(filename, Context.MODE_PRIVATE));
//            outputStreamWriter.write(data);
//            outputStreamWriter.close();
//        } catch (IOException e) {
//            Log.e("Exception", "File write failed: " + e.toString());
//        }
//    }


//    public String readFromFile(String filename, Context context) {
//
//        String ret = "";
//
//        try {
//            InputStream inputStream = context.openFileInput(filename);
//
//            if (inputStream != null) {
//                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
//                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
//                String receiveString = "";
//                StringBuilder stringBuilder = new StringBuilder();
//
//                while ((receiveString = bufferedReader.readLine()) != null) {
//                    stringBuilder.append(receiveString);
//                }
//
//                inputStream.close();
//                ret = stringBuilder.toString();
//            }
//        } catch (FileNotFoundException e) {
//            Log.e("login activity", "File not found: " + e.toString());
//        } catch (IOException e) {
//            Log.e("login activity", "Can not read file: " + e.toString());
//        }
//
//        return ret;
//    }



    public String readFromFile(String filename, Context context)  {
        // The name of the file to open.
        //String fileName = "temp.txt";
        File path = context.getFilesDir();
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FTC/liftMotorOffset");
        dir.mkdirs();
        File file = null;

        if (isExternalStorageReadable()) {
            file = new File(dir, filename);  // external storage on micro sd card
        } else {
            file = new File(path, filename); // app's local storage
        }

        // This will reference one line at a time
        String line = null;
        String returnString = "error";

        try {
            // FileReader reads text files in the default encoding.
            FileReader fileReader =
                    new FileReader(file);

            // Always wrap FileReader in BufferedReader.
            BufferedReader bufferedReader =
                    new BufferedReader(fileReader);

            while((line = bufferedReader.readLine()) != null) {
                System.out.println(line);
                telemetry.addData("Reading Status", line);
                returnString = line;
            }

            // Always close files.
            bufferedReader.close();
        }
        catch(FileNotFoundException ex) {
            System.out.println(
                    "Unable to open file '" +
                            filename + "'");
            telemetry.addData("File Status", "Unable to open file '" + filename + "'");
        }
        catch(IOException ex) {
            System.out.println(
                    "Error reading file '"
                            + filename + "'");
            telemetry.addData("File Status", "Error reading file '" + filename + "'");
            // Or we could just do this:
            // ex.printStackTrace();
        }

        return returnString;
    }

    public void writeToFile(String filename, String data, Context context) {
        // The name of the file to open.
        //String fileName = "temp.txt";
        File path = context.getFilesDir();
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FTC/liftMotorOffset");
        dir.mkdirs();
        File file = null;

        if (isExternalStorageWritable()) {
            file = new File(dir, filename);  // external storage on micro sd card
        } else {
            file = new File(path, filename); // app's local storage
        }

//        File path = context.getFilesDir();
//        File file = new File(path, filename);

        try {
            // Assume default encoding.
            FileWriter fileWriter =
                    new FileWriter(file);

            // Always wrap FileWriter in BufferedWriter.
            BufferedWriter bufferedWriter =
                    new BufferedWriter(fileWriter);

            // Note that write() does not automatically
            // append a newline character.
            bufferedWriter.write(data);
//          bufferedWriter.write(" here is some text.");
//          bufferedWriter.newLine();
//          bufferedWriter.write("We are writing");
//          bufferedWriter.write(" the text to the file.");

            // Always close files.
            bufferedWriter.close();
        }
        catch(IOException ex) {
            System.out.println(
                    "Error writing to file '"
                            + filename + "'");
            telemetry.addData("File Status", "Error writing to file '" + filename + "'");
            // Or we could just do this:
            // ex.printStackTrace();
        }
    }

    public int stringToInt(String string) {
        int myInt = 0;

        try {
            myInt = Integer.parseInt(string);
        } catch(NumberFormatException nfe) {
            System.out.println("Could not parse " + nfe);
        }
        return myInt;
    }

    /* Checks if external storage is available for read and write */
    private boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
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
}