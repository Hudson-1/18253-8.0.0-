package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class VisionPoleRevised implements Subsystem {

    // Configurable Vision Variables
    public static int webcamHeight = 240;
    public static int webcamWidth = 320;
    public static double FOV = 55.0;            // FOV of the webcam
    public static double poleDiameter = 1.05;   // Actual size of the Pole's diameter in inches
    public static double offset = 0.1;          // offset from pole surface to robot
    public static double knownDistance = 12.0;  // Known distance to estimate the focal length
    public static double knownImageWidth = 26.0; // pole width in the image
    private double distanceFromPoleCenterToImageCenter;
    private double widthOfTheClosestPole;
    private int numberOfContours;

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 130;
    public static double strictHighS = 200;
    public static double hueMin = 16;  // was 20
    public static double hueMax = 30; // was 32
    public static double saturationMin = 70;  // was 70
    public static double saturationMax = 255;
    public static double valueMin = 80; // was 80
    public static double valueMax = 255;

    public VisionPoleRevised() {
        frameList = new ArrayList<>();
    }


    OpenCvCamera webcam;
    private VisionPipeline visionPipeline;

    @Override
    public void init(HardwareMap map) {

        // Initialize new VisionPipeline instance
        visionPipeline = new VisionPipeline();

        // Create a new Webcam instance and get the visionPipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam1"));
        webcam.setPipeline(visionPipeline);

        // Open the camera and start processing the frames from the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(webcamWidth, webcamHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    @Override
    public void update() {

    }

    @Override
    public Telemetry telemetry() {
        return null;
    }

    //
    // VISION PIPELINE
    //
    class VisionPipeline extends OpenCvPipeline {

        private final Mat workingMatrix = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) { // If the frame is empty, just return
                return input;
            }
            Mat mat = new Mat();

            //mat turns into HSV value
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return input;
            }

            // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
            Scalar lowHSV = new Scalar(hueMin, saturationMin, valueMin); // lenient lower bound HSV for yellow
            Scalar highHSV = new Scalar(hueMax, saturationMax, valueMax); // lenient higher bound HSV for yellow

            Mat thresh = new Mat();

            // Get a black and white image of yellow objects
            Core.inRange(mat, lowHSV, highHSV, thresh);

            Mat masked = new Mat();
            //color the white portion of thresh in with HSV from mat
            //output into masked
            Core.bitwise_and(mat, mat, masked, thresh);
            //calculate average HSV values of the white thresh values
            Scalar average = Core.mean(masked, thresh);

            Mat scaledMask = new Mat();
            //scale the average saturation to 150
            masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

            Mat scaledThresh = new Mat();
            //you probably want to tune this
            Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
            Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
            //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
            Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

            Mat finalMask = new Mat();
            //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
            Core.bitwise_and(mat, mat, finalMask, scaledThresh);

            Mat edges = new Mat();
            //detect edges(only useful for showing result)(you can delete)
            Imgproc.Canny(scaledThresh, edges, 100, 200);

            //contours, apply post processing to information
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            //find contours, input scaledThresh because it has hard edges
            Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            int maxWidth = 0;
            Rect maxRect = new Rect();

            //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
            if (frameList.size() > 5) {
                frameList.remove(0);
            }

            numberOfContours = contours.size();

            // Enumerate each contour and find the one that's closest to the camera, assuming that is the closet pole
            for (MatOfPoint c : contours) {
                MatOfPoint2f contour = new MatOfPoint2f(c.toArray());
                Rect boundingRect = Imgproc.boundingRect(contour);

                // This finds the width of the contour
                if (boundingRect.width > maxWidth) {
                    maxWidth = boundingRect.width;
                    maxRect = boundingRect;
                }

                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                contour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            //release all the data
            input.release();
            scaledThresh.copyTo(input);
            scaledThresh.release();
            scaledMask.release();
            mat.release();
            masked.release();
            edges.release();
            thresh.release();
            finalMask.release();

            // change the return to whatever mat you want
            // for example, if I want to look at the lenient thresh:
            // return thresh;
            // note that you must not do thresh.release() if you want to return thresh
            // you also need to release the input if you return thresh(release as much as possible)
            //return input;

            // Save the width of the closest pole to widthOfTheClosestPole in pixels
            widthOfTheClosestPole = maxWidth;

            // Calculate the distance from the center of the pole to the center of the image
            // If the value is negative, it is at the left side of the center;
            // otherwise, it is at the right side of the image center
            distanceFromPoleCenterToImageCenter = maxRect.x + (maxRect.width / 2.0) - (webcamWidth / 2.0);

            return input;
        }
    }

    private double getOffset() {
        return (poleDiameter * 0.5 + offset);
    }


    public double getDistanceFromPoleCenterToImageCenter() {
        return distanceFromPoleCenterToImageCenter;
    }

    public double getWidthOfTheClosestPole() {
        return widthOfTheClosestPole;
    }


    public double getAngle() {
        // This is supposed to be the angle between the line from camera to pole, and the line from camera to center of the image
        // If this value is negative, it means the robot needs to turn left, if the value is positive, it means the robot needs to turn right
        return distanceFromPoleCenterToImageCenter * FOV / webcamWidth;
    }

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }

    private double getFocalLength(double measuredDistance, double realWidth, double imageWidth) {
        return imageWidth * measuredDistance / realWidth;
    }

    public double getDistanceFromFocalLength() {
        return (poleDiameter * getFocalLength(knownDistance, poleDiameter, knownImageWidth)) / widthOfTheClosestPole;
    }

}

