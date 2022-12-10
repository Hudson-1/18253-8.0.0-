package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.InterpLUT;

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
public class VisionPole implements Subsystem {

    // Configurable Vision Variables
    public static int webcamHeight = 240;
    public static int webcamWidth = 320;
    public static double FOV = 55.0;            // FOV of the webcam
    public static double poleDiameter = 1.05;   // Actual size of the Pole's diameter in inches
    public static double offset = 0.1;          // offset from pole surface to robot

    // Current color it is detecting is yellow.
    public static double hueMin = 0;
    public static double hueMax = 100;
    public static double saturationMin = 110;
    public static double saturationMax = 255;
    public static double valueMin = 140;
    public static double valueMax = 255;
    private double distanceFromPoleCenterToImageCenter;
    private double widthOfTheClosestPole;

    OpenCvCamera webcam;
    private VisionPipeline visionPipeline;

    InterpLUT DISTANCE_FROM_CENTER;
    InterpLUT DISTANCE_FROM_POLE;

    @Override
    public void init(HardwareMap map) {
        // Set the Look Up Tables, convert pixels to inches
        DISTANCE_FROM_CENTER = new InterpLUT();
        DISTANCE_FROM_POLE = new InterpLUT();

        // Create a Look Up Table that converts distance to the center from pixels (image) to inches (real world)
        DISTANCE_FROM_CENTER.add(-webcamWidth / 2.0, 45);   // if the pixel is at the left edge of the image, its 'real life' distance to the center is 45 inches?
        DISTANCE_FROM_CENTER.add(0, 0);                     // if the pixel is at the center of the image, its 'real life' distance to the center is 0
        DISTANCE_FROM_CENTER.add(webcamWidth / 2.0, -45);   // if the pixel is at the right edge of the image, its 'real life' distance to the center is -45 inches.
        DISTANCE_FROM_CENTER.createLUT();

        // Create a Look Up Table that converts width of the pole (pixels) to the distance between the pole and camera (inches)
        DISTANCE_FROM_POLE.add(5, 2);   // if the width of the pole is 5 pixels, its actual distance to the camera is 2 inches?
        DISTANCE_FROM_POLE.add(50, 0);  // if the width of the pole is 50 pixels, its actual distance to the camera in real life is 0
        DISTANCE_FROM_POLE.add(100, -2);// if the width of the pole is 100 pixels, its actual distance to the camera in real life is -2 inches, meaning too close to be possible
        DISTANCE_FROM_POLE.createLUT();

        // Initialize new VisionPipeline instance
        visionPipeline = new VisionPipeline();

        // Create a new Webcam instance and get the visionPipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam2"));
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

        private Mat workingMatrix = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) { // If the frame is empty, just return
                return input;
            }

            // Here we set the filters and manipulate the image:

            // These filter out everything but yellow, and turn it into black and white
            workingMatrix = input.submat(100, 240, 60, 320); // added this if we want to crop the image
            Imgproc.GaussianBlur(workingMatrix, workingMatrix, new Size(5.0, 15.0), 0.00);
            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_BGR2HSV);
            Core.inRange(workingMatrix, new Scalar(hueMin, saturationMin, valueMin),
                    new Scalar(hueMax, saturationMax, valueMax), workingMatrix);

            // This finds the edges
            Imgproc.Canny(workingMatrix, workingMatrix, 100, 300);

            // This finds the contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(workingMatrix, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            int maxWidth = 0;
            Rect maxRect = new Rect();

            // Enumerate each contour and find the one that's closest to the camera, assuming that is the closet pole
            for (MatOfPoint c : contours) {
                MatOfPoint2f contour = new MatOfPoint2f(c.toArray());
                Rect boundingRect = Imgproc.boundingRect(contour);

                // This finds the width of the contour
                if (boundingRect.width > maxWidth) {
                    maxWidth = boundingRect.width;
                    maxRect = boundingRect;
                }

                // Release the temp objects
                hierarchy.release();
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                contour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            // Save the width of the closest pole to widthOfTheClosestPole in pixels
            widthOfTheClosestPole = maxWidth;

            // Calculate the distance from the center of the pole to the center of the image
            // If the value is negative, it is at the left side of the center;
            // otherwise, it is at the right side of the image center
            distanceFromPoleCenterToImageCenter = maxRect.x + (maxRect.width / 2.0) - (webcamWidth / 2.0);

            return workingMatrix;
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

    public double getDistance() {
        // This is supposed to be the distance between camera and pole *if and only if* when pole is at the center of the image
        double angle = widthOfTheClosestPole * 0.5 * FOV / webcamWidth; // get the angle based on the width of the closest pole
        return ((poleDiameter * 0.5 / Math.tan(angle)) - getOffset());                    // get the actual distance between pole and the camera in inches
    }

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }


}


