package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//     implementation 'org.openftc:easyopencv:1.5.1'
// Vision

@Config
public class VisionPole implements Subsystem {

    // Configurable Vision Variables
    public static int webcamHeight = 240;
    public static int webcamWidth = 320;

    // Current color it is detecting is yellow.
    public static double hueMin = 0;
    public static double hueMax = 100;
    public static double saturationMin = 110;
    public static double saturationMax = 255;
    public static double valueMin = 100;
    public static double valueMax = 255;
    public static double cannyThreshold1 = 100;
    public static double cannyThreshold2 = 300;


    private enum VisionType {
        BGR2HSVcolor
    }

    OpenCvCamera webcam;
    private VisionPipeline visionPipeline;
    private VisionType visionType;

    @Override
    public void init(HardwareMap map) {
        // Set the vision filter type:
        visionType = VisionType.BGR2HSVcolor;
        visionPipeline = new VisionPipeline();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam2"));
        webcam.setPipeline(visionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(webcamWidth, webcamHeight, OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) { }
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


        private double matTotal = 0;
        private Mat workingMatrix = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if(workingMatrix.empty()) {
                return input;
            }

            // Here you set the filters and manipulate the image:
            switch (visionType) {

                case BGR2HSVcolor:
                    workingMatrix = input.submat(100, 240, 60, 320); // added this to attempt to tune to middle
                    Imgproc.GaussianBlur(workingMatrix, workingMatrix, new Size(5.0, 15.0), 0.00);
                    Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_BGR2HSV);
                    Core.inRange(workingMatrix, new Scalar(hueMin, saturationMin, valueMin),
                            new Scalar(hueMax, saturationMax, valueMax), workingMatrix);
                // SOMEHOW? Use Canny Edge Detection to find edges -- you might have to tune the thresholds for hysteresis
                    Mat edges = new Mat();
                    Imgproc.Canny(workingMatrix, edges, cannyThreshold1, cannyThreshold2);
                // CODE HERE NEEDS TO IDENTIFY THE EDGES AND:
                    // 1. CALCULATE THE DISTANCE BETWEEN THEM (THIS TELLS US HOW FAR AWAY WE ARE)
                    // 2. CALCULATE THE CENTER POINT (THIS TELLS US HOW OFF OUR ANGLE IS)
                    // OUR GOAL WILL BE TO TURN SO THE CENTER OF THE POLE MATCHES THE CENTER OF THE CAMERA SCREEN
                    // AND TO MOVE SO THAT THE WIDTH OF THE POLE MATCHES A PREDETERMINED NUMBER
                    break;

            }
            return workingMatrix;
        }
    }


    // THE CODE BELOW IS FROM DETECTING GREEN - IT WILL BE REPLACED

    // THIS IS WHERE ALL THE MATH/LOGIC FOR AUTO AIM CAN GO

    // INSTEAD OF RETURNING DETECTION STATES, IT WILL RETURN X, Y, AND ANGLE


    public double getColorNum() {
        return (getVisionPipeline().matTotal)/1000000;
    }

    public Detection_States returnVisionState() {

        if (getColorNum() > .3) {
            return Detection_States.THREE;
        } else if (getColorNum() < 0.01) {
            return Detection_States.ONE;
        } else {
            return Detection_States.TWO;
        }
    }

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }

    public enum Detection_States {
        ONE,
        TWO,
        THREE
    }

}





