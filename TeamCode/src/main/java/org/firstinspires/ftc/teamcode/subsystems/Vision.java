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
public class Vision implements Subsystem {

    // Configurable Vision Variables
    public static int webcamHeight = 240;
    public static int webcamWidth = 320;

    // Current color it is detecting is light green.
    public static double hueMin = 50;
    public static double hueMax = 75;
    public static double saturationMin = 60;
    public static double saturationMax = 200;
    public static double valueMin = 90;
    public static double valueMax = 250;


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

        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam1"));
        webcam.setPipeline(visionPipeline);

        // If the camera doesn't start up right away, maybe uncomment this section
        /*
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
            }

            @Override
            public void onError(int errorCode) {
            }
        });

         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(webcamWidth, webcamHeight, OpenCvCameraRotation.SIDEWAYS_LEFT); }
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
                    Imgproc.GaussianBlur(workingMatrix, workingMatrix, new Size(5.0, 15.0), 0.00);
                    Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_BGR2HSV);
                    Core.inRange(workingMatrix, new Scalar(hueMin, saturationMin, valueMin),
                            new Scalar(hueMax, saturationMax, valueMax), workingMatrix);

                    matTotal = Core.sumElems(workingMatrix).val[0];
                break;

            }
            return workingMatrix;
        }
    }

    public double getColorNum() {
        return (getVisionPipeline().matTotal)/1000000;
    }

    public String returnVisionState() {

        if (getColorNum() > .3) {
            return "3";
        } else if (getColorNum() < 0.1) {
            return "1";
        } else {
            return "2";
        }
    }

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }
}





