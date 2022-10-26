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

    //
    public static double hueMinColor1 = 50;
    public static double hueMaxColor1 = 75;
    public static double saturationMinColor1 = 60;
    public static double saturationMaxColor1 = 200;
    public static double valueMinColor1 = 90;
    public static double valueMaxColor1 = 250;

    public static double hueMinColor2 = 50;
    public static double hueMaxColor2 = 75;
    public static double saturationMinColor2 = 60;
    public static double saturationMaxColor2 = 200;
    public static double valueMinColor2 = 90;
    public static double valueMaxColor2 = 250;

    public static double hueMinColor3 = 50;
    public static double hueMaxColor3 = 75;
    public static double saturationMinColor3 = 60;
    public static double saturationMaxColor3 = 200;
    public static double valueMinColor3 = 90;
    public static double valueMaxColor3 = 250;

    //

    private enum VisionType {
        BGR2HSVcolor,
        BGR2HSVcolor3
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

        private double matTotalColor1 = 0;
        private double matTotalColor2 = 0;
        private double matTotalColor3 = 0;

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

                case BGR2HSVcolor3:

                    Mat color1 = new Mat();
                    Mat color2 = new Mat();
                    Mat color3 = new Mat();
                    workingMatrix.copyTo(color1);
                    workingMatrix.copyTo(color2);
                    workingMatrix.copyTo(color3);

                    Imgproc.GaussianBlur(color1, color1, new Size(5.0, 15.0), 0.00);
                    Imgproc.cvtColor(color1, color1, Imgproc.COLOR_BGR2HSV);
                    Core.inRange(color1, new Scalar(hueMinColor1, saturationMinColor1, valueMinColor1),
                            new Scalar(hueMaxColor1, saturationMaxColor1, valueMaxColor1), color1);

                    Imgproc.GaussianBlur(color2, color2, new Size(5.0, 15.0), 0.00);
                    Imgproc.cvtColor(color2, color2, Imgproc.COLOR_BGR2HSV);
                    Core.inRange(color2, new Scalar(hueMinColor2, saturationMinColor2, valueMinColor2),
                            new Scalar(hueMaxColor2, saturationMaxColor2, valueMaxColor2), color2);

                    Imgproc.GaussianBlur(color3, color3, new Size(5.0, 15.0), 0.00);
                    Imgproc.cvtColor(color3, color3, Imgproc.COLOR_BGR2HSV);
                    Core.inRange(color3, new Scalar(hueMinColor3, saturationMinColor3, valueMinColor3),
                            new Scalar(hueMaxColor3, saturationMaxColor3, valueMaxColor3), color3);

                    matTotalColor1 = Core.sumElems(color1).val[0];
                    matTotalColor2 = Core.sumElems(color2).val[0];
                    matTotalColor3 = Core.sumElems(color3).val[0];

                    break;

            }
            return workingMatrix;
        }
    }

    public double getMatTotal() {
        return visionPipeline.matTotal;
    }

    public double getMatTotalColor1() {
        return visionPipeline.matTotalColor1;
    }

    public double getMatTotalColor2() {
        return visionPipeline.matTotalColor2;
    }

    public double getMatTotalColor3() {
        return visionPipeline.matTotalColor3;
    }

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }
}





