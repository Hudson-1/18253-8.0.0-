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
    private double midLine;
    private double width;


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


        private double matTotal = 0;
        private Mat workingMatrix = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) {
                return input;
            }

            switch (visionType) {

                // Here we set the filters and manipulate the image:

                case BGR2HSVcolor:
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
                    Imgproc.findContours(workingMatrix, contours, workingMatrix, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                    int maxWidth = 0;
                    Rect maxRect = new Rect();
                    for (MatOfPoint c : contours) {
                        MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                        Rect rect = Imgproc.boundingRect(copy);

                    // This finds the width of the contour
                        int w = rect.width;
                        if (w > maxWidth) {
                            maxWidth = w;
                            maxRect = rect;
                        }

                    // the width of the rect is going to be stored in maxWidth:
                        int maxWidthX = maxRect.width;
                        width = maxWidthX;

                    // the center line of the rect is going to be stored in midLine:
                        double midLineX = (maxRect.width / 2.0) - (webcamWidth / 2.0);
                        midLine = midLineX;

                        c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                        copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
                    }

                    // Now we know how wide the rectangle is, which tells us how much we need to change our distance
                    // And we know the center line, which tells us how much we need to change our angle
                    // We might want to look at this, which helps use these variables to calculate a spline:
                    // https://tools.timodenk.com/cubic-spline-interpolation

                    // We need to create a helper class that will do all the math
                    // And for starters, we might just want it to output telemetry so we can test it
                    // Like, at width 10px we need to move 10 inches
                    // At 100px we need to move 2 inches
                    // At 200px we need to move 0.5 inches
                    // Then we can create a lookup table
                    // https://docs.ftclib.org/ftclib/features/util#interplut-interpolated-look-up-table
                    //
                    // The helper class will then send the three variables X, Y, Angle to PoleAimTele

                    break;

            }
            return workingMatrix;
        }
    }

    public double getMid() {
        return midLine;
    }

    public double getWidth() {
        return width;
    }

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }

}



