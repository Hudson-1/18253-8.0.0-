package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.util.Range;

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

    // Current color it is detecting is yellow.
    public static double hueMin = 0;
    public static double hueMax = 100;
    public static double saturationMin = 110;
    public static double saturationMax = 255;
    public static double valueMin = 140;
    public static double valueMax = 255;
    private double midLine;
    private double width;

    OpenCvCamera webcam;
    private VisionPipeline visionPipeline;

    InterpLUT DISTANCE_FROM_CENTER;
    InterpLUT DISTANCE_FROM_POLE;

    @Override
    public void init(HardwareMap map) {
        // Set the Look Up Tables, convert pixels to inches
        DISTANCE_FROM_CENTER = new InterpLUT();
        DISTANCE_FROM_POLE = new InterpLUT();

        DISTANCE_FROM_CENTER.add(-webcamWidth / 2, 45);
        DISTANCE_FROM_CENTER.add(0, 0);
        DISTANCE_FROM_CENTER.add(webcamWidth / 2, -45);
        DISTANCE_FROM_CENTER.createLUT();

        DISTANCE_FROM_POLE.add(5, 2);
        DISTANCE_FROM_POLE.add(50, 0);
        DISTANCE_FROM_POLE.add(100, -2);
        DISTANCE_FROM_POLE.createLUT();

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

        private Mat workingMatrix = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) {
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
            for (MatOfPoint c : contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                // This finds the width of the contour
                int w = rect.width;
                if (w > maxWidth) {
                    maxWidth = w;
                    maxRect = rect;
                }

                // the width of the rect is going to be stored in width:
                int maxWidthX = maxRect.width;
                width = maxWidthX; // the width of the rectangle of the pole

                // the center line of the rect is going to be stored in midLine:
                double midLineX = maxRect.x + (maxRect.width / 2.0) - (webcamWidth / 2.0);
                midLine = midLineX; // x of the mid of pole

                hierarchy.release();
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            //  Measure alignment (which is the difference between midline and the center of the camera screen)
            //  Measure width (which is how wide the rectangle is)
            //  Use 2 LUTs to convert those inputs (which are in pixels) into inches
            //        - Alignment will become DISTANCE_FROM_CENTER
            //        - Width will become DISTANCE_FROM_POLE
            //  Perform an ATAN calc to get ANGLE_TO_TURN
            //          I think it's: double ANGLE_TO_TURN = Math.atan(DISTANCE_FROM_CENTER/DISTANCE_FROM_POLE);
            //  Use pythagorean formula in which B is DISTANCE_FROM_CENTER, A is DISTANCE_FROM_POLE,
            //          and C is a new variable DISTANCE_TO_TRAVEL
            //  In PoleAimTele we will:
            //         .turn(ANGLE_TO_TURN)
            //         .forward(DISTANCE_TO_TRAVEL - OFFSET)
            //         where OFFSET is how far back from the pole we need to be to drop the cone

            // test change from Sophie

            return workingMatrix;
        }
    }

    public double getMid() {
        return midLine;
    }  // this is the midline position of the rectangle

    public double getWidth() {
        return width;
    }  // this is the width of the rectangle

    public VisionPipeline getVisionPipeline() {
        return visionPipeline;
    }

   // public double getAngle() {
  //      return angle.get(Range.clip(midLine, -webcamWidth / 2 + 0.01, webcamWidth / 2 - 0.01));
 //   }

  //  public double getDistance() {
   //     return distance.get(Range.clip(width, 5.01, 99.99));
 //   }


}


