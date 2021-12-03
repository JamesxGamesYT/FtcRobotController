package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;


public class CVPositioning {
    public OpenCvCamera camera;

    CVPositioning(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    }


    /**
     * Begins continuous frame acquisition and image processing.
     * NOTE: We'll need some instance of a robot class or position at this point to attach to the pipeline, since once started it will be thread inaccessible
     */
    public void startStreaming(Robot robot) {
        camera.setPipeline(new CVPositioningPipeline(robot));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /** This will be called if the camera could not be opened
                 */
            }
        });
    }


    /**
     * Runs one cycle of a position estimate based on ac single shot frame.
     * @return The estimated position. Will be null if no frame is detectable (or maybe we use an exception here)
     */
    public Position getPositionEstimate() {return null;}


    /**
     * Potentially camera-specific methods that we may want to generalize into an acquisition class
     */
    // private Mat currentFrame;
    // static void UpdateFrame(Mat frame) {}
}



class CVPositioningPipeline extends OpenCvPipeline {

    Robot robot;

    CVPositioningPipeline(Robot robot) {
        super();
        this.robot = robot;
    }


    /**
     * @param input The current frame read from the attached camera
     * @return An output frame to be displayed on the phone
     */
    @Override
    public Mat processFrame(Mat input) {
        if (robot.barcodeScanState == Robot.BarcodeScanState.SCAN) {
            int result = processBarcodeFrame(input);
            robot.numBarcodeAttempts++;

            if (robot.numBarcodeAttempts >= Robot.MaxBarcodeAttempts || result != -1) {
                robot.barcodeScanResult = result;
                robot.barcodeScanState = Robot.BarcodeScanState.CHECK_SCAN;
            }
        }


        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(input, input,190, 255, Imgproc.THRESH_BINARY);

        // Imgproc.adaptiveThreshold(input, input, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 31, 20);
        // Imgproc.adaptiveThreshold(input, input,255, Imgproc.ADAPTIVE_THRESH_MEAN_C,Imgproc.THRESH_BINARY, 15, 40);
        // Imgproc.morphologyEx(input, input, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3,3)));

        return input;
    }


    /**
     * @param input The current framecontaining the barcode to be scanned
     * @return an integer in the interval [-1, 3], where -1 denotes no/invalid result, and each subsequent entry
     *         represents a result of the
     */
    private int processBarcodeFrame(Mat input) {
        return -1;
    }


    @Override
    public void onViewportTapped() {
//        phoneCam.pauseViewport();
    }
}

