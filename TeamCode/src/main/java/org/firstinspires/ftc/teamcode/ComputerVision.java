package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

//import org.opencv.features2d.SIFT;
//import org.opencv.calib3d.Calib3d;
//import org.opencv.features2d.Features2d;
//import org.opencv.features2d.SIFT;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;

import org.openftc.easyopencv.*;

import java.util.Arrays;
import java.util.ArrayList;


public class ComputerVision {
    public OpenCvCamera camera;
    public ComputerVisionPipeline pipeline;

    ComputerVision(HardwareMap hardwareMap, Robot robot) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ComputerVisionPipeline(robot);
    }


    /** Begins continuous frame acquisition and image processing.
     */
    public void startStreaming() {

        camera.setPipeline(pipeline);
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
}



class ComputerVisionPipeline extends OpenCvPipeline {
    ComputerVisionPipeline(Robot robot) {
        super();
        this.robot = robot;


        // TODO: there might be a cleaner way to default-initialize these
        output = new Mat();
        barcodeHsv = new Mat();

        barcodeTapeRegions = new Mat();
        barcodeTapeLabels = new Mat();
        barcodeTapeStats = new Mat();
        barcodeTapeCentroids = new Mat();

        barcodeCapRegions = new Mat();
        barcodeCapLabels = new Mat();
        barcodeCapStats = new Mat();
        barcodeCapCentroids = new Mat();
        barcodeTapeRegionsBlue = new Mat();
        barcodeTapeRegionsRed1 = new Mat();
        barcodeTapeRegionsRed2 = new Mat();

    }


    private Robot robot;
    private Mat output;


    /** The main pipeline method, called whenever a frame is received
     * @param input The current frame read from the attached camera.
     *              NOTE: the camera will be mounted in landscape, so make sure to flip x/y coords
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


        Position currentPosition = processPositioningFrame(input);
        if (currentPosition != null) robot.positionManager.updateCvPosition(currentPosition);

        return output;
    }


    Position processPositioningFrame(Mat input) {
        return null;
    }


    // BARCODE SCANNING
    // =================


    // Single-time allocated mats that will hold frame processing data
    final private Mat barcodeHsv;
    final private Mat barcodeCapRegions, barcodeTapeRegions;
    final private Mat barcodeTapeRegionsBlue, barcodeTapeRegionsRed1, barcodeTapeRegionsRed2;
    final private Mat barcodeTapeLabels, barcodeTapeStats, barcodeTapeCentroids, barcodeCapLabels, barcodeCapStats, barcodeCapCentroids;

    // The Region of Interest that contains all the barcode elements and the least non-floor background possible
    final static Rect BarcodeImageROI = new Rect(220, 120, 500, 230);

    // Define HSV scalars that represent ranges of color to be selected from the barcode image
    final static Scalar[] BarcodeCapRange      = {new Scalar(15, 100, 50), new Scalar(45, 255, 255)};
    final static Scalar[] BarcodeTapeRangeBlue = {new Scalar(100, 100, 50), new Scalar(130, 255, 255)};
    final static Scalar[] BarcodeTapeRangeRed1 = {new Scalar(170, 100, 50), new Scalar(180, 255, 255)};
    final static Scalar[] BarcodeTapeRangeRed2 = {new Scalar(0,   100, 50), new Scalar(10,  255, 255)};

    /** Isolates the sections of an image in a given HSV range and removes noise, to find large solid-color areas
     * @param hsv The input image to be isolated, in HSV color format
     * @param out The image in which the detected areas will be stored
     * @param a HSV color in Scalar format that represents the lower bound of the area to be isolated
     * @param b HSV color in Scalar format that represents the upper bound of the area to be isolated
     * NOTE: OpenCV represents hue from 0-180
     */
    static void IsolateBarcodeRange(Mat hsv, Mat out, Scalar a, Scalar b)
    {
        Core.inRange(hsv, a, b, out);
        Imgproc.morphologyEx(out, out, Imgproc.MORPH_CLOSE, Mat.ones(new Size(25, 25), CvType.CV_32F));
        Imgproc.morphologyEx(out, out, Imgproc.MORPH_OPEN, Mat.ones(new Size(25, 25), CvType.CV_32F));
    }


    /**
     * @param input The current frame containing the barcode to be scanned
     * @return an integer in the interval [-1, 2], where -1 denotes no result, and 0-2 represent positions (in screen space) of the object of interest
     */
    private int processBarcodeFrame(Mat input/*, Mat output*/) {
        // Todo: perform cropping based on region of image we expect to find barcode in
        // input = new Mat(input, BarcodeImageROI);

        // Convert input image to HSV space and perform basic blur
        Imgproc.cvtColor(input, barcodeHsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(barcodeHsv, barcodeHsv, new Size(7, 7), 5);

        // Do HSV thresholding to identify the barcode tape as well as the shipping element
        IsolateBarcodeRange(barcodeHsv, barcodeCapRegions, BarcodeCapRange[0], BarcodeCapRange[0]);

        // HSV thresholding for barcode tape isolation
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsRed1, BarcodeTapeRangeRed1[0], BarcodeTapeRangeRed1[1]);
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsRed2, BarcodeTapeRangeRed2[0], BarcodeTapeRangeRed2[1]);
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsBlue, BarcodeTapeRangeBlue[0], BarcodeTapeRangeBlue[1]);

        Core.bitwise_or(barcodeTapeRegionsRed1, barcodeTapeRegionsRed2, barcodeTapeRegions);
        Core.bitwise_or(barcodeTapeRegionsBlue, barcodeTapeRegions, barcodeTapeRegions);

        // Visualize the detected areas with appropriately colored outlines
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(barcodeCapRegions, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Draw the detected areas to the output for visualization
//        input.copyTo(output);
//
//        for (int idx = 0; idx < contours.size(); idx++) {
//            Imgproc.drawContours(output, contours, idx, new Scalar(255, 255, 0), 6);
//        }
//
//        contours.clear();
//        Imgproc.findContours(barcodeTapeRegions, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//
//        for (int idx = 0; idx < contours.size(); idx++) {
//            Imgproc.drawContours(output, contours, idx, new Scalar(255, 0, 0), 6);
//        }

        // Determine the centroids of the tape regions
        double[] tapeCentroidsX = new double[2];
        int tapeComponentsCount = Imgproc.connectedComponentsWithStats(barcodeTapeRegions, barcodeTapeLabels, barcodeTapeStats, barcodeTapeCentroids, 8);

        // For now, we'll make sure that we're identifying only two non-cap tapes.
        if (tapeComponentsCount != 3) return -1;
        for (int i = 1; i < tapeComponentsCount; i++) tapeCentroidsX[i - 1] = barcodeTapeCentroids.at(double.class, i, 1).getV();

        // Make sure the centroids are listed in ascending order of X-coordinate (left-to-right, in screen space)
        Arrays.sort(tapeCentroidsX);

        // Determine the centroid of the cap region
        int capComponentsCount = Imgproc.connectedComponentsWithStats(barcodeCapRegions, barcodeCapLabels, barcodeCapStats, barcodeCapCentroids, 8);
        if (capComponentsCount != 2) return -2;
        double capCentroidX = barcodeCapCentroids.at(double.class, 1, 1).getV();

        if (capCentroidX < tapeCentroidsX[0]) return 1;
        else if (capCentroidX < tapeCentroidsX[1]) return 2;
        return 3;
    }


    @Override
    public void onViewportTapped() {
//        camera.pauseViewport();
    }
}

