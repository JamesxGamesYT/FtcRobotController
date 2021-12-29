package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonArray;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.Nullable;
import android.os.Environment;
import android.util.Base64;

import com.google.gson.Gson;
import com.google.gson.JsonParser;
import com.google.gson.JsonObject;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import org.opencv.android.Utils;
import android.graphics.BitmapFactory;
import android.graphics.Bitmap;

import org.opencv.core.*;

import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;

import org.openftc.easyopencv.*;


/** Managing class for opening cameras, attaching pipelines, and beginning streaming.
 */
public class ComputerVision {

    public OpenCvCamera camera;
    public OpenCvPipeline pipeline;

    public static String DataDir = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/cvdata";


    ComputerVision(HardwareMap hardwareMap, OpenCvPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        this.pipeline = pipeline;
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

            // TODO: responsible error handling
            @Override
            public void onError(int errorCode) {}
        });
    }
}


/** Contains all image processing done for scanning the barcode and getting position.
 * This will
 */
class AutonPipeline extends OpenCvPipeline {
    final private Robot robot;
    final private Telemetry telemetry;
    final private Mat output;  // Frame to be displayed on the phone


    AutonPipeline(Robot robot, Telemetry telemetry) {
        super();
        this.robot = robot;

        // TODO: there might be a cleaner way to default-initialize these
        output = new Mat();

        // Initialize barcode data
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

        this.telemetry = telemetry;
    }


    /** The main pipeline method, called whenever a frame is received. It will execute all necessary CV tasks, such as localization and barcode scanning
     *  @param input The current frame read from the attached camera.
     *               NOTE: the camera will be mounted in landscape, so make sure to flip x/y coords
     *  @return An output frame to be displayed on the phone
     */
    @Override
    public Mat processFrame(Mat input) {
        // Check if a barcode scan has been requested
        if (robot.barcodeScanState == Robot.BarcodeScanState.SCAN) {
            int result = processBarcodeFrame(input);
            robot.numBarcodeAttempts++;

            if (robot.numBarcodeAttempts >= Robot.MaxBarcodeAttempts || result != -1) {
                robot.barcodeScanResult = result; // result may still be -1, meaning no result
                robot.barcodeScanState = Robot.BarcodeScanState.CHECK_SCAN;
            }
            else return input;  // We have more iterations of barcode scanning to do, so don't waste time on positioning
        }

        Position currentPosition = processPositioningFrame(input, output);
        if (currentPosition != null) robot.positionManager.updateCvPosition(currentPosition);

        return output;
    }



    // CV POSITIONING
    // =================
    private static final SIFT Sift = SIFT.create();
    private static final ORB Orb = ORB.create();
    private static final BFMatcher BfMatcher = BFMatcher.create(BFMatcher.BRUTEFORCE_HAMMING, false);


    private static final MatOfKeyPoint kp1 = new MatOfKeyPoint(), kp2 = new MatOfKeyPoint();
    private static final Mat des1 = new Mat(), des2 = new Mat();

//    enum NavTargets {ALLIANCE}

    static List<Point3> NavTargetsWorldSpace = new ArrayList<Point3> () {{
        add(new Point3(0,42 + (5.5),5.75 - (8.5 / 2))); // br
        add(new Point3(0,42 - (5.5),5.75 - (8.5 / 2))); // bl
        add(new Point3(0,42 - (5.5),5.75 + (8.5 / 2))); // tl
        add(new Point3(0,42 + (5.5),5.75 + (8.5 / 2))); // tr
    }};


    /** Transforms a set of points on a provided 2d template image onto the instance of the template in frame using the detected homography transformation.
     * or maybe it will do something else like just returning the transformation.
     * @param template A flat image to be detected in the frame
     * @param frame The image in which the 3d-distorted {@param template} may be found
     * param points A set of points, in the coordinate space defined by {@param template}, to be transformed into the coordinate space of {@param frame}
     * @param output An image to draw the detected KeyPoint matches to
     * @return The input {@param points}, transformed into the destination space
     */
    @Nullable
    private static MatOfPoint2f DetectTemplate(Mat template, Mat frame, MatOfPoint2f points, Mat output) {
        Orb.detectAndCompute(template, new Mat(), kp1, des1);
        Orb.detectAndCompute(frame, new Mat(), kp2, des2);

        // find a way to set knn params here
//        FlannBasedMatcher matcher = FlannBasedMatcher.create();
        MatOfDMatch matchesB = new MatOfDMatch();
//        ArrayList<MatOfDMatch> matches = new ArrayList<MatOfDMatch>();
        ArrayList<DMatch> goodMatches = new ArrayList<DMatch>();


        if (des1.empty() || des2.empty()) return null;
        if (kp1.total() < 2 && kp2.total() < 2) return null;

        des1.convertTo(des1, CvType.CV_32F);
        des2.convertTo(des2, CvType.CV_32F);
//        matcher.knnMatch(des1, des2, matches, 2);

//        BfMatcher.knnMatch(des1, des2, matches, 2);
        BfMatcher.match(des1, des2, matchesB);

        for (int i = 0; i < des1.rows(); i++) {
            if ((matchesB.toList().get(i)).distance < 200) goodMatches.add(matchesB.toList().get(i));
        }
//
//        for (MatOfDMatch matchSet : matches) {
//            DMatch[] matchSetArr = matchSet.toArray();
//            if (matchSetArr.length < 2)
//                continue;
//
//            DMatch m1 = matchSetArr[0];
//            DMatch m2 = matchSetArr[1];
//
//            if (m1.distance < 0.7 * m2.distance)
//                goodMatches.add(m1);
//
////            if (m2.distance - m1.distance > 0.19)
////                goodMatches.add(m1);
//        }
//

        MatOfDMatch goodMatchesMat = new MatOfDMatch();
        goodMatchesMat.fromList(goodMatches);
        Features2d.drawMatches(template, kp1, frame, kp2, goodMatchesMat, output);
        Imgproc.resize(output, output, frame.size(), 0, 0, Imgproc.INTER_CUBIC);

        if (goodMatches.size() < 10) return null;

        ArrayList<org.opencv.core.Point> obj = new ArrayList<org.opencv.core.Point>();
        ArrayList<org.opencv.core.Point> scene = new ArrayList<org.opencv.core.Point>();

        for (DMatch match : goodMatches) {
            obj.add(kp1.toList().get(match.queryIdx).pt);
            scene.add(kp2.toList().get(match.trainIdx).pt);
        }

        MatOfPoint2f objM = new MatOfPoint2f(), sceneM = new MatOfPoint2f();
        objM.fromList(obj);
        sceneM.fromList(scene);

//        Mat homography = Calib3d.findHomography(objM, sceneM, Calib3d.RANSAC, 5.0, new Mat());
        Mat homography = Calib3d.findHomography(objM, sceneM, Calib3d.LMEDS);

        MatOfPoint2f result = new MatOfPoint2f();
        Core.perspectiveTransform(points, result, homography);

        // homography LMeDS
        return result;
    }


    Mat template1 = new Mat();

    /** Main CV localization estimator - determines position relative to any detected navigation targets2
     * @param input The frame to be processed (either containing nav targets or not)
     * //@param output
     * @return Determined pos("Stuff", frame.at(Point3.class, 0, 0).getV());
        telemetry.update(ition of the robot, as presented in the {@param input} image.
     */
    @Nullable
    Position processPositioningFrame(Mat input, Mat output) {
        Bitmap bitmap = BitmapFactory.decodeFile(Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/navimgs/features3.png");
        Utils.bitmapToMat(bitmap, template1);

        input.copyTo(output);

        Imgproc.cvtColor(template1, template1, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);

        List<org.opencv.core.Point> templateCorners = new ArrayList<org.opencv.core.Point> () {{
            add(new org.opencv.core.Point(template1.cols(), template1.rows()));     // br
            add(new org.opencv.core.Point(0, template1.rows()));                 // bl
            add(new org.opencv.core.Point(0, 0));                             // tl
            add(new org.opencv.core.Point(template1.cols(), 0));                 // tr
        }};

        MatOfPoint2f points = new MatOfPoint2f();
        points.fromList(templateCorners);

        MatOfPoint2f screenPoints = DetectTemplate(template1, input, points, output);

        if (screenPoints == null) return null;
        Imgproc.line(output, screenPoints.toArray()[0], screenPoints.toArray()[1], new Scalar(0, 255, 0), 4);
        Imgproc.line(output, screenPoints.toArray()[1], screenPoints.toArray()[2], new Scalar(0, 0, 255), 4);
        Imgproc.line(output, screenPoints.toArray()[2], screenPoints.toArray()[3], new Scalar(255, 0, 0), 4);
        Imgproc.line(output, screenPoints.toArray()[3], screenPoints.toArray()[0], new Scalar(0, 255, 255), 4);
        return null;
//        Calib3d.solvePnP(NavTargetsWorldSpace, screenPoints, )
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
    private static void IsolateBarcodeRange(Mat hsv, Mat out, Scalar a, Scalar b)
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





/** Pipeline to perform intrinsic camera calibration with user-defined snapshots
 *
 */
class CalibrationPipeline extends OpenCvPipeline {

    final private Mat output;

    boolean captureRequested;
    boolean doCapture;
    boolean displayCapture;
    int displayCaptureFrames = 0;
    static final int MaxDisplayCaptureDelay = 20;
    static final int NumTotalCaptures = 1;

    final static Size BoardSize = new Size(10, 7);
    final static double boardSquareSize = 1.0;  // TODO: measure this
    static MatOfPoint3f CheckerboardWorldCoords = new MatOfPoint3f();


    final List<Mat> imgCorners = new ArrayList<Mat>();  // 2d coordinates of checkerboard corners for each frame
    final List<Mat> objCorners = new ArrayList<Mat>();  // 3d corners of the checkerboard (each element should be identical)

    final Mat lastCapture = new Mat();



    CalibrationPipeline() {
        super();
        output = new Mat();
        doCapture = false;
        displayCapture = false;

        List<Point3> checkerboardWorldCoordsList = new ArrayList<>();
        for (int i = 0; i < BoardSize.height; i++) {
            for (int j = 0; j < BoardSize.width; j++) {
                checkerboardWorldCoordsList.add(new Point3((double)j * boardSquareSize, (double)i * boardSquareSize, 0));
            }
        }

        CheckerboardWorldCoords.fromList(checkerboardWorldCoordsList);
    }


    static public class MatJsonFmt {
        int rows;
        int cols;
        int type;
        float[] data;
    }

    final static private Gson gson = new Gson();

    /** Reference: https://answers.opencv.org/question/8873/best-way-to-store-a-mat-object-in-android/
     * @param mat Opencv Mat to be written to file
     */
    public static String MatToJson(Mat mat) {
        JsonObject obj = new JsonObject();

        if (!mat.isContinuous()) return "{}";

        double[] data = new double[(int)mat.total() * mat.channels()];
        mat.get(0, 0, data);
//        return "test";
//
//        String dataString = new String(Base64.encode(data, Base64.DEFAULT));
        JsonArray dataArr = gson.toJsonTree(data).getAsJsonArray();

        obj.addProperty("rows", mat.rows());
        obj.addProperty("cols", mat.cols());
        obj.addProperty("type", mat.type());
        obj.add("data", dataArr);
        return gson.toJson(obj);
    }

    /** Reference: https://answers.opencv.org/question/8873/best-way-to-store-a-mat-object-in-android/
     * @param json JSON string to be parsed to an OpenCV Mat
     */
    public static Mat MatFromJson(String json) {
        MatJsonFmt fmt = gson.fromJson(json, MatJsonFmt.class);

        Mat mat = new Mat(fmt.rows, fmt.cols, fmt.type);
        mat.put(0, 0, fmt.data);
        return mat;
    }


    public Mat cameraMatrix = new Mat();
    public Mat distortionMatrix = new Mat();
    public boolean calibrated = false;

    @Override
    public Mat processFrame(Mat input) {

        // Check if we've got enough calibration frames
        if (imgCorners.size() >= NumTotalCaptures && !calibrated) {
            Calib3d.calibrateCamera(objCorners, imgCorners, input.size(), cameraMatrix, distortionMatrix, new ArrayList<Mat>(), new ArrayList<Mat>());
            calibrated = true;
            return input;
        }

        if (doCapture) {
            input.copyTo(lastCapture);
            MatOfPoint2f imgP = Calibrate(input, lastCapture);
            if (imgP != null) {
                imgCorners.add(imgP);
                objCorners.add(CheckerboardWorldCoords);
                displayCapture = true;
            }

            doCapture = false;
            captureRequested = false;
        }

        if (captureRequested) {
            doCapture = true;
            Imgproc.putText(input, "Detecting...", new org.opencv.core.Point(10.0, 50.0), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 0, 0), 4);                               // Thickness
        }

        if (displayCapture) {
            displayCaptureFrames++;

            if (displayCaptureFrames >= MaxDisplayCaptureDelay) {
                displayCapture = false;
                displayCaptureFrames = 0;
            }

            return lastCapture;
        }

        return input;
    }


    @Nullable
    static MatOfPoint2f Calibrate(Mat input, Mat output) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
        MatOfPoint2f corners = new MatOfPoint2f();

        boolean found = Calib3d.findChessboardCorners(input, BoardSize, corners, Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_FILTER_QUADS);

        if (!found) return null;

        Calib3d.find4QuadCornerSubpix(input, corners, new Size(10, 10));
        Calib3d.drawChessboardCorners(output, BoardSize, corners, true);
        return corners;
    }


    @Override
    public void onViewportTapped() {
        if (!calibrated) captureRequested = true;
    }
}

