/* Author: Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.google.gson.JsonArray;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.*;

import androidx.annotation.Nullable;
import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import java.util.*;

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
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
//            public void onOpened() {
//                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }


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
    final private RobotManager.AllianceColor allianceColor;

    boolean first = true;


    AutonPipeline(Robot robot, Telemetry telemetry, RobotManager.AllianceColor allianceColor) {
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

        try {
            cameraMatrix = CalibrationPipeline.MatFromFile(ComputerVision.DataDir + "/camera-matrix.json");
            distortionMatrix = new MatOfDouble(CalibrationPipeline.MatFromFile(ComputerVision.DataDir + "/distortion-matrix.json"));
        } catch (FileNotFoundException e) {
        }

        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
    }


    /**
     * The main pipeline method, called whenever a frame is received. It will execute all necessary CV tasks, such as localization and barcode scanning
     *
     * @param input The current frame read from the attached camera.
     *              NOTE: the camera will be mounted in landscape, so make sure to flip x/y coords
     * @return An output frame to be displayed on the phone
     */
    @Override
    public Mat processFrame(Mat input) {
        if (first) {
            Bitmap.Config conf = Bitmap.Config.ARGB_8888;
            Bitmap bm = Bitmap.createBitmap(input.cols(), input.rows(), conf);
            Utils.matToBitmap(input, bm);

            try {
                FileOutputStream fo = new FileOutputStream(Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/cvdata/firstimage.png");
                bm.compress(Bitmap.CompressFormat.PNG, 100, fo);
            }
            catch (FileNotFoundException e) {}
        }
//        processBarcodeFrame(input, output);
        input.copyTo(output);

        // Check if a barcode scan has been requested
        if (robot.barcodeScanState == Robot.BarcodeScanState.SCAN) {
            // Scan the barcode
            Robot.BarcodeScanResult result = processBarcodeFrame(input, output);

            // Increment the barcode result in the frequency counter and find the max value in that map
            int freq = robot.barcodeScanResultMap.get(result);
            robot.barcodeScanResultMap.put(result, freq + 1);

            Map.Entry<Robot.BarcodeScanResult, Integer> max = Collections.max(robot.barcodeScanResultMap.entrySet(), Comparator.comparingInt(Map.Entry::getValue));
            robot.numBarcodeAttempts++;

            if (robot.numBarcodeAttempts >= Robot.MaxBarcodeAttempts || max.getValue() >= Robot.MinBarcodeRepeat) {
                Map<Robot.BarcodeScanResult, Integer> fullResultMap = robot.barcodeScanResultMap;

                // Ensure that we don't end up with an invalid state as the most frequent. This will modify the map, so save a copy first.
                while (max.getKey() == Robot.BarcodeScanResult.WRONG_CAPS || max.getKey() == Robot.BarcodeScanResult.WRONG_TAPE) {
                    robot.barcodeScanResultMap.remove(max.getKey());
                    max = Collections.max(robot.barcodeScanResultMap.entrySet(), Comparator.comparingInt(Map.Entry::getValue));
                }

                robot.barcodeScanResult = max.getKey();
                robot.barcodeScanState = Robot.BarcodeScanState.CHECK_SCAN;
                robot.barcodeScanResultMap = fullResultMap;
            }
            else {
                telemetry.update();
                return output;  // We have more iterations of barcode scanning to do, so we needn't spend time on positioning
            }
        }

//        Position currentPosition = processPositioningFrame(input, output);
//        if (currentPosition != null) robot.positionManager.updateCvPosition(currentPosition);

        return output;
    }


    // CV POSITIONING
    // =================

    // TODO: May want to abstract this into a calibration class
    private Mat cameraMatrix = new Mat();
    private MatOfDouble distortionMatrix = new MatOfDouble();


    // Represents the distance from the center of the nav images to the edge. X/Y are in the paper 2d space.
    // TODO: determine where the template imgs start, and update these
    private final static double templateOffsetX = 11 / 2.0;
    private final static double templateOffsetY = 8.5 / 2.0;

    // The dimensions of the field tiles in inches
    private final static double tileSize = 24.0;

    /**
     * Represents each navigation image on the field walls, as well as necessary data about them.
     * Contains the image, coordinates, and alliance color corresponding to each nav image.
     */
    static enum NavTarget {
        BLUE_ALLIANCE_WALL(RobotManager.AllianceColor.BLUE, "/features3.jpg", Arrays.asList(
                new Point3(0, (3.5 * tileSize) + templateOffsetX, 5.75 - templateOffsetY), // br
                new Point3(0, (3.5 * tileSize) - templateOffsetX, 5.75 - templateOffsetY), // bl
                new Point3(0, (3.5 * tileSize) - templateOffsetX, 5.75 + templateOffsetY), // tl
                new Point3(0, (3.5 * tileSize) + templateOffsetX, 5.75 + templateOffsetY)  // tr
        )),
        BLUE_STORAGE_UNIT(RobotManager.AllianceColor.BLUE, "/features3.jpg", Arrays.asList(
                new Point3((1.5 * tileSize) - templateOffsetX, 0, 5.75 - templateOffsetY), // br
                new Point3((1.5 * tileSize) + templateOffsetX, 0, 5.75 - templateOffsetY), // bl
                new Point3((1.5 * tileSize) + templateOffsetX, 0, 5.75 + templateOffsetY), // tl
                new Point3((1.5 * tileSize) - templateOffsetX, 0, 5.75 + templateOffsetY)  // tr
        )),
        RED_ALLIANCE_WALL(RobotManager.AllianceColor.RED, "/features3.jpg", Arrays.asList(
                new Point3(6 * tileSize, (3.5 * tileSize) - templateOffsetX, 5.75 - templateOffsetY), // br
                new Point3(6 * tileSize, (3.5 * tileSize) + templateOffsetX, 5.75 - templateOffsetY), // bl
                new Point3(6 * tileSize, (3.5 * tileSize) + templateOffsetX, 5.75 + templateOffsetY), // tl
                new Point3(6 * tileSize, (3.5 * tileSize) - templateOffsetX, 5.75 + templateOffsetY)  // tr
        )),
        RED_STORAGE_UNIT(RobotManager.AllianceColor.RED, "/features3.jpg", Arrays.asList(
                new Point3((4.5 * tileSize) - templateOffsetX, 0, 5.75 - templateOffsetY), // br
                new Point3((4.5 * tileSize) + templateOffsetX, 0, 5.75 - templateOffsetY), // bl
                new Point3((4.5 * tileSize) + templateOffsetX, 0, 5.75 + templateOffsetY), // tl
                new Point3((4.5 * tileSize) - templateOffsetX, 0, 5.75 + templateOffsetY)  // tr
        ));


        public final RobotManager.AllianceColor allianceColor;
        public final MatOfPoint3f worldCoords;

        public final String path;
        public final Mat image;
        public final MatOfPoint2f imgCoords;

        public final MatOfKeyPoint keyPoints;
        public final Mat descriptors;


        private NavTarget(RobotManager.AllianceColor allianceColor, String path, List<Point3> worldCoords) {
            this.allianceColor = allianceColor;
            this.path = ComputerVision.DataDir + path;

            this.worldCoords = new MatOfPoint3f();
            this.worldCoords.fromList(worldCoords);

            // Read and process nav image from storage
            this.image = new Mat();
            Bitmap bitmap = BitmapFactory.decodeFile(this.path);
            Utils.bitmapToMat(bitmap, image);
            Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2GRAY);

            // Detect and cache template keypoints/descriptors
            this.keyPoints = new MatOfKeyPoint();
            this.descriptors = new Mat();
            DetectKeyPointsAndDesc(image, keyPoints, descriptors);

            // Determine the corners from the dims of the loaded image
            this.imgCoords = new MatOfPoint2f();
            this.imgCoords.fromList(Arrays.asList(
                    new org.opencv.core.Point(image.cols(), image.rows()),   // br
                    new org.opencv.core.Point(0, image.rows()),           // bl
                    new org.opencv.core.Point(0, 0),                   // tl
                    new org.opencv.core.Point(image.cols(), 0)            // tr
            ));
        }
    }


//    private static final ORB Orb = ORB.create(500, 1.2f, 8, 31, 0, 2, ORB.HARRIS_SCORE, 31, 20);
//    private static final SIFT Sift = SIFT.create(0, 3, 0.04, 10, 1.6);


    private static final SIFT Sift = SIFT.create(0, 5, 0.04, 10, 1.0);
    private static final ORB Orb = ORB.create(1000, 1.2f, 8, 31, 0, 2, ORB.FAST_SCORE, 31, 10);
    private static final BFMatcher BfMatcher = BFMatcher.create(BFMatcher.BRUTEFORCE_HAMMING, false);
    private static final FlannBasedMatcher FbMatcher = FlannBasedMatcher.create();


    // Single-time allocations for template detection
    private static final MatOfKeyPoint frameKeyPoints = new MatOfKeyPoint();
    private static final Mat frameDescriptors = new Mat();


    /**
     * Detects and computes keypoints and descriptors in a given frame. Abstracted for template keypoint caching.
     *
     * @param frame the image to be analyzed
     * @param kp    the output keypoints
     * @param des   the output descriptors
     */
    private static void DetectKeyPointsAndDesc(Mat frame, MatOfKeyPoint kp, Mat des) {
        Sift.detectAndCompute(frame, new Mat(), kp, des);
    }


    /** Detects and filters the best matches between two detected keypoint descriptor sets
     *  Either will end up using binary matching or a flann matcher.
     */
//    private static MatOfDMatch MatchAndFilter(Mat des1, Mat des2) {
//
//    }


    /** Detects a navigation target in a provided image, and determines the screen-space coordinates of the target
     * @param target The nav target to be detected. This enum contains the image of the template, the keypoints/descriptors,
     *               and the points to be transformed by the detected homography to determine the template's screen-space corners
     * @param frame The image in which the perspective-distorted template may (or may not) be found
     * @return The corners defined by the {@param target}, transformed into screen space (or null, if no nav target is found)
     */
    @Nullable
    private MatOfPoint2f DetectTargetScreenCorners(NavTarget target, Mat frame, Mat output) {
        DetectKeyPointsAndDesc(frame, frameKeyPoints, frameDescriptors);
//        Features2d.drawKeypoints(frame, frameKeyPoints, output);
//        return nulrl;

        // find a way to set knn params here
////        MatOfDMatch matchesB = new MatOfDMatch();
        ArrayList<MatOfDMatch> matches = new ArrayList<MatOfDMatch>();
        List<DMatch> goodMatches = new ArrayList<DMatch>(target.descriptors.rows());
////
////
        if (frameDescriptors.empty() || target.descriptors.empty()) return null;
        if (frameKeyPoints.total() < 2 && target.keyPoints.total() < 2) return null;
//
//        des1.convertTo(des1, CvType.CV_32F);
//        des2.convertTo(des2, CvType.CV_32F);
//
        FbMatcher.knnMatch(target.descriptors, frameDescriptors, matches, 2);
//
////        BfMatcher.knnMatch(des1, des2, matches, 2);
////        BfMatcher.match(des1, des2, matchesB);
////
////        for (int i = 0; i < des1.rows(); i++) {
////            goodMatches.add(matchesB.toList().get(i));
////        }
////
////        Collections.sort(goodMatches, new Comparator<DMatch>() {
////            public int compare(DMatch m1, DMatch m2) {
////                return Double.compare(m1.distance, m2.distance);
////            }
////        });
//
////        List<DMatch> matches =
//
////
        for (MatOfDMatch matchSet : matches) {
            DMatch[] matchSetArr = matchSet.toArray();
            if (matchSetArr.length < 2)
                continue;

            DMatch m1 = matchSetArr[0];
            DMatch m2 = matchSetArr[1];

            if (m1.distance < 0.7 * m2.distance)
                goodMatches.add(m1);

//            if (m2.distance - m1.distance > 0.19)
//                goodMatches.add(m1);
        }
//
////
        MatOfDMatch goodMatchesMat = new MatOfDMatch();
        goodMatchesMat.fromList(goodMatches/*.subList(0, Math.min(goodMatches.size(), 10))*/);
//
////        Features2d.drawMatches(template, kp1, frame, kp2, goodMatchesMat, output);
//        Imgproc.resize(output, output, frame.size(), 0, 0, Imgproc.INTER_CUBIC);
////        return null;
        if (goodMatches.size() < 10) return null;
//
        ArrayList<org.opencv.core.Point> obj = new ArrayList<org.opencv.core.Point>();
        ArrayList<org.opencv.core.Point> scene = new ArrayList<org.opencv.core.Point>();

        for (DMatch match : goodMatches) {
            obj.add(target.keyPoints.toList().get(match.queryIdx).pt);
            scene.add(frameKeyPoints.toList().get(match.trainIdx).pt);
        }
//
        MatOfPoint2f objM = new MatOfPoint2f(), sceneM = new MatOfPoint2f();
        objM.fromList(obj);
        sceneM.fromList(scene);
//
        Mat homography = Calib3d.findHomography(objM, sceneM, Calib3d.RANSAC, 5.0, new Mat());
//        Mat homography = Calib3d.findHomography(objM, sceneM, Calib3d.LMEDS);
        if (homography.empty()) return null;

        MatOfPoint2f result = new MatOfPoint2f();
        Core.perspectiveTransform(target.imgCoords, result, homography);
//
//        // homography LMeDS
        return result;
    }



    /** Main CV localization estimator - determines position relative to any detected navigation targets
     * @param input The frame to be processed (either containing nav targets or not)
     * @return Determined position of the robot, as presented in the {@param input} image.
     */
    @Nullable
    Position processPositioningFrame(Mat input, Mat output) {
        input.copyTo(output);

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);
        NavTarget t = NavTarget.RED_STORAGE_UNIT;
        telemetry.addData("test", t.imgCoords.toString());
        telemetry.update();

//        for (NavTarget t : NavTarget.values()) {
//            if (t.allianceColor == allianceColor) {
                MatOfPoint2f screenCorners = DetectTargetScreenCorners(t, input, output);

//                if (screenPoints == null) continue;
                if (screenCorners == null) return null;

                Imgproc.line(output, screenCorners.toArray()[0], screenCorners.toArray()[1], new Scalar(0, 255, 0), 4);
                Imgproc.line(output, screenCorners.toArray()[1], screenCorners.toArray()[2], new Scalar(0, 0, 255), 4);
                Imgproc.line(output, screenCorners.toArray()[2], screenCorners.toArray()[3], new Scalar(255, 0, 0), 4);
                Imgproc.line(output, screenCorners.toArray()[3], screenCorners.toArray()[0], new Scalar(0, 255, 255), 4);

                // TODO: Clean this into a real-world coord finder method
                Mat tvec = new Mat(), rvec = new Mat();

                // TODO: Determine which algorithm is most appropriate here
                // Calib3d.solvePnPRansac(worldCoords, screenPoints, cameraMatrix, distortionMatrix, rvec, tvec);
                Calib3d.solvePnP(t.worldCoords, screenCorners, cameraMatrix, distortionMatrix, rvec, tvec);

                telemetry.addData("rvec", rvec.dump());
                telemetry.addData("tvec", tvec.dump());
                telemetry.update();

//                break;
//            }
//        }

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
    final static Scalar[] BarcodeCapRange      = {new Scalar(15, 100, 50), new Scalar(80, 255, 255)};
    final static Scalar[] BarcodeTapeRangeBlue = {new Scalar(100, 100, 50), new Scalar(130, 255, 255)};
    final static Scalar[] BarcodeTapeRangeRed1 = {new Scalar(170, 100, 50), new Scalar(180, 255, 255)};
    final static Scalar[] BarcodeTapeRangeRed2 = {new Scalar(0,   100, 50), new Scalar(10,  255, 255)};


    static final Size NoiseSize = new Size(5, 5);

    /** Isolates the sections of an image in a given HSV range and removes noise, to find large solid-color areas
     * @param hsv The input image to be isolated, in HSV color format
     * @param out The image in which the detected areas will be stored
     * @param a HSV color in Scalar format that represents the lower bound of the area to be isolated
     * @param b HSV color in Scalar format that represents the upper bound of the area to be isolated
     * NOTE: OpenCV represents hue from 0-180
     */
    private static void IsolateBarcodeRange(Mat hsv, Mat out, Scalar a, Scalar b) {
        Core.inRange(hsv, a, b, out);

        Imgproc.morphologyEx(out, out, Imgproc.MORPH_CLOSE, Mat.ones(NoiseSize, CvType.CV_32F));
        Imgproc.morphologyEx(out, out, Imgproc.MORPH_OPEN, Mat.ones(NoiseSize, CvType.CV_32F));

//        Imgproc.morphologyEx(out, out, Imgproc.MORPH_CLOSE, Mat.ones(new Size(25, 25), CvType.CV_32F));
//        Imgproc.morphologyEx(out, out, Imgproc.MORPH_OPEN, Mat.ones(new Size(25, 25), CvType.CV_32F));
    }


    /**
     * @param input The current frame containing the barcode to be scanned
     * @return an integer in the interval [-1, 2], where -1 denotes no result, and 0-2 represent positions (in screen space) of the object of interest
     */
    private Robot.BarcodeScanResult processBarcodeFrame(Mat input, Mat output) {
        // Todo: perform cropping based on region of image we expect to find barcode in
        // input = new Mat(input, BarcodeImageROI);

        // Convert input image to HSV space and perform basic blur
        Imgproc.cvtColor(input, barcodeHsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(barcodeHsv, barcodeHsv, new Size(7, 7), 5);

        // Do HSV thresholding to identify the barcode tape as well as the shipping element
        IsolateBarcodeRange(barcodeHsv, barcodeCapRegions, BarcodeCapRange[0], BarcodeCapRange[1]);

        // HSV thresholding for barcode tape isolation
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsRed1, BarcodeTapeRangeRed1[0], BarcodeTapeRangeRed1[1]);
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsRed2, BarcodeTapeRangeRed2[0], BarcodeTapeRangeRed2[1]);
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsBlue, BarcodeTapeRangeBlue[0], BarcodeTapeRangeBlue[1]);

        Core.bitwise_or(barcodeTapeRegionsRed1, barcodeTapeRegionsRed2, barcodeTapeRegions);
        Core.bitwise_or(barcodeTapeRegionsBlue, barcodeTapeRegions, barcodeTapeRegions);

        // Visualize the detected areas with appropriately colored outlines
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(barcodeCapRegions, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

//        Draw the detected areas to the output for visualization
        input.copyTo(output);

        for (int idx = 0; idx < contours.size(); idx++) {
            Imgproc.drawContours(output, contours, idx, new Scalar(255, 255, 0), 6);
        }

        contours.clear();
        Imgproc.findContours(barcodeTapeRegions, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (int idx = 0; idx < contours.size(); idx++) {
            Imgproc.drawContours(output, contours, idx, new Scalar(255, 0, 0), 6);
        }

        // Determine the centroids of the tape regions
        double[] tapeCentroidsX = new double[2];
        int tapeComponentsCount = Imgproc.connectedComponentsWithStats(barcodeTapeRegions, barcodeTapeLabels, barcodeTapeStats, barcodeTapeCentroids, 8);

        // For now, we'll make sure that we're identifying only two non-cap tapes.
        if (tapeComponentsCount != 3) return Robot.BarcodeScanResult.WRONG_TAPE;
        for (int i = 1; i < tapeComponentsCount; i++) tapeCentroidsX[i - 1] = barcodeTapeCentroids.at(double.class, i, 1).getV();

        // Make sure the centroids are listed in ascending order of X-coordinate (left-to-right, in screen space)
        Arrays.sort(tapeCentroidsX);

        // Determine the centroid of the cap region
        int capComponentsCount = Imgproc.connectedComponentsWithStats(barcodeCapRegions, barcodeCapLabels, barcodeCapStats, barcodeCapCentroids, 8);
        if (capComponentsCount != 2) return Robot.BarcodeScanResult.WRONG_CAPS;
        double capCentroidX = barcodeCapCentroids.at(double.class, 1, 1).getV();


        if (capCentroidX < tapeCentroidsX[0]) return Robot.BarcodeScanResult.LEFT;
        else if (capCentroidX < tapeCentroidsX[1]) return Robot.BarcodeScanResult.CENTER;
        return Robot.BarcodeScanResult.RIGHT;
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
    static final int MaxDisplayCaptureDelay = 10;
    static final int NumTotalCaptures = 30;

    final static Size BoardSize = new Size(10, 7);

    // the dimensions of a single square on the board, in inches
    final static double boardSquareSize = 0.845;
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


    static private class MatJsonFmt {
        int rows;
        int cols;
        int type;
        double[] data;
    }

    final static private Gson gson = new Gson();

    /** Reference: https://answers.opencv.org/question/8873/best-way-to-store-a-mat-object-in-android/
     * @param mat Opencv Mat to be written to file
     */
    static private String MatToJson(Mat mat) {
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
    static private Mat MatFromJson(String json) {
        MatJsonFmt fmt = gson.fromJson(json, MatJsonFmt.class);

        Mat mat = new Mat(fmt.rows, fmt.cols, fmt.type);
        mat.put(0, 0, fmt.data);
        return mat;
    }


    public static void MatToFile(Mat mat, String path) {
        try (FileOutputStream stream = new FileOutputStream(path)) {
            stream.write(MatToJson(mat).getBytes());
        } catch (IOException e) {}
    }


    public static Mat MatFromFile(String path) throws FileNotFoundException {
        File fl = new File(path);
        FileInputStream fin = new FileInputStream(fl);
        BufferedReader reader = new BufferedReader(new InputStreamReader(fin));
        StringBuilder sb = new StringBuilder();
        String line = null;

        try {
            while ((line = reader.readLine()) != null) {
                sb.append(line).append("\n");
            }

            reader.close();
            fin.close();
        } catch (IOException e) {}


        return MatFromJson(sb.toString());
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

