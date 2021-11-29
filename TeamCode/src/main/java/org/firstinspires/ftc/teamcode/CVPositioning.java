package org.firstinspires.ftc.teamcode;


/** Estimates the robot's position using the provided navigation images. This gives us a positioning method
 *  in which error does not compound, however it may not always be possible to obtain a value using this method.
 */
public class CVPositioning {

    /** Runs one cycle of a position estimate based on a single shot frame.
     *  @return The estimated position. Will be null if no frame is detectable (or maybe we use an exception here)
     */
    public Position getPositionEstimate() {return null;}


    /** param input The current frame read from the attached camera
     *  @return An output frame to be displayed on the phone
     */
//    static private Mat ProcessFrame(Mat input) {return null;}

    /** Updates the CV position estimate in the robot's PositionManager
     */
    public void submitEstimate(Robot robot) {}

    /** Potentially camera-specific methods that we may want to generalize into an acquisition class
     */
    // private Mat currentFrame;
    // static void UpdateFrame(Mat frame) {}
}