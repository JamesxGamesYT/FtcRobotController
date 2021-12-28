package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Incorporates estimates from multiple sources to create a single positioning estimate
 */
public class PositionManager {
    PositionManager(HardwareMap hardwareMap) {
        position = new Position();
        encoderPositioning = new EncoderPositioning();
        cvPositioning = new CVPositioning(hardwareMap);
    }

    PositionManager(double x, double y, double r){
        position = new Position(x, y, r);
    }

    public Position position;

    /** Calls all appropriate sensor update methods to get an updated estimate of the Robot's current position
     * @param robot Robot object whose estimate should be updated
     */
    public void updatePosition(Robot robot) {
        encoderPositioning.getDeltaEstimate(robot);
    }


    public EncoderPositioning encoderPositioning;
    public CVPositioning cvPositioning;

    /** Adds new detected encoder movement change to both a temporary encoderDelta variable and to the overall position attribute
     *
     *  @param delta A delta position represented as a vector from the last seen position.
     *               e.g. delta = Position(1, 1, 0) would mean a movement of 1 inch on all axis with no rotation
     */
    private void updateEncoderPosition(Position delta) {}

    /** Sets the encoder delta back to (0, 0, 0).
     *  To be called when an image is received, so that the delta stores the movement done during image processing
     */
    public void resetEncoderDelta() {}


    private Position encoderDelta;
    private Position cvEstimate;
}
