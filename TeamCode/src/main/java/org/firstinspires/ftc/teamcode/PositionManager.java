package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Incorporates estimates from multiple sources to create a single positioning estimate
 */
public class PositionManager {
    PositionManager() {
        position = new Position();
        encoderPositioning = new EncoderPositioning();
    }

    PositionManager(double x, double y, double theta){
        position = new Position(x, y, theta);
        encoderPositioning = new EncoderPositioning();
    }

    // Stores the best guess of the robot's position (location + orientation) at any given time. To be accessed by nav methods
    public Position position;

    public EncoderPositioning encoderPositioning;


    /** Calls all appropriate sensor update methods to get an updated estimate of the Robot's current position
     * @param robot Robot object whose estimate should be updated
     */
    public void updatePosition(Robot robot) {
        updateEncoderPosition(encoderPositioning.getDeltaEstimate(robot));
    }


    /** Adds new detected encoder movement change to both a temporary encoderDelta variable and to the overall position attribute
     *
     *  @param delta A delta position represented as a vector from the last seen position.
     *               e.g. delta = Position(1, 1, 0) would mean a movement of 1 inch on all axis with no rotation
     */
    private void updateEncoderPosition(Position subDelta) {
        position = Position.add(position, subDelta);
        encoderDelta = Position.add(encoderDelta, subDelta);
    }


    /** To be called from the CV positioning Pipeline; incorporates a new cv estimate into the position using the encoder deltas
     * @param newPos The CV estimate
     */
    public void updateCvPosition(Position newPos) {
        Position compounded = Position.add(newPos, encoderDelta);

        // NOTE: combine compounded with current position (compounded shouldn't be an unnecessary local)
        position = compounded;
        encoderDelta.reset();
    }

    private Position encoderDelta;
}
