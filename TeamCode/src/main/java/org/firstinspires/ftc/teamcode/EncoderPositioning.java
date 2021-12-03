package org.firstinspires.ftc.teamcode;


/** Estimates the robot's position based on the encoders on the drivetrain's motors. This will require a baseline
 *  position to add onto.
 */
public class EncoderPositioning {
    /** Checks the robot's encoders to get an estimate of the distance and direction traveled.
     *  @return a position in the form of a vector from the origin that can be added to an existing measurement
     */
    public Position updateDeltaEstimate(Robot robot) {return null;}


    /** Updates the encoder's position estimate in the robot's PositionManager
     */
    private void submitEstimate(Robot robot, Position delta) {
        robot.position.updateEncoderPosition(delta);
    }
}