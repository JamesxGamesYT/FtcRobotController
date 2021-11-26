package org.firstinspires.ftc.teamcode;


/** Estimates the robot's position based on the encoders on the drivetrain's motors. This will require a baseline
 *  position to add onto.
 */
public class EncoderPositioning {
    /** Checks the encoder's DC motors
     *  @return a position in the form of a vector from the origin that can be added to an existing measurement
     */
    private Position updateDeltaEstimate(Robot robot) {return null;}


    /** Takes an existing position, either as a param or member (based on existence of robot class)
     *  @return an absolute position that combines the delta and the reference position
     */
    public Position getPositionEstimate(Position position) {return null;}


    /** Updates the encoder's position estimate in the robot's PositionManager
     */
    public void submitEstimate(Robot robot) {}
}