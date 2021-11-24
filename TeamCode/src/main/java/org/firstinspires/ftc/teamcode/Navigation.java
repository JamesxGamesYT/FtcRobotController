/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin
 */

package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.CVPositioning;
import org.firstinspires.ftc.teamcode.EncoderPositioning;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation
{
    final double SPEED = 1.0;

    // First point in this ArrayList is the first point that robot is planning to go to.
    // This condition must be maintained (points should be deleted as the robot travels)
    private ArrayList<Point> path = new ArrayList<>();

    private DcMotor frontRight, rearRight, frontLeft, rearLeft;
    private CVPositioning cvPositioning;
    private EncoderPositioning encoderPositioning;

    public Navigation(DcMotor frontRight, DcMotor rearRight, DcMotor frontLeft, DcMotor rearLeft,
                      CVPositioning cvPositioning, EncoderPositioning encoderPositioning) {}

    /** Adds a desired point to the path.
     */
    public void addDestination(Point loc) {}

    /** Adds a desired point to the path at a specific index.
     */
    public void addDestination(Point loc, int index) {}

    /** Makes the robot travel a certain number of points along the path.
     *  @param numPoints The number of points along the path to travel.
     */
    public void travel(int numPoints) {}

    /** Makes the robot travel in a straight line for a certain distance.
     *  @param dist The distance the robot should travel.
     *  @param relativeAngle The relative angle from the current position of the robot
     */
    private void travelLinear(double dist, double relativeAngle) {}

    /** Determines the amount of rotation required for the robot to face its next destination. For example, this should
     *  be used to determine a value to be passed into moveLinear as relativeAngle.
     */
    private double getAngleToNextPoint() { return 0.0; }
}
