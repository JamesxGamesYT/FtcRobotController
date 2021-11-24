/*
* Authors: ningning ying, elicia 0
* https://code-with-me.jetbrains.com/XKKH3G59srClhs2J-yw3zQ#p=IC&fp=11E83DBF4852D367814E3D348B3543B60C28995641E977BD9AC2E26485A08445
* */

import java.util.ArrayList;
import Point;


/** Keeps track of robot's desired path and makes it follow it accurately.
 */
public class Navigation
{
    // First point in this arraylist is the first point that robot is planning to go to.
    // This condition must be maintained (points should be deleted as the robot travels)
    private ArrayList<Point> path = new ArrayList<Point>();
    private double speed;

    /** Adds a desired point to the path.
     */
    public void addDestination(Point loc) {}

    /** Adds a desired point to the path at a specific index.
     */
    public void addDestination(Point loc, int index) {}

    /** Travel a certain number of points along the path.
     *  @param numPoints The number of points along the path to travel.
     */
    public void travel(int numPoints) {}

    /** Moves the robot in a straight line for a certain distance.
     *  @param dist The distance the robot should travel.
     *  @param relativeAngle The relative angle from the current position of the robot
     */
    private void moveLinear(double dist, double relativeAngle) {};

    /** Determine the angle relative to the angle the robot is facing that it needs to go in in order to get to the
     *  next point. For example, this should be used to determine a value to be passed into moveLinear as relativeAngle.
     */
     private double getAngleToNextPoint() {}
}





