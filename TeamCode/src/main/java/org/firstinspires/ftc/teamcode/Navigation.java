/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin
 */

package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation
{
    final double SPEED = 1.0;
    // Accepted amounts of deviation between the robot's desired position and actual position.
    final double EPSILON_LOC = 0.1;
    final double EPSILON_ANGLE = 0.1;

    // First position in this ArrayList is the first position that robot is planning to go to.
    // This condition must be maintained (positions should be deleted as the robot travels)
    // NOTE: a position is both a location and a rotation.
    // NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
    // TODO: implement a system to keep track of which positions in this attribute are POIs.
    private ArrayList<Position> path = new ArrayList<>();

    public Navigation(ArrayList<Position> path) {}

    /** Adds a desired position to the path.
     */
    public void addPosition(Position loc) {}

    /** Adds a desired position to the path at a specific index.
     */
    public void addPosition(Position loc, int index) {}

    /** Makes the robot travel along the path until it reaches a POI.
     */
    public void travelToNextPOI(Robot robot) {
        // Repeat until at POI:
        // - realPos = <get position from robot>
        // - while realPos and path[0] are not within epsilon values:
        //     - angle = getAngleBetween(currentPosition, path[0])
        //     - dist = getEuclideanDistance(currentPosition, path[0])
        //     - travelLinear(angle, dist)
        //     - rotate(desiredRotation - currentRotation)
        //     - update realPos
        // - remove path[0] from path
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX, double rightStickY, Robot robot) {
        robot.rearRightDrive.setPower(rightStickY / 3);
    }

    /** Rotates the robot a number of degrees.
     */
    private void rotate(double angle, Robot robot) {}

    /** Makes the robot travel in a straight line for a certain distance.
     *  @param dist The distance the robot should travel.
     *  @param angle The angle in which the robot will strafe.
     */
    private void travelLinear(double dist, double angle, Robot robot) {}

    /** Determines the angle between the horizontal axis and the segment connecting A and B.
     */
    private double getAngleBetween(Point a, Point b) { return 0.0; }

    /** Calculates the euclidean distance between two points.
     */
    private double getEuclideanDistance(Point a, Point b) { return 0.0; }
}
