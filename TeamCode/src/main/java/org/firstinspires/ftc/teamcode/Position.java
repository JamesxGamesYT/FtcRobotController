package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */
public class Position {

    /** Stores an x/y coordinate.
     *  @see Point for more information
     */
    public Point location;
    public double rotation;

    public boolean equals(Position a) {
        return a.location.x == this.location.x && a.location.y == this.location.y;
    }
}
