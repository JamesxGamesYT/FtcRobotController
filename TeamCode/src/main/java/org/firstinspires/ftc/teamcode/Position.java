package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */
public class Position {
    Position() {
        setX(0.0);
        setY(0.0);
        setRotation(0.0);
    }

    Position(double x, double y, double r) {
        setX(x);
        setY(y);
        setRotation(r);
    }

    public void setX(double x) {
        location.setX(x);
    }

    public void setY(double y) {
        location.setY(y);
    }

    public void setRotation(double r) {
        this.rotation = r;
    }

    public double getX() {
        return location.x;
    }

    public double getY() {
        return location.y;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean equals(Position a) {
        return a.location.x == this.location.x && a.location.y == this.location.y;
    }

    /** Stores an x/y coordinate.
     *  @see Point for more information
     */
    public Point location;
    public double rotation;
}
