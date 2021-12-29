package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */
public class Position {
    Position() {
        location = new Point(0.0, 0.0, "");
        setRotation(0.0);
    }

    Position(double x, double y, double theta) {
        location = new Point(x, y, "");
        setRotation(theta);
    }

    public void setX(double x) {
        location.setX(x);
    }

    public void setY(double y) {
        location.setY(y);
    }

    public void setRotation(double theta) {
        this.rotation = theta;
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

    public void reset() {
        setX(0.0);
        setY(0.0);
        setRotation(0.0);
    }

    public static Position add(Position a, Position b) {
        return new Position(a.getX() + b.getX(), a.getY() + b.getY(), (a.getRotation() + b.getRotation()) % (2 * Math.PI));
    }

    /** Stores an x/y coordinate.
     *  @see Point for more information
     */
    private Point location;

    /** A rotation, in radians, in the interval [0, 2pi)
     */
    private double rotation;
}
