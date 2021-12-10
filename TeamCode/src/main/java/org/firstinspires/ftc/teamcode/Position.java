package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 *
 *  TODO: currently there are both getter/setter method as well as public attributes. If we decide to keep the
 *        getters/setters we should make the attributes private, and vice versa.
 */
public class Position {
    Position() {
        location = new Point(0.0,0.0, "");
        setRotation(0.0);
    }

    Position(double x, double y, double r) {
        location = new Point(x, y, "");
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
