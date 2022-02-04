package org.firstinspires.ftc.teamcode;
/*
class created by Stephen Duffy
do not edit
*/


//a simple class that stores an x and Y value
public class Point {
    public enum Action {PRELOAD_BOX, CAROUSEL, NONE}
    protected double x;
    protected double y;
    protected Action action;
    // Prefix with POI if it's a point of interest (in which case it is a stopping point in Auton).
    protected String name;
    Point(double X,double Y,String Name){
        x=X;
        y=Y;
        name=Name;
        this.action = Action.NONE;
    }
    Point(double X,double Y,String Name, Action action){
        x=X;
        y=Y;
        name=Name;
        this.action = action;
    }
    void setX(double X){
        x=X;
    }
    void setY(double Y){
        y=Y;
    }
}
