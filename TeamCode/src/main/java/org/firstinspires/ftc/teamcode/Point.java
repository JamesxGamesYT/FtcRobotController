package org.firstinspires.ftc.teamcode;
/*
class created by Stephen Duffy
do not edit
*/


//a simple class that stores an x and Y value
public class Point {
    protected double x;
    protected double y;
    protected String name;
    Point(double X,double Y,String Name){
        x=X;
        y=Y;
        name=Name;
    }
    void setX(double X){
        x=X;
    }
    void setY(double Y){
        y=Y;
    }
}
