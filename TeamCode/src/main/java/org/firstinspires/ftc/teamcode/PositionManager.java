<<<<<<< HEAD
package org.firstinspires.ftc.teamcode;

public class PositionManager {
    private static double xcor;
    private static double ycor;
    private static double rotation;
    PositionManager(){
        xcor = 0;
        ycor = 0;
        rotation = 0;
    }
    PositionManager(double x, double y, double r){
        xcor = x;
        ycor = y;
        rotation = r;
    }
    public static double getXcor(){
        return(xcor);
    }
    public static double getYcor(){
        return(ycor);
    }
    public static double getRotation(){
        return(rotation);
    }
    public static void setXcor(double x){
        xcor = x;
    }
    public static double setYcor(double y){
        ycor = y;
    }
    public static double setRotation(double r){
        rotation = r;
    }
=======
/* Authors: Arin Khare, Kai Vernooy
 */


package org.firstinspires.ftc.teamcode;

/** Incorporates estimates from two sources (CV positioning and encoders) to create a single positioning estimate
 */
public class PositionManager {
    public Position position;

    private Position cvEstimate;
>>>>>>> origin/positioning
}
