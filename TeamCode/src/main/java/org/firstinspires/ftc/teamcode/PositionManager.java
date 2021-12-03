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
    public void setXcor(double x){
        xcor = x;
    }
    public void setYcor(double y){
        ycor = y;
    }
    public void setRotation(double r){
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

    /** Adds new detected encoder movement change to both a temporary encoderDelta variable and to the overall position attribute
     *
     *  @param delta A delta position represented as a vector from the last seen position.
     *               e.g. delta = Position(1, 1, 0) would mean a movement of 1 inch on all axis with no rotation
     */
    public void updateEncoderPosition(Position delta) {}

    /** Sets the encoder delta back to (0, 0, 0).
     *  To be called when an image is received, so that the delta stores the movement done during image processing
     */
    public void resetEncoderDelta() {}


    private Position encoderDelta;
    private Position cvEstimate;
>>>>>>> origin/positioning
}
