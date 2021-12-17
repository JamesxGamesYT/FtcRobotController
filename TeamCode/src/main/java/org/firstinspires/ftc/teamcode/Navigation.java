/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.jetbrains.annotations.NotNull;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation
{
    // DUCK: deliver duck from carousel.
    // FREIGHT: deliver one piece of freight from the warehouse to the shipping hub.
    public enum NavigationMode {DUCK, FREIGHT, TELEOP}
    public enum AllianceColor {BLUE, RED}

    final double POWER = 0.75;
    final double ROTATION_POWER = 0.75;  // Power to use while rotating.
    final double MIN_POWER = 0.1;  // Power to use at start/end of ramp up/down.
    // Rate at which to ramp up/down in terms of power units over radians.
    // Rationale: max power should be reached when we're pi/3 radians away from starting rotation, and power should
    // begin decreasing when we're pi/3 radians away from target rotation.
    final double RAMP_SLOPE_ROTATION = POWER / (Math.PI / 3);
    // Follows similar logic to RAMP_SLOPE_STRAFING. Note that the denominator is in inches.
    final double RAMP_SLOPE_STRAFING = POWER / 5;
    // Accepted amounts of deviation between the robot's desired position and actual position.
    final double EPSILON_LOC = 0.1;
    final double EPSILON_ANGLE = 0.1;

    public enum RotationDirection {CLOCKWISE, COUNTERCLOCKWISE}

    // First position in this ArrayList is the first position that robot is planning to go to.
    // This condition must be maintained (positions should be deleted as the robot travels)
    // NOTE: a position is both a location and a rotation.
    // NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
    private ArrayList<Position> path;

    public Navigation(NavigationMode navMode, AllianceColor allianceColor) {
        if (navMode == NavigationMode.TELEOP) {
            path = new ArrayList<>(Collections.emptyList());
        }
        else if (navMode == NavigationMode.DUCK) {
            path = AutonomousPaths.DUCK_PATH;
        }
        else if (navMode == NavigationMode.FREIGHT) {
            path = AutonomousPaths.FREIGHT_PATH;
        }

        // NOTE: This may actually have to be the other way around. It depends on which side we do our measurements for.
        if (allianceColor == AllianceColor.RED) {
            reflectPath();
        }
    }

    /** Adds a desired position to the path.
     */
    public void addPosition(Position pos) {
        path.add(pos);
    }

    /** Adds a desired position to the path at a specific index.
     */
    public void addPosition(Position pos, int index) {
        path.add(index, pos);
    }

    /** Makes the robot travel along the path until it reaches a POI.
     */
    public void travelToNextPOI(Robot robot) {
        while (true) {
            Position target = path.get(0);
            // TODO: update robot position
            Position start = robot.positionManager.position;
            rotate(start.rotation - target.rotation, robot);
            travelLinear(target, robot);
            path.remove(0);
            if (target.location.name.substring(0, 3).equals("POI")) break;
        }
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     *  TODO: make this use JoystickValues
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX, Robot robot) {
        // Uses left stick to go forward, and right stick to turn.
        // NOTE: right-side drivetrain motor inputs don't have to be negated because their directions will be reversed
        //       upon initialization.

        double moveDirection, power, turn, sinMoveDirection, cosMoveDirection, frontLeftPower, frontRightPower,
                rearLeftPower, rearRightPower;

        leftStickY = -leftStickY;  // Y coordinate is reversed.
        turn = rightStickX;
        if (-0.05 < turn && turn < 0.05) {  // joystick dead zone
            turn = 0;
        }
        turn /= 2.0;  // Scale input sensitivity.

        moveDirection = Math.atan2(leftStickY, leftStickX);

        power = Range.clip(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2)), 0, 1);
        if (power <= 0.05) { // joystick dead zone
            power = 0;
        }

        sinMoveDirection = Math.sin(moveDirection);
        cosMoveDirection = Math.cos(moveDirection);

        // Set the power for each wheel based on the angle of the stick and how far the stick is from center
        frontLeftPower = Range.clip(sinMoveDirection + cosMoveDirection, -1, 1) * power + turn;
        frontRightPower = Range.clip(sinMoveDirection - cosMoveDirection, -1, 1) * power - turn;
        rearLeftPower = Range.clip(sinMoveDirection - cosMoveDirection, -1, 1) * power + turn;
        rearRightPower = Range.clip(sinMoveDirection + cosMoveDirection, -1, 1) * power - turn;

        robot.telemetry.addData("Left Stick Position", Math.toDegrees(moveDirection) + " degrees");
        robot.telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        robot.telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)", rearLeftPower, rearRightPower);
    }

    /** Rotates the robot a number of degrees.
     *
     * @param angle The amount for the robot to rotate (radians, positive for clockwise, negative for CC).
     *              Within interval [-pi, pi]
     */
    private void rotate(double angle, Robot robot) 
    {
        // TODO: update robot position

        // Both values are restricted to interval [0, 2pi] for simplified comparisons.
        double startingRotation = robot.positionManager.position.rotation;
        double currentRotation = startingRotation;
        if (startingRotation < 0.0) {
            startingRotation = Math.PI + Math.abs(startingRotation);
        }
        double targetRotation = (startingRotation + angle) % (2 * Math.PI);

        // Ramping algorithm:
        // - Check whether to ramp up or down based on whether you are halfway to target
        // - Set power proportional to distance to target when ramping down, inversely proportional when ramping up
        // - Clip value between max/min powers

        boolean rampUp = true;
        double power = MIN_POWER;
        RotationDirection direction = (angle > 0) ? RotationDirection.CLOCKWISE : RotationDirection.COUNTERCLOCKWISE;

        rotate(direction, power, robot);

        // While position is not reached.
        while (Math.abs(robot.positionManager.position.rotation - targetRotation) >= EPSILON_ANGLE)
        {
            // TODO: update robot position
            if (currentRotation < 0.0) {
                currentRotation = Math.PI + Math.abs(currentRotation);
            }
            if (rampUp) {
                switch (direction) {
                    case CLOCKWISE:
                        // As currentRotation decreases (along unit circle), power should increase.
                        power = (startingRotation - currentRotation) * RAMP_SLOPE_ROTATION;
                    case COUNTERCLOCKWISE:
                        // As currentRotation increases, power should increase.
                        power = (currentRotation - startingRotation) * RAMP_SLOPE_ROTATION;
                }
                rotate(direction, power, robot);
                // Check whether to start ramping down (if we're at least halfway there).
                rampUp = Math.abs(startingRotation - currentRotation) >= angle / 2;
            }
            else
            {
                switch (direction) {
                    case CLOCKWISE:
                        // As currentRotation decreases, power should decrease.
                        power = (currentRotation - targetRotation) * RAMP_SLOPE_ROTATION;
                    case COUNTERCLOCKWISE:
                        // As currentRotation increases, power should decrease.
                        power = (targetRotation - currentRotation) * RAMP_SLOPE_ROTATION;
                }
                rotate(direction, power, robot);
            }
        }

        robot.rearLeftDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
    }

    /** Sets motor powers to rotate the robot in a certain direction.
     */
    private void rotate(RotationDirection direction, double power, Robot robot) {
        switch (direction) {
            case CLOCKWISE:
                // Right wheels go forward and left ones go backward.
                robot.rearLeftDrive.setPower(-power);
                robot.frontLeftDrive.setPower(-power);
                robot.rearRightDrive.setPower(power);
                robot.frontRightDrive.setPower(power);
            case COUNTERCLOCKWISE:
                // Right wheels go backward and left ones go forward.
                robot.rearLeftDrive.setPower(power);
                robot.frontLeftDrive.setPower(power);
                robot.rearRightDrive.setPower(-power);
                robot.frontRightDrive.setPower(-power);
        }
    }

    /** Makes the robot travel in a straight line for a certain distance.
     *
     *  @param desiredPosition The desired position of the robot.
     */
    private void travelLinear(Position desiredPosition, Robot robot) {
        double frontLeftPower = 0; double frontRightPower = 0; double rearLeftPower = 0; double rearRightPower = 0;
        Point origPoint = robot.position.location;

        double moveDirection, power, turn, sinMoveDirection, cosMoveDirection;

        double desiredX = desiredPosition.location.x;
        double desiredY = -desiredPosition.location.y;  // Y coordinate is reversed.

        /*
        turn = rightStickX;
        if (-0.05 < turn && turn < 0.05) {  // joystick dead zone
            turn = 0;
        }
        turn /= 2.0;  // Scale input sensitivity.
         */

        final double POWERSLOPE = 0.05;

        // TODO: Replace this with a call to getAngleBetween
        moveDirection = Math.atan2(desiredY-origPoint.y, desiredX-origPos.x);

        power = Range.clip(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2)),0,1);
        if (power <= 0.05) { // joystick dead zone
            power = 0;
        }

        sinMoveDirection = Math.sin(moveDirection);
        cosMoveDirection = Math.cos(moveDirection);

        frontLeftPower = Range.clip(sinMoveDirection + cosMoveDirection, -1, 1) * power + turn;
        frontRightPower = Range.clip(sinMoveDirection - cosMoveDirection, -1, 1) * power - turn;
        rearLeftPower = Range.clip(sinMoveDirection - cosMoveDirection, -1, 1) * power + turn;
        rearRightPower = Range.clip(sinMoveDirection + cosMoveDirection, -1, 1) * power - turn;

        // Set the power for each wheel based on the angle of the stick and how far the stick is from center
        while (Math.abs(desiredX-robot.position.location.x) <= EPSILON_LOC && Math.abs(desiredY-robot.position.location.y) <= EPSILON_LOC) {
            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.frontRightDrive.setPower(frontRightPower);
            robot.rearLeftDrive.setPower(rearLeftPower);
            robot.rearRightDrive.setPower(rearRightPower);
        }

        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

        /*
        robot.telemetry.addData("Left Stick Position",Math.toDegrees(moveDirection) + " degrees");
        robot.telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        robot.telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)", rearLeftPower, rearRightPower);
         */
        }
    }

    /** Determines the angle between the horizontal axis and the segment connecting A and B.
     */
    private double getAngleBetween(Point a, Point b) { return Math.atan2((b.y-a.y), (b.x-a.x)); }

    /** Calculates the euclidean distance between two points.
     *
     *  @param a A 2D point on the playing field.
     *  @param b The point to find the distance to point A from.
     *  @return The Euclidean distance between the two points.
     */
    private double getEuclideanDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    /** Reflects the path to the other side of the playing field.
     */
    private void reflectPath() {}

    // PATHFINDING
    // ===========

    //variables relating to the operations of pathfinding
    int hubexclousionRadious=11;
    double segmentDist=1;
    /**determine if the provided point is inside a hub
     *
     * @param p the point to checked
     * @return boolean, true if the provided point is inside a hub
     */
    boolean insideHub(Point p){
        if(Math.sqrt(Math.pow(p.x-72,2)+Math.pow(p.y-120,2))<=hubexclousionRadious){
            return true;
        }
        if(Math.sqrt(Math.pow(p.x-48,2)+Math.pow(p.y-60,2))<=hubexclousionRadious){
            return true;
        }
        if(Math.sqrt(Math.pow(p.x-96,2)+Math.pow(p.y-60,2))<=hubexclousionRadious){
            return true;
        }
        return false;
    }

    /**determine if the provided point is inside the horizontal barrier
     *
     * @param p the point to checked
     * @return boolean, true if the provided point is inside the barrier
     */
    boolean insideBarrierH(Point p){
        //13.68 99.5 116.32 5.77
        if(p.x>=13.68&&p.x<=13.68+116.32&&p.y>=99.5-5.77&&p.y<=99.5){
            return true;
        }
        return false;
    }

    /**determines if the provided point is inside the left vertical barrier
     *
     * @param p the point to checked
     * @return boolean, true if the provided point is inside the barrier
     */
    boolean insideBarrierVL(Point p){
        //13.68 99.5 116.32 5.77
        if(p.x>=44.6&&p.x<=44.6+5.77&&p.y>=130.2-30.75&&p.y<=130.2){
            return true;
        }
        return false;
    }

    /**determines if the provided point is inside the right vertical barrier
     *
     * @param p the point to checked
     * @return boolean, true if the provided point is inside the barrier
     */
    boolean insideBarrierVR(Point p){
        //13.68 99.5 116.32 5.77
        if(p.x>=93.75&&p.x<=93.75+5.77&&p.y>=130.2-30.75&&p.y<=130.2){
            return true;
        }
        return false;
    }

    /**generates a path as an array list of points that goes from start to end without running into any obstacles
     *it is recommended that you run the output of this function through optimisePath1 and then optimisePath2
     * @link https://github.com/jSdCool/FTC-robot-pathfinging for a visual deminstration
     * @param start the point to start the path at (usually the robots current position)
     * @param end the point to end the path at
     * @return an arraylist of points that form a path between the provided point
     */
    ArrayList<Position> createPath(Position start,Position end){
        ArrayList<Position> p=new ArrayList<Position>();
        p.add(start);
        boolean working=true;
        int itteration=0;
        double anglein=start.rotation;
        while(working){
            double angle=Math.atan2((end.location.y-p.get(p.size()-1).location.y),(end.location.x-p.get(p.size()-1).location.x));//find the absolute angle to the next point in a straight line to the end point
            Point work;
            do{

                work =new Point(Math.cos(angle)*segmentDist+p.get(p.size()-1).location.x,Math.sin(angle)*segmentDist+p.get(p.size()-1).location.y,"");//create the next point
                angle += 0.01;//add 0.01 radians to the theoretical angle

            }while(insideHub(work));//if the created point was inside a hub then calculate the point again with the new agale and check again
            if(insideBarrierH(work)){//if the  calculated point is inside the horizontal barrier
                ArrayList<Position> temp;//create a temporary array list of points

                if(angle>0){//if the robot is heading up ish
                    if(work.x>72){//if it is on the right side of the field
                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,92,""),Math.PI));//crate a path that goes to a predefined point at the side of the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,92,""),0));//crate a path that goes to a predefined point at the side of the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge generated path into the current working path
                    }
                    if(work.x>72){//if it is on the right side of the field
                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,104,""),Math.PI));//make a path going past the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,104,""),0));//make a path going past the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge path into main path
                    }
                }else{//if the robot is heading down ish
                    if(work.x>72){//if it is on the right
                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,104,""),Math.PI));//crate a path that goes to a predefined point at the side of the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,104,""),0));//crate a path that goes to a predefined point at the side of the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    if(work.x>72){//if on the right side of the field
                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,92,""),Math.PI));//make a path going past the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,92,""),0));//make a path going past the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }
            }else if (insideBarrierVL(work)){//path around the left vertical barrier
                ArrayList<Position> temp;
                if(angle<Math.PI/2&&angle>-Math.PI/2){//if it is heading right
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(42,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(54,137,""),-Math.PI/2));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }else{//if the robot in heading left
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(54,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(42,137,""),-Math.PI/2));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }

            }else if (insideBarrierVR(work)){//path around the right vertical barrier
                ArrayList<Position> temp;
                if(angle>Math.PI/2||angle<-Math.PI/2){//if the robot is heading left
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(101,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(91,137,""),-Math.PI/2));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }else{
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(91,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(101,137,""),-Math.PI/2));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }
            }else{
                p.add(new Position(work,anglein));//add the current working point to the path
            }
            if(Math.sqrt(Math.pow(end.location.x-p.get(p.size()-1).location.x,2)+Math.pow(end.location.y-p.get(p.size()-1).location.y,2))<segmentDist)//if the point is less than the distance of the segments from the end point
                working=false;//tell the loop to stop


            itteration++;//increase the iteration
            if(itteration>1000){//if the program is stuck in an infinite loop(too many iterations)
                return null;
            }
        }
        p.add(end);//add the final point to the path
        return p;
    }

    /**the first step in optimising a path, this function reduces the numbers of point in a path by detecting straight lines and removing the points that make them up
     @param p the path that you want to optimise
     @return a path that contains fewer points
     */
    ArrayList<Position> optimisePath1(ArrayList<Position> p){
        ArrayList<Position> o=new ArrayList<Position>();//the object to return
        o.add(p.get(0));//add the first point of the path to the new path
        int beginindex=0;
        double devation=0.01;//how far(in radians) is a line allowed to lean in either direction before it is consisted a new line
        double angle=Math.atan2(p.get(1).location.y-p.get(0).location.y,p.get(1).location.x-p.get(0).location.x);//calculate the initial angle that the line is going in
        for(int i=1;i<p.size();i++){
            double newAngle=Math.atan2(p.get(i).location.y-p.get(beginindex).location.y,p.get(i).location.x-p.get(beginindex).location.x);//calculate the angle between the base point of the current line and the next point in the list
            if(newAngle>=angle-devation&&newAngle<=angle+devation){//if the angle is inside the acceptable range
                continue;
            }else{
                o.add(p.get(i-1));//add the previous point to the optimised path
                beginindex=i;//set the current point as the new base point
                angle=Math.atan2(p.get(i).location.y-p.get(i-1).location.y,p.get(i).location.x-p.get(i-1).location.x);//calculate the new angle of the next line
            }
        }
        o.add(p.get(p.size()-1));//add the  final point to the new path
        return o;
    }

    /**the second step in optimizing paths this function generates paths between point in a given path to see if it can find a shorter path between them
     @param path the path that you want to optimise that has been run through optimisePath1
     @return a path that has a shorter overall travel
     */
    ArrayList<Position> optimisePath2(ArrayList<Position> path){
        ArrayList<Position> p=new ArrayList(path);//copy the input path

        if(p.size()==2){//if the path only consists of 2 points then the path can not be optimised so do nothing
            return p;
        }

        ArrayList<Position> o=new ArrayList<Position>();//the object to return

        for(int i=0;i<p.size()-1;){//seek through the path
            int curbest=i+1,sigbest=-1;
            for(int j=i+1;j<p.size();j++){//check every point in the path ahead of this point
                double l1,l2;
                ArrayList<Position> temp=new ArrayList<Position>(),temp2;//create temporary paths
                for(int n=i;n<=j;n++){//make the temp path the section of the main path between the 2 points
                    temp.add(p.get(n));
                }
                temp2=optimisePath1(createPath(p.get(i),p.get(j)));//generate a new path directly between the 2 selected points
                l1=pathlength(temp2);
                l2=pathlength(temp);
                if(l1<l2){//compare the lengths of the paths, if the new path is less than the original
                    curbest=j;//set the current best index to j
                    if(sigbest==-1){//if the best significant is -1 then set it to the current best
                        sigbest=curbest;
                    }
                }

                if(l1<=l2*0.7){//if this path is significantly shorter than the old best then set sigbest to this path      this value may need to be tweaked
                    sigbest=j;
                }

            }//end of loop
            if(sigbest==-1){//if the best significant is -1 then set it to the current best
                sigbest=curbest;
            }
            ArrayList<Position> temp=new ArrayList<Position>();//create a temp path
            temp=optimisePath1(createPath(p.get(i),p.get(sigbest)));//set the temp path to the new best path
            for(int j=0;j<temp.size();j++){
                o.add(temp.get(j));//add the new best path to the output
            }
            i=sigbest;
        }

        return o;
    }

    /**gets the total travel distance of a path
    @param p the path you want the length of
    @return the length of the path
    */
    double pathlength(ArrayList<Position> p){
        double length=0;
        for(int i=0;i<p.size()-1;i++){
            length+=Math.sqrt(Math.pow(p.get(i+1).location.y-p.get(i).location.y,2)+Math.pow(p.get(i+1).location.x-p.get(i).location.x,2));
        }
        return length;
    }
}


/** Hardcoded paths through the playing field during the Autonomous period.
 */
class AutonomousPaths {
    public static final ArrayList<Position> DUCK_PATH = new ArrayList<>(Arrays.asList(
            // Construct Position objects
    ));
    public static final ArrayList<Position> FREIGHT_PATH = new ArrayList<>(Arrays.asList());
}
