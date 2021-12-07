/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin
 */

package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;
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
    final double SPEED = 1.0;
    // Accepted amounts of deviation between the robot's desired position and actual position.
    final double EPSILON_LOC = 0.1;
    final double EPSILON_ANGLE = 0.1;

    // First position in this ArrayList is the first position that robot is planning to go to.
    // This condition must be maintained (positions should be deleted as the robot travels)
    // NOTE: a position is both a location and a rotation.
    // NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
    // TODO: implement a system to keep track of which positions in this attribute are POIs.
    private ArrayList<Position> path;

    public Navigation(ArrayList<Position> path) {
        this.path = path;
    }

    /** Adds a desired position to the path.
     */
    public void addPosition(Position pos) {
        path.add(pos);
    }

    /** Adds a desired position to the path at a specific index.
     */
    public void addPosition(Position pos, int index) {
        path.add(pos, index);
    }

    /** Makes the robot travel along the path until it reaches a POI.
     */
    public void travelToNextPOI(TravelDirection travelDirection, Robot robot) {
        double angle;
        switch(travelDirection) {
            case FORWARD:
                angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-robot.position.rotation;
                break;
            case REVERSE:
                angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-(robot.position.rotation+Math.PI);
                break;
            case LEFT:
                angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-(robot.position.rotation-Math.PI/2);
                break;
            case RIGHT:
                angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-(robot.position.rotation+Math.PI/2);
                break;
        }

        rotate(angle, robot);
        travelLinear(path.get(0), travelDirection, 1, robot);
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX, double rightStickY, Robot robot) {
        double g1StickLY = -leftStickY;
        double g1StickLX = leftStickX;

        double g1StickLDirection=Math.atan2(g1StickLY,g1StickLX);//get the angle the left stick is at

        double generalPower=Range.clip(Math.sqrt(Math.pow(g1StickLX,2)+Math.pow(g1StickLY,2)),0,1);
        if(generalPower<=0.05){//joystick dead zone
            generalPower=0;
        }

        //set the power for each wheel absed on the angle of the stick and how far the stick is from center
        double frontLeftPower = Range.clip(Math.sin(g1StickLDirection)+Math.cos(g1StickLDirection),-1,1)*generalPower;
        double frontRightPower = Range.clip(Math.sin(g1StickLDirection)-Math.cos(g1StickLDirection),-1,1)*generalPower;
        double backLeftPower = Range.clip(Math.sin(g1StickLDirection)-Math.cos(g1StickLDirection),-1,1)*generalPower;
        double backRightPower = Range.clip(Math.sin(g1StickLDirection)+Math.cos(g1StickLDirection),-1,1)*generalPower;
    }

    /** Rotates the robot a number of degrees.
     */
    private void rotate(double angle, Robot robot) 
    {
        // Assign the original rotation to a variable
        robot.position.position.rotation;

        double rotationPowerPositive = angle/360;
        double rotationPowerNegative = -1*angle/360;


        if(angle > 0)
        {
            //going backwards
            robot.rearLeftDrive.setPower(rotationPowerNegative);
            robot.frontLeftDrive.setPower(rotationPowerNegative);
            //going forwards
            robot.rearRightDrive.setPower(rotationPowerPositive);
            robot.frontRightDrive.setPower(rotationPowerPositive);

        }
        else
        {
            //going forwards
            robot.rearLeftDrive.setPower(rotationPowerPositive);
            robot.frontLeftDrive.setPower(rotationPowerPositive);
            //going backwards
            robot.rearRightDrive.setPower(rotationPowerNegative);
            robot.frontRightDrive.setPower(rotationPowerNegative);
        }


        double rotationPosition = 0;
        double finalRotation = 0;

        //checking/reading position (is incomplete)
        while (Math.abs(robot.position.position.rotation- finalRotation) >= 0.1)//rotation is not done
        {
            //reads position
            double nPosition = robot.position.position.rotation;
            // TODO: update robot position and compare // ////  TimeUnit.MILLISECONDS.sleep(30);}



    }

    /** Makes the robot travel in a straight line for a certain distance.
     *  @param desiredPosition The desired position of the robot.
     *  @param angle The angle in which the robot will strafe.
     *  @param travelDirection The direction in which the robot will be traveling once it has turned to the desired angle.
     */
    private void travelLinear(Position desiredPosition, @NotNull TravelDirection travelDirection, double generalPower, Robot robot) throws InterruptedException {
        double frontLeftPower = 0; double frontRightPower = 0; double rearLeftPower = 0; double rearRightPower = 0;
        Point origPoint = robot.position.location;

        switch(travelDirection) {
            case FORWARD:
                frontLeftPower = generalPower;
                frontRightPower = generalPower;
                rearLeftPower = generalPower;
                rearRightPower = generalPower;
                break;
            case REVERSE:
                frontLeftPower = -generalPower;
                frontRightPower = -generalPower;
                rearLeftPower = -generalPower;
                rearRightPower = -generalPower;
                break;
            case LEFT:
                frontLeftPower = -generalPower;
                frontRightPower = generalPower;
                rearLeftPower = generalPower;
                rearRightPower = -generalPower;
                break;
            case RIGHT:
                frontLeftPower = generalPower;
                frontRightPower = -generalPower;
                rearLeftPower = -generalPower;
                rearRightPower = generalPower;
                break;
        }

        robot.frontLeftDrive.setPower(frontLeftPower);
        robot.frontRightDrive.setPower(frontRightPower);
        robot.rearLeftDrive.setPower(rearLeftPower);
        robot.rearRightDrive.setPower(rearRightPower);

        if (robot.position.location.x == desiredPosition.location.x && robot.position.location.y == desiredPosition.location.y) {
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.rearLeftDrive.setPower(0);
            robot.rearRightDrive.setPower(0);
        } else if (Math.atan2(desiredPosition.y-robot.position.location.y, desiredPosition.x-robot.position.location.x) == Math.atan2(desiredPosition.y-origPoint.y, desiredPosition.x-origPoint.x)) {
            TimeUnit.MILLISECONDS.sleep(30);
        } else {
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.rearLeftDrive.setPower(0);
            robot.rearRightDrive.setPower(0);

            double angle;
            switch(travelDirection) {
                case FORWARD:
                    angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-robot.position.rotation;
                    break;
                case REVERSE:
                    angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-(robot.position.rotation+Math.PI);
                    break;
                case LEFT:
                    angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-(robot.position.rotation-Math.PI/2);
                    break;
                case RIGHT:
                    angle = Math.atan2(path.get(0).location.y-robot.position.location.y, path.get(0).location.x-robot.position.location.x)-(robot.position.rotation+Math.PI/2);
                    break;
            }

            rotate(angle, robot);

            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.frontRightDrive.setPower(frontRightPower);
            robot.rearLeftDrive.setPower(rearLeftPower);
            robot.rearRightDrive.setPower(rearRightPower);
        }
    }

    /** Determines the angle between the horizontal axis and the segment connecting A and B.
     */
    private double getAngleBetween(Point a, Point b) { return Math.atan((b.y-a.y)/(b.x-a.x)); }

    /** Calculates the euclidean distance between two points.
     *
     *  @param a A 2D point on the playing field.
     *  @param b The point to find the distance to point A from.
     *  @return The Euclidean distance between the two points.
     */
    private double getEuclideanDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    // PATHFINDING
    // ===========

    //varibles relating to the opperations of pathfinding
    int hubexclousionRadious=11;
    double segmentDist=1;
    /**deterime if the provided point is insid of a hub
     *
     * @param p the point to checked
     * @return boolen, true if the provided point is inside a hub
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

    /**determine if the providdede point is inside the horoxontal berrier
     *
     * @param p the point to checked
     * @return boolen, true if the provided point is inside the barrier
     */
    boolean insideBarrierH(Point p){
        //13.68 99.5 116.32 5.77
        if(p.x>=13.68&&p.x<=13.68+116.32&&p.y>=99.5-5.77&&p.y<=99.5){
            return true;
        }
        return false;
    }

    /**deterimes if the provided point is inside the left verticle barrier
     *
     * @param p the point to checked
     * @return boolen, true if the provided point is inside the barrier
     */
    boolean insideBarrierVL(Point p){
        //13.68 99.5 116.32 5.77
        if(p.x>=44.6&&p.x<=44.6+5.77&&p.y>=130.2-30.75&&p.y<=130.2){
            return true;
        }
        return false;
    }

    /**determines if the pprovided point is inside the right verticle barrier
     *
     * @param p the point to checked
     * @return boolen, true if the provided point is inside the barrier
     */
    boolean insideBarrierVR(Point p){
        //13.68 99.5 116.32 5.77
        if(p.x>=93.75&&p.x<=93.75+5.77&&p.y>=130.2-30.75&&p.y<=130.2){
            return true;
        }
        return false;
    }

    /**generates a path as an array llist of points that goes from start to end without running into any obsticals
     *
     * @param start the point to start the path at (usualy the robots current position)
     * @param end the point to end the path at
     * @return an arraylist of points that form a path between the provided point
     */
    ArrayList<Point> createPath(Point start,Point end){
        ArrayList<Point> p=new ArrayList<>();
        p.add(start);
        boolean working=true;
        int itteration=0;
        while(working){
            double angle=Math.atan2((end.y-p.get(p.size()-1).y),(end.x-p.get(p.size()-1).x));//find the absoult angle to the next point in a straightvliine to the end point
            Point work;
            do{

                work =new Point(Math.cos(angle)*segmentDist+p.get(p.size()-1).x,Math.sin(angle)*segmentDist+p.get(p.size()-1).y,"");//create the next point
                angle += 0.01;//add 0.01 radians to the teroretical angle

            }while(insideHub(work));//if the created point was indide of a hub then calcuate the point again with the new agale and check again
            if(insideBarrierH(work)){//if the  calculated point is inside the horozontal barrier
                ArrayList<Point> temp;//create a temporary array list of points

                if(angle>0){//if the robot is heading up ish
                    if(work.x>72){//if it is on the right side of the field
                        temp=createPath(p.get(p.size()-1),new Point(137,92,""));//crate a path that goes to a pre defined point at the side of the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1),new Point(6,92,""));//crate a path that goes to a pre defined point at the side of the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge generated path into the current working path
                    }

                    if(work.x>72){//if it is on the right side of the fiels
                        temp=createPath(p.get(p.size()-1),new Point(137,104,""));//make a path going past the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1),new Point(6,104,""));//make a path going past the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge path into main path
                    }
                }else{//if the robot is heading down ish

                    if(work.x>72){//if it is on the right
                        temp=createPath(p.get(p.size()-1),new Point(137,104,""));//crate a path that goes to a pre defined point at the side of the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1),new Point(6,104,""));//crate a path that goes to a pre defined point at the side of the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    if(work.x>72){//if on the right side of the field
                        temp=createPath(p.get(p.size()-1),new Point(137,92,""));//make a path going past the barrier
                    }else{
                        temp=createPath(p.get(p.size()-1),new Point(6,92,""));//make a path going past the barrier
                    }
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }
            }else if (insideBarrierVL(work)){//path arround the left verticle barrier
                ArrayList<Point> temp;
                if(angle<Math.PI/2&&angle>-Math.PI/2){//if it is heading right
                    temp=createPath(p.get(p.size()-1),new Point(42,137,""));//crate a path that goes to a pre defined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1),new Point(54,137,""));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }else{//if the robot in heading left
                    temp=createPath(p.get(p.size()-1),new Point(54,137,""));//crate a path that goes to a pre defined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1),new Point(42,137,""));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }

            }else if (insideBarrierVR(work)){//path arropunf the right varticle barrier
                ArrayList<Point> temp;
                if(angle>Math.PI/2||angle<-Math.PI/2){//if the robot is heading left
                    temp=createPath(p.get(p.size()-1),new Point(101,137,""));//crate a path that goes to a pre defined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1),new Point(91,137,""));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }else{
                    temp=createPath(p.get(p.size()-1),new Point(91,137,""));//crate a path that goes to a pre defined point at the side of the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                    temp=createPath(p.get(p.size()-1),new Point(101,137,""));//make a path going past the barrier
                    for(int i=0;i<temp.size();i++){
                        p.add(temp.get(i));//merge the temporary path into the main path
                    }
                }
            }else{
                p.add(work);//add the current working point to the path
            }
            if(Math.sqrt(Math.pow(end.x-p.get(p.size()-1).x,2)+Math.pow(end.y-p.get(p.size()-1).y,2))<segmentDist)//if the point is less than the distacne of the segments from the end point
                working=false;//tell the loop to stop

            println("point "+p.size()+" "+p.get(p.size()-1).x+" "+p.get(p.size()-1).y);//write the current point to the console
            itteration++;//increase the itteration
            if(itteration>1000){//if the program is stuck in an infinite loop(too many itterations)
                return null;//stop the function
            }
        }
        path.add(end);//add the final point to the path
        println("point "+p.size()+" "+end.x+" "+end.y);//print the last point of the path to the console
        return p;
    }
}
