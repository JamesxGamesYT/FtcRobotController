/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.Range;

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
    public void travelToNextPOI(Robot robot) {
        // Repeat until at POI:
        // - realPos = <get position from robot>
        // - while realPos and path[0] are not within epsilon values:
        //     - angle = getAngleBetween(currentPosition, path[0])
        //     - dist = getEuclideanDistance(currentPosition, path[0])
        //     - travelLinear(angle, dist)
        //     - rotate(desiredRotation - currentRotation)
        //     - update realPos
        // - remove path[0] from path
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
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

        power = Range.clip(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2)),0,1);
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

        robot.telemetry.addData("Left Stick Position",Math.toDegrees(moveDirection) + " degrees");
        robot.telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        robot.telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)", rearLeftPower, rearRightPower);
    }

    /** Rotates the robot a number of degrees.
     */
    private void rotate(double angle, Robot robot) 
    {
        // Assign the original rotation to a variable
        robot.position.position.rotation;
        
        // Set motor powers to start rotating the robot
        robot.rearLeftDrive.setPower();
        // Wait until the robot is rotated to the desired angle.
        while (/*rotation is not done*/) {
            TimeUnit.SECONDS.sleep(1);
        }
    }

    /** Makes the robot travel in a straight line for a certain distance.
     *  @param desiredPosition The desired position of the robot.
     *  @param angle The angle in which the robot will strafe.
     *  @param travelDirection The direction in which the robot will be traveling once it has turned to the desired angle.
     */
    private void travelLinear(Position desiredPosition, double angle, @NotNull TravelDirection travelDirection, double generalPower, Robot robot) throws InterruptedException {
        double frontLeftPower = 0; double frontRightPower = 0; double rearLeftPower = 0; double rearRightPower = 0;

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

        robot.frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        rotate(angle, robot);

        robot.frontLeftDrive.setPower(frontLeftPower);
        robot.frontRightDrive.setPower(frontRightPower);
        robot.rearLeftDrive.setPower(rearLeftPower);
        robot.rearRightDrive.setPower(rearRightPower);

        while(!robot.position.equals(desiredPosition)) {
            TimeUnit.SECONDS.sleep(1);
        }

        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
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

//            println("point "+p.size()+" "+p.get(p.size()-1).x+" "+p.get(p.size()-1).y);//write the current point to the console
            itteration++;//increase the itteration
            if(itteration>1000){//if the program is stuck in an infinite loop(too many itterations)
                return null;//stop the function
            }
        }
//        path.add(end);//add the final point to the path
//        println("point "+p.size()+" "+end.x+" "+end.y);//print the last point of the path to the console
        return p;
    }
}
