/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin
 */

package org.firstinspires.ftc.teamcode;


import java.util.*;

import com.qualcomm.robotcore.util.Range;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation
{
    // DUCK: deliver duck from carousel.
    // FREIGHT: deliver one piece of freight from the warehouse to the shipping hub.
    public enum NavigationMode {DUCK, FREIGHT, TELEOP}

    public enum AllianceColor {BLUE, RED}

    final double SPEED = 1.0;
    // Accepted amounts of deviation between the robot's desired position and actual position.
    final double EPSILON_LOC = 0.1;
    final double EPSILON_ANGLE = 0.1;

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
    public void maneuver(JoystickValues joystickValues, Robot robot) {
    }

    /** Rotates the robot a number of degrees.
     */
    private void rotate(double angle, Robot robot) 
    {
        // Assign the original rotation to a variable
        robot.positionManager.position.rotation;
        
        // Set motor powers to start rotating the robot
        robot.rearLeftDrive.setPower();
        // Wait until the robot is rotated to the desired angle.
        while (/*rotation is not done*/) {
            sleep(1);
        }
    }

    /** Makes the robot travel in a straight line for a certain distance.
     *  @param dist The distance the robot should travel.
     *  @param angle The angle in which the robot will strafe.
     */
    private void travelLinear(double dist, double angle, Robot robot) {}

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

    /** Reflects the path to the other side of the playing field.
     */
    private void reflectPath() {}

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


/** Hardcoded paths through the playing field during the Autonomous period.
 */
class AutonomousPaths {
    public static final ArrayList<Position> DUCK_PATH = new ArrayList<>(Arrays.asList(
            // Construct Position objects
    ));
    public static final ArrayList<Position> FREIGHT_PATH = new ArrayList<>(Arrays.asList());
}