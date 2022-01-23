/* Authors: Ningning Ying, Elicia Esmeris, Smyan Sengupta, Cristian Santibanez, Arin Khare, Kristal Lin, Jesse Angrist
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Objects;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation
{
    public enum RotationDirection {CLOCKWISE, COUNTERCLOCKWISE}

    // AUTON CONSTANTS
    // ===============
    final double STRAFE_RAMP_DISTANCE = 10.0;  // Inches
    final double ROTATION_RAMP_DISTANCE = Math.PI / 4;  // Radians
    final int ENCODER_RAMP_DISTANCE = 1000;
    final double MAX_STRAFE_POWER = 1.0;
    final double MIN_STRAFE_POWER = 0.75;
    final double MAX_ROTATION_POWER = 0.75;
    final double MIN_ROTATION_POWER = 0.5;
    final boolean ROTATIONAL_RAMPING = false;
    // Accepted amounts of deviation between the robot's desired position and actual position.
    final double EPSILON_LOC = 1.0;
    final double EPSILON_ANGLE = 0.1;
    final int EPSILON_ENCODERS = 30;

    // TELEOP CONSTANTS
    // ================
    final double COARSE_MOVEMENT_MAX_POWER = 1.0;
    final double FINE_MOVEMENT_MAX_POWER = 0.5;
    final double COARSE_ROTATION_POWER = 0.8;
    final double FINE_ROTATION_POWER = 0.4;

    // INSTANCE ATTRIBUTES
    // ===================

    // Speeds relative to one another.
    //                              RL   RR   FL   FR
    public double[] wheel_speeds = {0.67, 0.67, 1.0, 1.0};
    public double strafePower;  // Tele-Op only

    // First position in this ArrayList is the first position that robot is planning to go to.
    // This condition must be maintained (positions should be deleted as the robot travels)
    // NOTE: a position is both a location and a rotation.
    // NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
    private ArrayList<Position> path;

    public Navigation(RobotManager.NavigationMode navMode, RobotManager.AllianceColor allianceColor) {
        switch (navMode) {
            case TELEOP:
                path = new ArrayList<>(Collections.emptyList());
                break;
            case DUCK_CAROUSEL:
                path = AutonomousPaths.DUCK_CAROUSEL_PATH;
                break;
            case DUCK_WAREHOUSE:
                path = AutonomousPaths.DUCK_WAREHOUSE_PATH;
                break;
            case NO_DUCK_CAROUSEL:
                path = AutonomousPaths.NO_DUCK_CAROUSEL_PATH;
                break;
            case NO_DUCK_WAREHOUSE:
                path = AutonomousPaths.NO_DUCK_WAREHOUSE_PATH;
                break;
        }

        if (allianceColor == RobotManager.AllianceColor.RED) {
            reflectPath();
        }
    }

    public Navigation(ArrayList<Position> path, RobotManager.AllianceColor allianceColor) {
        this.path = path;

        if (allianceColor == RobotManager.AllianceColor.RED) {
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
            travelLinear(target.getLocation(), robot);
            rotate(target.getRotation(), ROTATIONAL_RAMPING, robot);
            path.remove(0);
            if (target.getLocation().name.length() >= 3 && target.getLocation().name.substring(0, 3).equals("POI")) break;

            robot.telemetry.addData("Got to", target.getLocation().name);
            robot.telemetry.update();
        }
    }

    /** Updates the strafe power according to movement mode and gamepad 1 left trigger.
     *
     *  @return Whether the strafe power is greater than zero.
     */
    public void updateStrafePower(AnalogValues analogValues, Robot robot) {
        double throttle = analogValues.gamepad1LeftTrigger;
        if (throttle < 0.05) {  // Throttle dead zone.
            strafePower = 0.0;
            return;
        }
        if (robot.fineMovement) {
            strafePower = throttle * FINE_MOVEMENT_MAX_POWER;
        }
        else {
            strafePower = throttle * COARSE_MOVEMENT_MAX_POWER;
        }
    }

    /** Moves the robot straight in one of the cardinal directions or at a 45 degree angle.
     *
     *  @return whether any of the D-Pad buttons were pressed.
     */
    public boolean moveStraight(boolean forward, boolean backward, boolean left, boolean right, Robot robot) {
        double direction;
        if (forward) {
            if (left) {
                direction = Math.PI * 0.75;
            }
            else if (right) {
                direction = Math.PI * 0.25;
            }
            else {
                direction = Math.PI * 0.5;
            }
        }
        else if (backward) {
            if (left) {
                direction = -Math.PI * 0.75;
            }
            else if (right) {
                direction = -Math.PI * 0.25;
            }
            else {
                direction = -Math.PI * 0.5;
            }
        }
        else if (left) {
            direction = Math.PI;
        }
        else if (right) {
            direction = 0.0;
        }
        else {
            return false;
        }
        setDriveMotorPowers(direction, strafePower, 0.0, robot);
        return true;
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     */
    public void maneuver(AnalogValues analogValues, Robot robot) {
        // Uses left stick to go forward, and right stick to turn.
        // NOTE: right-side drivetrain motor inputs don't have to be negated because their directions will be reversed
        //       upon initialization.

        double turn = analogValues.gamepad1RightStickX;
        if (-0.05 < turn && turn < 0.05) {  // joystick dead zone
            turn = 0;
        }
        if (robot.fineRotation) {
            turn *= FINE_ROTATION_POWER;
        }
        else {
            turn *= COARSE_ROTATION_POWER;
        }

        double moveDirection = Math.atan2(analogValues.gamepad1LeftStickY, analogValues.gamepad1LeftStickX);
        setDriveMotorPowers(moveDirection, strafePower, turn, robot);
        robot.telemetry.addData("Left Stick Position",Math.toDegrees(moveDirection) + " degrees");
    }

    /** Rotates the robot a number of degrees.
     *
     * @param target The orientation the robot should assume once this method exits.
     *               Within the interval (-pi, pi].
     * @param ramping Whether to use ramping.
     */
    public void rotate(double target, boolean ramping, Robot robot)
    {
        robot.positionManager.updatePosition(robot);
        // Both values are restricted to interval (-pi, pi].
        final double startOrientation = robot.getPosition().getRotation();
        double currentOrientation = startOrientation;  // Copies by value because double is primitive.

        double rotationSize = getRotationSize(startOrientation, target);

        double power = MIN_ROTATION_POWER;  // If ramping is false, power will stay at this value.
        double rotationRemaining = getRotationSize(currentOrientation, target);
        double rotationProgress = getRotationSize(startOrientation, currentOrientation);

        while (rotationRemaining > EPSILON_ANGLE) {

            if (ramping) {
                if (rotationProgress < rotationSize / 2) {
                    // Ramping up.
                    if (rotationProgress <= ROTATION_RAMP_DISTANCE) {
                        power = Range.clip(
                                (rotationProgress / ROTATION_RAMP_DISTANCE) * MAX_ROTATION_POWER,
                                MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                    }
                }
                else {
                    // Ramping down.
                    if (rotationRemaining <= ROTATION_RAMP_DISTANCE) {
                        power = Range.clip(
                                (rotationRemaining / ROTATION_RAMP_DISTANCE) * MAX_ROTATION_POWER,
                                MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                    }
                }
            }

            switch (getRotationDirection(currentOrientation, target)) {
                case CLOCKWISE:
                    setDriveMotorPowers(0.0, 0.0, power, robot);
                    break;
                case COUNTERCLOCKWISE:
                    setDriveMotorPowers(0.0, 0.0, -power, robot);
                    break;
            }

            robot.positionManager.updatePosition(robot);
            currentOrientation = robot.getPosition().getRotation();

            rotationRemaining = getRotationSize(currentOrientation, target);
            rotationProgress = getRotationSize(startOrientation, currentOrientation);
        }

        stopMovement(robot);
    }

    /** Rotates the robot a specified amount.
     *
     *  @param targetEncoderCounts The number of average encoder counts that the drivetrain motors must travel before exiting
     *                             the method.
     *  @param direction The direction in which the robot is to rotate.
     *
     *  Note that this function keeps track of the difference in means, not the mean of the differences. It turns out
     *  those are the same thing, and the former is easier to use.
     */
    public void rotateRelative(int targetEncoderCounts, RotationDirection direction, boolean ramping, Robot robot) {
        int[] startEncoderCounts = getDriveEncoderCounts(robot);
        int[] currentEncoderCounts = getDriveEncoderCounts(robot);

        double power = MIN_ROTATION_POWER;
        double distanceToTarget = targetEncoderCounts;
        double distanceTraveled = 0;

        while (distanceToTarget > EPSILON_ENCODERS) {
            if (ramping) {
                // Ramping up.
                if (distanceTraveled <= ENCODER_RAMP_DISTANCE) {
                    power = Range.clip(
                            (distanceTraveled / ENCODER_RAMP_DISTANCE) * MAX_ROTATION_POWER,
                            MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                }
                // Ramping down.
                if (distanceToTarget <= ENCODER_RAMP_DISTANCE) {
                    power = Range.clip(
                            (distanceToTarget / ENCODER_RAMP_DISTANCE) * MAX_ROTATION_POWER,
                            MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                }
            }

            switch (direction) {
                case CLOCKWISE:
                    setDriveMotorPowers(0, 0, power, robot);
                case COUNTERCLOCKWISE:
                    setDriveMotorPowers(0, 0, -power, robot);
            }

            currentEncoderCounts = getDriveEncoderCounts(robot);
            distanceTraveled = getMeanAbsDiff(currentEncoderCounts, startEncoderCounts);
            distanceToTarget = targetEncoderCounts - distanceTraveled;
        }

        stopMovement(robot);
    }

    /** Returns an array containing the encoder counts of the drivetrain motors.
     */
    private int[] getDriveEncoderCounts(Robot robot) {
        int[] encoderCounts = new int[4];
        int i = 0;
        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            encoderCounts[i] = Math.abs(robot.driveMotors.get(motor).getCurrentPosition());
        }
        return encoderCounts;
    }

    /** Calculates the mean of the element-wise absolute differences between two arrays of equal length.
     */
    private double getMeanAbsDiff(int[] arr1, int[] arr2) {
        double sumAbsDiff = 0;
        for (int i = 0; i < arr1.length; i++) {
            sumAbsDiff += Math.abs(arr1[i] - arr2[i]);
        }
        return sumAbsDiff / arr1.length;
    }

    /** Determines whether the robot has to turn clockwise or counterclockwise to get from theta to target.
     */
    private RotationDirection getRotationDirection(double theta, double target) {
        double angleDiff = target - theta;  // Counterclockwise distance to target
        if ((angleDiff >= -Math.PI && angleDiff < 0) || (angleDiff > Math.PI)) {
            return RotationDirection.CLOCKWISE;
        }
        return RotationDirection.COUNTERCLOCKWISE;
    }

    /** Calculates the number of radians of rotation required to get from theta to target.
     */
    private double getRotationSize(double theta, double target) {
        double rotationSize = Math.abs(target - theta);
        if (rotationSize > Math.PI) {
            rotationSize = 2 * Math.PI - rotationSize;
        }
        return rotationSize;
    }

    /** Makes the robot travel in a straight line for a certain distance.
     *
     *  @param target The desired position of the robot.
     */
    public void travelLinear(Point target, Robot robot) {
        robot.positionManager.updatePosition(robot);
        final Point startLoc = robot.getPosition().getLocation();
        Point currentLoc = new Point(startLoc.x, startLoc.y, "current");

        double totalDistance = getEuclideanDistance(startLoc, target);

        double power = MIN_STRAFE_POWER;
        double distanceToTarget = getEuclideanDistance(currentLoc, target);
        double distanceTraveled = getEuclideanDistance(startLoc, currentLoc);

        while (distanceToTarget > EPSILON_LOC) {

            if (distanceTraveled < totalDistance / 2) {
                // Ramping up.
                if (distanceTraveled <= STRAFE_RAMP_DISTANCE) {
                    power = Range.clip(
                            (distanceTraveled / STRAFE_RAMP_DISTANCE) * MAX_STRAFE_POWER,
                            MIN_STRAFE_POWER, MAX_STRAFE_POWER);
                }
            }
            else {
                // Ramping down.
                if (distanceToTarget <= STRAFE_RAMP_DISTANCE) {
                    power = Range.clip(
                            (distanceToTarget / STRAFE_RAMP_DISTANCE) * MAX_STRAFE_POWER,
                            MIN_STRAFE_POWER, MAX_STRAFE_POWER);
                }
            }

            setDriveMotorPowers(getAngleBetween(currentLoc, target), power, 0.0, robot);

            robot.positionManager.updatePosition(robot);
            currentLoc = robot.getPosition().getLocation();

            distanceToTarget = getEuclideanDistance(currentLoc, target);
            distanceTraveled = getEuclideanDistance(startLoc, currentLoc);

//            robot.telemetry.addData("X", currentLoc.x);
//            robot.telemetry.addData("Y", currentLoc.y);
//            robot.telemetry.addData("dX", target.x);
//            robot.telemetry.addData("dY", target.y);
//            robot.telemetry.update();
        }

        stopMovement(robot);
    }

    /** Determines the angle between the horizontal axis and the segment connecting A and B.
     */
    private double getAngleBetween(Point a, Point b) { return -Math.atan2((b.y - a.y), (b.x - a.x)); }

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
    private void reflectPath() {
        for (Position pos : path) {
            pos.setX(144.0 - pos.getX());
            pos.setRotation(Math.PI - pos.getRotation());
        }
    }

    /** Sets drive motor powers to make the robot move a certain way.
     *
     *  @param strafeDirection the direction in which the robot should strafe.
     *  @param power the speed at which the robot should strafe. Must be in the interval [-1, 1]. Set this to zero if
     *               you only want the robot to rotate.
     *  @param turn the speed at which the robot should rotate (clockwise). Must be in the interval [-1, 1]. Set this to
     *              zero if you only want the robot to strafe.
     */
    private void setDriveMotorPowers(double strafeDirection, double power, double turn, Robot robot) {
//        strafeDirection *= -1;
        double sinMoveDirection = Math.sin(strafeDirection);
        double cosMoveDirection = Math.cos(strafeDirection);

        double powerSet1 = sinMoveDirection + cosMoveDirection;
        double powerSet2 = sinMoveDirection - cosMoveDirection;
        double [] rawPowers = scaleRange(powerSet1, powerSet2);

        robot.driveMotors.get(RobotConfig.DriveMotors.REAR_LEFT).setPower((rawPowers[1] * power + turn) * wheel_speeds[0]);
        robot.driveMotors.get(RobotConfig.DriveMotors.REAR_RIGHT).setPower((rawPowers[0] * power - turn) * wheel_speeds[1]);
        robot.driveMotors.get(RobotConfig.DriveMotors.FRONT_LEFT).setPower((rawPowers[0] * power + turn) * wheel_speeds[2]);
        robot.driveMotors.get(RobotConfig.DriveMotors.FRONT_RIGHT).setPower((rawPowers[1] * power - turn) * wheel_speeds[3]);

        robot.telemetry.addData("Front Motors", "left (%.2f), right (%.2f)",
                (rawPowers[0] * power + turn) * wheel_speeds[2], (rawPowers[1] * power - turn) * wheel_speeds[3]);
        robot.telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)",
                (rawPowers[1] * power + turn) * wheel_speeds[0], (rawPowers[0] * power - turn) * wheel_speeds[1]);
    }

    /** Sets all drivetrain motor powers to zero.
     */
    private void stopMovement(Robot robot) {
        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            Objects.requireNonNull(robot.driveMotors.get(motor)).setPower(0.0);
        }
    }

    /**preserves the ratio between a and b while restricting them to the range [-1, 1]
     *
     * @param a value to be scaled
     * @param b value to be scaled
     * @return an array containing the scaled versions of a and b
     */
    double[] scaleRange(double a,double b){
        double max;
        if (Math.abs(a) > Math.abs(b)) {
            max = Math.abs(a);
        }
        else {
            max = Math.abs(b);
        }
        return new double[] {a / max, b / max};
    }

    // PATHFINDING
    // ===========

//    //variables relating to the operations of pathfinding
//    int hubexclousionRadious=11;
//    double segmentDist=1;
//    /**determine if the provided point is inside a hub
//     *
//     * @param p the point to checked
//     * @return boolean, true if the provided point is inside a hub
//     */
//    boolean insideHub(Point p){
//        if(Math.sqrt(Math.pow(p.x-72,2)+Math.pow(p.y-120,2))<=hubexclousionRadious){
//            return true;
//        }
//        if(Math.sqrt(Math.pow(p.x-48,2)+Math.pow(p.y-60,2))<=hubexclousionRadious){
//            return true;
//        }
//        if(Math.sqrt(Math.pow(p.x-96,2)+Math.pow(p.y-60,2))<=hubexclousionRadious){
//            return true;
//        }
//        return false;
//    }
//
//    /**determine if the provided point is inside the horizontal barrier
//     *
//     * @param p the point to checked
//     * @return boolean, true if the provided point is inside the barrier
//     */
//    boolean insideBarrierH(Point p){
//        //13.68 99.5 116.32 5.77
//        if(p.x>=13.68&&p.x<=13.68+116.32&&p.y>=99.5-5.77&&p.y<=99.5){
//            return true;
//        }
//        return false;
//    }
//
//    /**determines if the provided point is inside the left vertical barrier
//     *
//     * @param p the point to checked
//     * @return boolean, true if the provided point is inside the barrier
//     */
//    boolean insideBarrierVL(Point p){
//        //13.68 99.5 116.32 5.77
//        if(p.x>=44.6&&p.x<=44.6+5.77&&p.y>=130.2-30.75&&p.y<=130.2){
//            return true;
//        }
//        return false;
//    }
//
//    /**determines if the provided point is inside the right vertical barrier
//     *
//     * @param p the point to checked
//     * @return boolean, true if the provided point is inside the barrier
//     */
//    boolean insideBarrierVR(Point p){
//        //13.68 99.5 116.32 5.77
//        if(p.x>=93.75&&p.x<=93.75+5.77&&p.y>=130.2-30.75&&p.y<=130.2){
//            return true;
//        }
//        return false;
//    }
//
//    /**generates a path as an array list of points that goes from start to end without running into any obstacles
//     *it is recommended that you run the output of this function through optimisePath1 and then optimisePath2
//     * @link https://github.com/jSdCool/FTC-robot-pathfinging for a visual deminstration
//     * @param start the point to start the path at (usually the robots current position)
//     * @param end the point to end the path at
//     * @return an arraylist of points that form a path between the provided point
//     */
//    ArrayList<Position> createPath(Position start,Position end){
//        ArrayList<Position> p=new ArrayList<Position>();
//        p.add(start);
//        boolean working=true;
//        int itteration=0;
//        double anglein=start.rotation;
//        while(working){
//            double angle=Math.atan2((end.location.y-p.get(p.size()-1).location.y),(end.location.x-p.get(p.size()-1).location.x));//find the absolute angle to the next point in a straight line to the end point
//            Point work;
//            do{
//
//                work =new Point(Math.cos(angle)*segmentDist+p.get(p.size()-1).location.x,Math.sin(angle)*segmentDist+p.get(p.size()-1).location.y,"");//create the next point
//                angle += 0.01;//add 0.01 radians to the theoretical angle
//
//            }while(insideHub(work));//if the created point was inside a hub then calculate the point again with the new agale and check again
//            if(insideBarrierH(work)){//if the  calculated point is inside the horizontal barrier
//                ArrayList<Position> temp;//create a temporary array list of points
//
//                if(angle>0){//if the robot is heading up ish
//                    if(work.x>72){//if it is on the right side of the field
//                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,92,""),Math.PI));//crate a path that goes to a predefined point at the side of the barrier
//                    }else{
//                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,92,""),0));//crate a path that goes to a predefined point at the side of the barrier
//                    }
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge generated path into the current working path
//                    }
//                    if(work.x>72){//if it is on the right side of the field
//                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,104,""),Math.PI));//make a path going past the barrier
//                    }else{
//                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,104,""),0));//make a path going past the barrier
//                    }
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge path into main path
//                    }
//                }else{//if the robot is heading down ish
//                    if(work.x>72){//if it is on the right
//                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,104,""),Math.PI));//crate a path that goes to a predefined point at the side of the barrier
//                    }else{
//                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,104,""),0));//crate a path that goes to a predefined point at the side of the barrier
//                    }
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                    if(work.x>72){//if on the right side of the field
//                        temp=createPath(p.get(p.size()-1).setRotation(Math.PI),new Position(new Point(137,92,""),Math.PI));//make a path going past the barrier
//                    }else{
//                        temp=createPath(p.get(p.size()-1).setRotation(0),new Position(new Point(6,92,""),0));//make a path going past the barrier
//                    }
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                }
//            }else if (insideBarrierVL(work)){//path around the left vertical barrier
//                ArrayList<Position> temp;
//                if(angle<Math.PI/2&&angle>-Math.PI/2){//if it is heading right
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(42,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(54,137,""),-Math.PI/2));//make a path going past the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                }else{//if the robot in heading left
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(54,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(42,137,""),-Math.PI/2));//make a path going past the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                }
//
//            }else if (insideBarrierVR(work)){//path around the right vertical barrier
//                ArrayList<Position> temp;
//                if(angle>Math.PI/2||angle<-Math.PI/2){//if the robot is heading left
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(101,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(91,137,""),-Math.PI/2));//make a path going past the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                }else{
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(91,137,""),-Math.PI/2));//crate a path that goes to a predefined point at the side of the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                    temp=createPath(p.get(p.size()-1).setRotation(-Math.PI/2),new Position(new Point(101,137,""),-Math.PI/2));//make a path going past the barrier
//                    for(int i=0;i<temp.size();i++){
//                        p.add(temp.get(i));//merge the temporary path into the main path
//                    }
//                }
//            }else{
//                p.add(new Position(work,anglein));//add the current working point to the path
//            }
//            if(Math.sqrt(Math.pow(end.location.x-p.get(p.size()-1).location.x,2)+Math.pow(end.location.y-p.get(p.size()-1).location.y,2))<segmentDist)//if the point is less than the distance of the segments from the end point
//                working=false;//tell the loop to stop
//
//
//            itteration++;//increase the iteration
//            if(itteration>1000){//if the program is stuck in an infinite loop(too many iterations)
//                return null;
//            }
//        }
//        p.add(end);//add the final point to the path
//        return p;
//    }
//
//    /**the first step in optimising a path, this function reduces the numbers of point in a path by detecting straight lines and removing the points that make them up
//     @param p the path that you want to optimise
//     @return a path that contains fewer points
//     */
//    ArrayList<Position> optimisePath1(ArrayList<Position> p){
//        ArrayList<Position> o=new ArrayList<Position>();//the object to return
//        o.add(p.get(0));//add the first point of the path to the new path
//        int beginindex=0;
//        double devation=0.01;//how far(in radians) is a line allowed to lean in either direction before it is consisted a new line
//        double angle=Math.atan2(p.get(1).location.y-p.get(0).location.y,p.get(1).location.x-p.get(0).location.x);//calculate the initial angle that the line is going in
//        for(int i=1;i<p.size();i++){
//            double newAngle=Math.atan2(p.get(i).location.y-p.get(beginindex).location.y,p.get(i).location.x-p.get(beginindex).location.x);//calculate the angle between the base point of the current line and the next point in the list
//            if(newAngle>=angle-devation&&newAngle<=angle+devation){//if the angle is inside the acceptable range
//                continue;
//            }else{
//                o.add(p.get(i-1));//add the previous point to the optimised path
//                beginindex=i;//set the current point as the new base point
//                angle=Math.atan2(p.get(i).location.y-p.get(i-1).location.y,p.get(i).location.x-p.get(i-1).location.x);//calculate the new angle of the next line
//            }
//        }
//        o.add(p.get(p.size()-1));//add the  final point to the new path
//        return o;
//    }
//
//    /**the second step in optimizing paths this function generates paths between point in a given path to see if it can find a shorter path between them
//     @param path the path that you want to optimise that has been run through optimisePath1
//     @return a path that has a shorter overall travel
//     */
//    ArrayList<Position> optimisePath2(ArrayList<Position> path){
//        ArrayList<Position> p=new ArrayList(path);//copy the input path
//
//        if(p.size()==2){//if the path only consists of 2 points then the path can not be optimised so do nothing
//            return p;
//        }
//
//        ArrayList<Position> o=new ArrayList<Position>();//the object to return
//
//        for(int i=0;i<p.size()-1;){//seek through the path
//            int curbest=i+1,sigbest=-1;
//            for(int j=i+1;j<p.size();j++){//check every point in the path ahead of this point
//                double l1,l2;
//                ArrayList<Position> temp=new ArrayList<Position>(),temp2;//create temporary paths
//                for(int n=i;n<=j;n++){//make the temp path the section of the main path between the 2 points
//                    temp.add(p.get(n));
//                }
//                temp2=optimisePath1(createPath(p.get(i),p.get(j)));//generate a new path directly between the 2 selected points
//                l1=pathlength(temp2);
//                l2=pathlength(temp);
//                if(l1<l2){//compare the lengths of the paths, if the new path is less than the original
//                    curbest=j;//set the current best index to j
//                    if(sigbest==-1){//if the best significant is -1 then set it to the current best
//                        sigbest=curbest;
//                    }
//                }
//
//                if(l1<=l2*0.7){//if this path is significantly shorter than the old best then set sigbest to this path      this value may need to be tweaked
//                    sigbest=j;
//                }
//
//            }//end of loop
//            if(sigbest==-1){//if the best significant is -1 then set it to the current best
//                sigbest=curbest;
//            }
//            ArrayList<Position> temp=new ArrayList<Position>();//create a temp path
//            temp=optimisePath1(createPath(p.get(i),p.get(sigbest)));//set the temp path to the new best path
//            for(int j=0;j<temp.size();j++){
//                o.add(temp.get(j));//add the new best path to the output
//            }
//            i=sigbest;
//        }
//
//        return o;
//    }
//
//    /**gets the total travel distance of a path
//    @param p the path you want the length of
//    @return the length of the path
//    */
//    double pathlength(ArrayList<Position> p){
//        double length=0;
//        for(int i=0;i<p.size()-1;i++){
//            length+=Math.sqrt(Math.pow(p.get(i+1).location.y-p.get(i).location.y,2)+Math.pow(p.get(i+1).location.x-p.get(i).location.x,2));
//        }
//        return length;
//    }
}


/** Hardcoded paths through the playing field during the Autonomous period.
 */
class AutonomousPaths {
    public static final ArrayList<Position> DUCK_CAROUSEL_PATH = new ArrayList<>(Arrays.asList());
    public static final ArrayList<Position> DUCK_WAREHOUSE_PATH = new ArrayList<>(Arrays.asList());
    public static final ArrayList<Position> NO_DUCK_CAROUSEL_PATH = new ArrayList<>(Arrays.asList());
    public static final ArrayList<Position> NO_DUCK_WAREHOUSE_PATH = new ArrayList<>(Arrays.asList());

    // TESTING PATHS
    // =============

    // NOTE:
    // - These currently only incorporate strafing at intervals of pi/2, moving forward/backward whenever possible.
    // - These assume both orientation and location to be relative to the robot's starting position.
    public static final ArrayList<Position> PRELOAD_BOX_ONLY = new ArrayList<>(Arrays.asList(
            new Position(new Point(10, 0, "Out from wall"), 0),
            new Position(new Point(10, 10, "In line with shipping hub"), 0),
            new Position(new Point(10, 10, "Facing shipping hub"), -Math.PI / 2),
            new Position(new Point(20, 10, "POI Shipping hub"), -Math.PI / 2)
    ));
    public static final ArrayList<Position> PRELOAD_BOX_AND_PARK = new ArrayList<>(Arrays.asList(
            new Position(new Point(10, 0, "Out from wall"), 0),
            new Position(new Point(10, 10, "In line with shipping hub"), 0),
            new Position(new Point(10, 10, "Facing shipping hub"), -Math.PI / 2),
            new Position(new Point(20, 10, "POI Shipping hub"), -Math.PI / 2),
            new Position(new Point(15, 10, "Backed up from shipping hub"), -Math.PI / 2),
            new Position(new Point(15, 10, "Facing storage unit"), Math.PI),
            new Position(new Point(15, -20, "Partially in storage unit"), Math.PI),
            new Position(new Point(25, -20, "POI Parked"), Math.PI)
    ));
    public static final ArrayList<Position> MOVE_STRAIGHT = new ArrayList<>(Arrays.asList(
            new Position(new Point(0, 20, "P1"), 0)
    ));
    public static final ArrayList<Position> ROTATE_180 = new ArrayList<>(Arrays.asList(
            new Position(new Point(0, 0, "P1"), Math.PI)
    ));
}
