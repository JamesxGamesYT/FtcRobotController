/* Authors: Arin Khare
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/** A completely encompassing class of all functionality of the robot. An OpMode should interface through an instance of
 *  this class in order to send or receive any data with the real robot.
 *
 *  OpModes should pass `this` into the Robot constructor.
 */
public class Robot {

    // DUCK: deliver duck from carousel.
    // FREIGHT: deliver one piece of freight from the warehouse to the shipping hub.
    enum AUTON_MODE {DUCK, FREIGHT}

    private DcMotor carousel, slidesLeft, slidesRight, frontRightDrive, rearRightDrive, frontLeftDrive, rearLeftDrive;
    private Servo claw;

    private EncoderPositioning encoderPositioning;
    private CVPositioning cvPositioning;
    private MechanismDriving mechanismDriving;
    private Navigation navigation;

    public Robot() {}

    // MECHANISM DRIVING
    // =================

    /** Delivers a duck by spinning the carousel.
     */
    public void deliverDuck() {}

    /** Picks up a piece of freight from the warehouse.
     */
    public void obtainFreight() {}

    /** Delivers a piece of freight to a particular level of the alliance shipping hub.
     * @param level the level to witch the cargo needs to be deliverd 1-bottom level 2-miiddle level 3-top level
     */
    public void deliverToShippingHub(int level) {}

    /** Delivers a piece of freight the alliance storage unit.
     */
    public void deliverToStorageUnit() {}

    /** Places the capstone on the top of the alliance shipping hub.
     */
    public void placeCapstone() {}

    // NAVIGATION
    // ==========

    /** Adds necessary points to the robot's itinerary for the Autonomous period.
     * @param the position of the dick and whare house freight as defind by {@link AUTON_MODE}
     */
    public void initAutonPath(AUTON_MODE mode) {}

    /** Moves the robot to the next point of interest.
     */
    public void goToNextPOI() {}

    /** Changes drivetrain motor inputs based off the controller inputs.
     * @param leftStickX the left stick x-axis
     * @param leftStickY the left sdtaick y-axis
     * @param rightStickX the right stick x-axis
     * @param rightStickY the right staick y-xis
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {}

    // OTHER
    // =====

    /** Determines the position of the capstone on the barcode.
     *  @return 0 indicates the position closest to the hub, 1 indicates the middle position, 2 indicates the position
     *          farthest from the hub.
     */
    public int readBarcode() { return 0; }
}
