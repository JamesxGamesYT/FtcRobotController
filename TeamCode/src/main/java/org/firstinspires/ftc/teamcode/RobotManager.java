/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/** A completely encompassing class of all functionality of the robot. An OpMode should interface through an instance of
 *  this class in order to send or receive any data with the real robot.
 */
public class RobotManager {

    // DUCK: deliver duck from carousel.
    // FREIGHT: deliver one piece of freight from the warehouse to the shipping hub.
    enum AutonMode {DUCK, FREIGHT}

    public Robot robotState;

    public EncoderPositioning encoderPositioning;
    public CVPositioning cvPositioning;
    public MechanismDriving mechanismDriving;
    public Navigation navigation;

    private Gamepad gamepad1, gamepad2;

    public RobotManager(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Construct each team class and robot state.
    }

    // TELE-OP
    // =======

    /** Determine new robot states based on controller input
     */
    public void readControllerInputs() {
        // If a button is pressed:
        // - Change the corresponding state in robotState
    }

    /** Calls all non-blocking FSM methods to read from state and act accordingly.
     */
    public void driveMechanisms() {}

    /** Changes drivetrain motor inputs based off the controller inputs.
     * @param leftStickX the left stick x-axis
     * @param leftStickY the left stick y-axis
     * @param rightStickX the right stick x-axis
     * @param rightStickY the right stick y-xis
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {}

    // AUTONOMOUS
    // ==========

    /** Adds necessary points to the robot's itinerary for the Autonomous period.
     * @param mode the position of the duck and warehouse freight as defined by {@link AutonMode}
     */
    public void initAutonPath(AutonMode mode) {}

    /** Moves the robot to the next point of interest.
     */
    public void goToNextPOI() {}

    /** Determines the position of the capstone on the barcode.
     *  @return 0 indicates the position closest to the hub, 1 indicates the middle position, 2 indicates the position
     *          farthest from the hub.
     *
     *  TODO: figure out a CV system that lets us implement this as a synchronous method; or, figure out a better system.
     */
    public int readBarcode() { return 0; }

    // Each of these methods manually sets the robot state so that a specific task is started, and forces these tasks to
    // be synchronous by repeatedly calling the mechanism driving methods. These are to be used in a linear auton opmode.

    /** Delivers a duck by spinning the carousel.
     */
    public void deliverDuck() {}

    /** Picks up a piece of freight from the warehouse.
     */
    public void obtainFreight() {}

    /** Delivers a piece of freight to a particular level of the alliance shipping hub.
     * @param level the level to witch the cargo needs to be delivered 1-bottom level 2-middle level 3-top level
     */
    public void deliverToShippingHub(int level) {}
}
