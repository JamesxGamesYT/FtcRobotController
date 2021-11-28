/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;


/** A completely encompassing class of all functionality of the robot. An OpMode should interface through an instance of
 *  this class in order to send or receive any data with the real robot.
 */
public class RobotManager {

    // DUCK: deliver duck from carousel.
    // FREIGHT: deliver one piece of freight from the warehouse to the shipping hub.
    enum AutonMode {DUCK, FREIGHT}

    public Robot robot;

    public EncoderPositioning encoderPositioning;
    public CVPositioning cvPositioning;
    public MechanismDriving mechanismDriving;
    public Navigation navigation;

    private Gamepad gamepad1PreviousState;
    private Gamepad gamepad2PreviousState;

    public RobotManager(HardwareMap hardwareMap) {
        // Construct each team class and robot state.
        mechanismDriving = new MechanismDriving();
        navigation = new Navigation(new ArrayList<>());
        robot = new Robot(hardwareMap);
    }

    // TELE-OP
    // =======

    /** Determine new robot states based on controller input
     */
    public void readControllerInputs(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1PreviousState == null || gamepad2PreviousState == null) {
            gamepad1PreviousState = new Gamepad();
            gamepad2PreviousState = new Gamepad();
            return;
        }

        // On button release.
        if (!gamepad1.a && gamepad1PreviousState.a) {
            if (robot.carouselMotorState == Robot.CarouselMotorState.CHECK_START) {
                robot.carouselMotorState = Robot.CarouselMotorState.SPIN;
            }
            else {
                robot.carouselMotorState = Robot.CarouselMotorState.CHECK_START;
            }
        }

        try {
            gamepad1PreviousState.copy(gamepad1);
            gamepad2PreviousState.copy(gamepad2);
        } catch (RobotCoreException e) {}
    }

    /** Calls all non-blocking FSM methods to read from state and act accordingly.
     */
    public void driveMechanisms() {
        mechanismDriving.activateCarousel(robot);
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     * @param leftStickX the left stick x-axis
     * @param leftStickY the left stick y-axis
     * @param rightStickX the right stick x-axis
     * @param rightStickY the right stick y-xis
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
        navigation.maneuver(leftStickX, leftStickY, rightStickX, rightStickY, robot);
    }

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