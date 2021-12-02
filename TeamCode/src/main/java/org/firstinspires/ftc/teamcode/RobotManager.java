/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


/** A completely encompassing class of all functionality of the robot. An OpMode should interface through an instance of
 *  this class in order to send or receive any data with the real robot.
 */
public class RobotManager {

    public Robot robot;

    public MechanismDriving mechanismDriving;
    public Navigation navigation;

    private GamepadWrapper gamepads, previousStateGamepads;

    public RobotManager(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2,
                        Navigation.NavigationMode navMode, Navigation.AllianceColor allianceColor) {

        navigation = new Navigation(navMode, allianceColor);
        mechanismDriving = new MechanismDriving();

        robot = new Robot(hardwareMap);

        gamepads = new GamepadWrapper(gamepad1, gamepad2);
        previousStateGamepads = new GamepadWrapper();
    }

    // TELE-OP
    // =======

    /** Determine new robot states based on controller input
     */
    public void readControllerInputs() {
        // First iteration of OpMode, there's no way there was a button release.
        if (previousStateGamepads.firstIteration) {
            return;
        }

        // Detect button releases.

        if (!gamepads.getButtonState(GamepadWrapper.DriverAction.START_STOP_CAROUSEL)
                && previousStateGamepads.getButtonState(GamepadWrapper.DriverAction.START_STOP_CAROUSEL)) {
            if (robot.carouselMotorState == Robot.CarouselMotorState.CHECK_START) {
                robot.carouselMotorState = Robot.CarouselMotorState.SPIN;
            }
            else {
                robot.carouselMotorState = Robot.CarouselMotorState.CHECK_START;
            }
        }
    }

    /** Calls all non-blocking FSM methods to read from state and act accordingly.
     */
    public void driveMechanisms() {}

    /** Changes drivetrain motor inputs based off the controller inputs.
     * @param leftStickX the left stick x-axis
     * @param leftStickY the left stick y-axis
     * @param rightStickX the right stick x-axis
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX) {}

    // AUTONOMOUS
    // ==========

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
    // be synchronous by repeatedly calling the mechanism driving methods. These are to be used in an autonomous OpMode.

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
