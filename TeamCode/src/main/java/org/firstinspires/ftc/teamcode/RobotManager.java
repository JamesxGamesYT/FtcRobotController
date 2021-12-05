/* Authors: Arin Khare, Kai Vernooy
 */

// TODO: claw should have similar system to linear slides with setting the position to a particular value rather than
//       opening and closing.
// TODO: test if servo setPosition is blocking or non-blocking because that changes things with the FSM

package org.firstinspires.ftc.teamcode;

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
            previousStateGamepads.copyGamepad1(gamepads.gamepad1);
            previousStateGamepads.copyGamepad2(gamepads.gamepad2);
            return;
        }

        // Detect button releases.

        // Carousel
        if (getButtonRelease(GamepadWrapper.DriverAction.START_STOP_CAROUSEL)) {
            switch (robot.desiredCarouselState) {
                case STOPPED:
                    robot.desiredCarouselState = Robot.CarouselState.SPINNING;
                    break;
                case SPINNING:
                    robot.desiredCarouselState = Robot.CarouselState.STOPPED;
                    break;
            }
        }

        // Claw
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_CLAW_CUBE)) {
            robot.desiredClawState = Robot.ClawState.CUBE;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_CLAW_SPHERE)) {
            robot.desiredClawState = Robot.ClawState.SPHERE;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.OPEN_CLAW)) {
            robot.desiredClawState = Robot.ClawState.OPEN;
        }

        // Linear slides
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_RETRACTED)) {
            robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_L1)) {
            robot.desiredSlidesState = Robot.SlidesState.L1;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_L2)) {
            robot.desiredSlidesState = Robot.SlidesState.L2;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_L3)) {
            robot.desiredSlidesState = Robot.SlidesState.L3;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_CAPPING)) {
            robot.desiredSlidesState = Robot.SlidesState.CAPPING;
        }

        previousStateGamepads.copyGamepad1(gamepads.gamepad1);
        previousStateGamepads.copyGamepad2(gamepads.gamepad2);
    }

    /** Calls all non-blocking FSM methods to read from state and act accordingly.
     */
    public void driveMechanisms() {
        mechanismDriving.updateCarousel(robot);
        mechanismDriving.updateClaw(robot);
        mechanismDriving.updateSlides(robot);
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     * @param leftStickX the left stick x-axis
     * @param leftStickY the left stick y-axis
     * @param rightStickX the right stick x-axis
     */
    public void maneuver(double leftStickX, double leftStickY, double rightStickX) {}

    /** Determines whether the button for a particular action was released in the current OpMode iteration.
     */
    private boolean getButtonRelease(GamepadWrapper.DriverAction action) {
        return !gamepads.getButtonState(action) && previousStateGamepads.getButtonState(action);
    }

    // AUTONOMOUS
    // ==========

    /** Moves the robot to the next point of interest.
     */
    public void travelToNextPOI() {}

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
