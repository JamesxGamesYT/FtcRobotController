/* Author: Arin Khare
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

/** Wraps a gamepad so that button mappings are stored in one place.
 */
public class GamepadWrapper {
    public enum DriverAction {START_STOP_CAROUSEL, SET_SLIDES_L0, SET_SLIDES_L1, SET_SLIDES_L2, SET_SLIDES_L3,
                              SET_SLIDES_L4, OPEN_CLAW, CLOSE_CLAW_SPHERE, CLOSE_CLAW_CUBE}

    Gamepad gamepad1, gamepad2;
    boolean firstIteration;

    public GamepadWrapper(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        firstIteration = true;
    }

    public GamepadWrapper() {
        this.gamepad1 = new Gamepad();
        this.gamepad2 = new Gamepad();
        firstIteration = true;
    }

    public void copyGamepad1(Gamepad gamepad) {
        try {
            this.gamepad1.copy(gamepad);
            firstIteration = false;
        } catch (RobotCoreException e) {}
    }

    public void copyGamepad2(Gamepad gamepad) {
        try {
            this.gamepad2.copy(gamepad);
            firstIteration = false;
        } catch (RobotCoreException e) {}
    }

    /** Returns the state of a button (true/false).
     *
     *  This is essentially where the button mapping is stored. Assumes gamepad2 exclusively is used for mechanism
     *  driving.
     */
    public boolean getButtonState(DriverAction driverAction) {
        switch (driverAction) {
            case START_STOP_CAROUSEL:
                return gamepad2.a;
            case SET_SLIDES_L0:
                return gamepad2.dpad_down;
            case SET_SLIDES_L1:
                return gamepad2.dpad_left;
            case SET_SLIDES_L2:
                return gamepad2.dpad_right;
            case SET_SLIDES_L3:
                return gamepad2.dpad_up;
            case SET_SLIDES_L4:
                return gamepad2.right_bumper;
            case OPEN_CLAW:
                return gamepad2.b;
            case CLOSE_CLAW_SPHERE:
                return gamepad2.x;
            case CLOSE_CLAW_CUBE:
                return gamepad2.y;
        }
        return false;  // Is this good practice? I just put it here because IntelliJ was upset.
    }

    /** Returns the x and y coordinates of each of the 4 joysticks.
     */
    public JoystickValues getJoystickValues() {
        return new JoystickValues(gamepad1, gamepad2);
    }
}


/** Stores 8 joystick values (an x and y coordinate for each of 4 sticks across 2 gamepads).
 */
class JoystickValues {
    double gamepad1RightStickX, gamepad1RightStickY, gamepad1LeftStickX, gamepad1LeftStickY,
           gamepad2RightStickX, gamepad2RightStickY, gamepad2LeftStickX, gamepad2LeftStickY;

    public JoystickValues(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1RightStickX = gamepad1.right_stick_x;
        this.gamepad1RightStickY = gamepad1.right_stick_y;
        this.gamepad1LeftStickX = gamepad1.left_stick_x;
        this.gamepad1LeftStickY = gamepad1.left_stick_y;
        this.gamepad2RightStickX = gamepad2.right_stick_x;
        this.gamepad2RightStickY = gamepad2.right_stick_y;
        this.gamepad2LeftStickX = gamepad2.left_stick_x;
        this.gamepad2LeftStickY = gamepad2.left_stick_y;
    }
}