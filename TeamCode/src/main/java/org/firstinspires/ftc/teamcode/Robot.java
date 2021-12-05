/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;


/** A finite state of the robot: keeps track of its state (motors and position) and what actions it is doing (enums)
 */
public class Robot {
    // TODO: merge mechanism-driving into this branch to make necessary changes for the new enum values.

    // Finite state machine
    public enum CarouselMotorState {CHECK_START, SPIN}
    // NOTE: changed from "extend"-based to "set"-based so that movement from any state to any other state is possible
    // during TeleOp without the intermediate step of fully retracting the slides, which was necessary in the previous
    // method. This will come in handy if the driver in TeleOp makes a mistake and raises the slides to the wrong level.
    public enum SlidesMotorsState {CHECK_SET_LEVEL, SET_L0, SET_L1, SET_L2, SET_L3, SET_L4}
    public enum ClawMotorState {CHECK_SET_POSITION, SET_OPEN, SET_CLOSE_CUBE, SET_CLOSE_SPHERE}

    public CarouselMotorState carouselMotorState;
    public SlidesMotorsState slidesMotorsState;
    public ClawMotorState clawMotorState;

    // Hardware
    public DcMotor carousel, slidesLeft, slidesRight, frontRightDrive, rearRightDrive, frontLeftDrive, rearLeftDrive;
    public Servo claw;

    // Positioning
    public PositionManager positionManager;

    public Robot(HardwareMap hardwareMap) {
        // Initialize states.
        carouselMotorState = CarouselMotorState.CHECK_START;
        slidesMotorsState = SlidesMotorsState.CHECK_SET_LEVEL;
        clawMotorState = ClawMotorState.CHECK_SET_POSITION;

        // Initialize hardware.
        carousel = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.CAROUSEL));
        slidesLeft = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_LEFT));
        slidesRight = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_RIGHT));
        frontRightDrive = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.FRONT_RIGHT_DRIVE));
        rearRightDrive = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.REAR_RIGHT_DRIVE));
        frontLeftDrive = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.FRONT_LEFT_DRIVE));
        rearLeftDrive = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.REAR_LEFT_DRIVE));
        claw = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.CLAW));

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize position manager.
        positionManager = new PositionManager();
    }
}


/** Maps the robot's hardware to their names in the OpMode configuration, and contains any other necessary constants
 *  pertaining to the robot's state.
 */
class RobotConfig {
    enum Motors {CAROUSEL, SLIDES_LEFT, SLIDES_RIGHT, REAR_LEFT_DRIVE,
                 REAR_RIGHT_DRIVE, FRONT_LEFT_DRIVE, FRONT_RIGHT_DRIVE}

    enum Servos {CLAW}

    public static final Map<Motors, String> MotorNames = new HashMap<Motors, String>() {{
        put(Motors.CAROUSEL, "carousel");
        put(Motors.SLIDES_LEFT, "slides_left");
        put(Motors.SLIDES_RIGHT, "slides_right");
        put(Motors.REAR_LEFT_DRIVE, "rear_left");
        put(Motors.REAR_RIGHT_DRIVE, "rear_right");
        put(Motors.FRONT_LEFT_DRIVE, "front_left");
        put(Motors.FRONT_RIGHT_DRIVE, "front_right");
    }};

    public static final Map<Servos, String> ServoNames = new HashMap<Servos, String>() {{ put(Servos.CLAW, "claw"); }};
}
