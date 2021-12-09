/* Authors: Arin Khare, Kai Vernooy
this is an unofficial version of this call meant for testing purposes only
disregard all changes in this version of this class when merging
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;


/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    // Finite state machine
    public enum CarouselState {STOPPED, SPINNING}
    public enum SlidesState {RETRACTED, L1, L2, L3, CAPPING}
    public enum ClawState {OPEN, SPHERE, CUBE}

    public CarouselState desiredCarouselState;
    public SlidesState desiredSlidesState;
    public ClawState desiredClawState;

    // Hardware
    public DcMotor carousel, slidesLeft, slidesRight, frontRightDrive, rearRightDrive, frontLeftDrive, rearLeftDrive;
    public Servo claw;

    // Other
    public Telemetry telemetry;

    // Positioning
    public PositionManager positionManager;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        positionManager = new PositionManager();

        // Initialize desired states.
        desiredCarouselState = CarouselState.STOPPED;
        desiredSlidesState = SlidesState.RETRACTED;
        desiredClawState = ClawState.OPEN;

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

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Returns the position of the robot.
     */
    public Position getPosition() {
        return positionManager.position;
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
