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
    // Finite state machine
    enum CarouselMotorState {CHECK_START, SPIN}
    enum SlidesMotorsState {CHECK_EXTEND, EXTEND_1, EXTEND_2, EXTEND_3, EXTEND_4, CHECK_RETRACT, RETRACT}
    enum ClawMotorState {CHECK_OPEN, OPEN, CHECK_CLOSE, CLOSE}

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
        slidesMotorsState = SlidesMotorsState.CHECK_EXTEND;
        clawMotorState = ClawMotorState.CHECK_CLOSE;

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


class RobotConfig {
    enum Motors {CAROUSEL, SLIDES_LEFT, SLIDES_RIGHT, REAR_LEFT_DRIVE,
                 REAR_RIGHT_DRIVE, FRONT_LEFT_DRIVE, FRONT_RIGHT_DRIVE}

    enum Servos {CLAW}

    public static Map<Motors, String> MotorNames = new HashMap<Motors, String>() {{
        put(Motors.CAROUSEL, "carousel");
        put(Motors.SLIDES_LEFT, "slides_left");
        put(Motors.SLIDES_RIGHT, "slides_right");
        put(Motors.REAR_LEFT_DRIVE, "rear_left");
        put(Motors.REAR_RIGHT_DRIVE, "rear_right");
        put(Motors.FRONT_LEFT_DRIVE, "front_left");
        put(Motors.FRONT_RIGHT_DRIVE, "front_right");
    }};

    public static Map<Servos, String> ServoNames = new HashMap<Servos, String>() {{ put(Servos.CLAW, "claw"); }};
}
