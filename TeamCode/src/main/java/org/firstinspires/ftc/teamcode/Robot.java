/* Authors: Arin Khare, Kai Vernooy
this is an unofficial version of this call meant for testing purposes only
disregard all changes in this version of this class when merging
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;


/** A finite state of the robot: keeps track of its state (motors and position) and what actions it is doing (enums)
 */
public class Robot {
    // Finite state machine
    enum CarouselMotorState {STOP, SPIN}
    enum SlidesMotorsState {CHECK_EXTEND, EXTEND_1, EXTEND_2, EXTEND_3, EXTEND_4, CHECK_RETRACT, RETRACT}
    enum ClawMotorState {CHECK_OPEN, OPEN, CHECK_CLOSE, CLOSE}

    public CarouselMotorState carouselMotorState;
    public SlidesMotorsState slidesMotorsState;
    public ClawMotorState clawMotorState;

    // Hardware
    public DcMotor carousel, slidesLeft, slidesRight, frontRightDrive, rearRightDrive, frontLeftDrive, rearLeftDrive;
    public CRServo claw;

    // Positioning
    public PositionManager position;

    public Robot(HardwareMap hardwareMap) {
        carouselMotorState=CarouselMotorState.STOP;
        slidesMotorsState=SlidesMotorsState.CHECK_EXTEND;
        clawMotorState=ClawMotorState.CHECK_OPEN;

        // Initialize hardware.
        carousel = hardwareMap.get(DcMotor.class,"carousel");
        slidesLeft = hardwareMap.get(DcMotor.class, "slides_left");
        slidesRight = hardwareMap.get(DcMotor.class, "slides_right");
        claw = hardwareMap.get(CRServo.class,"claw");

        slidesRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}


// TODO: Decide on whether or not we need this class.
//class RobotConfig {
//    enum Motors {CAROUSEL, SLIDES_LEFT, SLIDES_RIGHT, REAR_LEFT_DRIVE,
//                 REAR_RIGHT_DRIVE, FRONT_LEFT_DRIVE, FRONT_RIGHT_DRIVE}
//
//    public static Map<Motors, String> MotorNames = new HashMap<Motors, String>() {{
//        put(Motors.CAROUSEL, "carousel");
//        put(Motors.SLIDES_LEFT, "slides_left");
//        put(Motors.SLIDES_RIGHT, "slides_right");
//        put(Motors.REAR_LEFT_DRIVE, "rear_left");
//        put(Motors.REAR_RIGHT_DRIVE, "rear_right");
//        put(Motors.FRONT_LEFT_DRIVE, "front_left");
//        put(Motors.FRONT_RIGHT_DRIVE, "front_right");
//    }};
//}
