/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


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
    public Servo claw;

    // Positioning
    public PositionManager position;

    public Robot(HardwareMap hardwareMap) {}
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
