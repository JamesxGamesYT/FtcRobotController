/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;


/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    // Robot desired states.
    public enum CarouselState {STOPPED, SPINNING}
    public enum SlidesState {RETRACTED, L1, L2, L3, CAPPING}
    public enum ClawState {OPEN, SPHERE, CUBE}

    public CarouselState desiredCarouselState;
    public SlidesState desiredSlidesState;
    public ClawState desiredClawState;

    enum BarcodeScanState {CHECK_SCAN, SCAN}
    public BarcodeScanState barcodeScanState;

    static final int MaxBarcodeAttempts = 100;   // How many times to try scanning the barcode before giving up
    int numBarcodeAttempts = 0;                  // Amount of current attempts to scan the barcode
    int barcodeScanResult = -1;                  // Represents the barcode state, with 0 being left, 1 being middle, 2 being right, and -1 being no result.

    boolean fineMovement = false;
    boolean fineRotation = false;

    HashMap<RobotConfig.DriveMotors, DcMotor> driveMotors = new HashMap<RobotConfig.DriveMotors, DcMotor>();

    // Hardware
    public DcMotor carousel, slidesLeft, slidesRight;
    public Servo claw;

    // Other
    public Telemetry telemetry;
    public ElapsedTime elapsedTime;

    // Positioning
    public PositionManager positionManager;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.telemetry = telemetry;
        this.elapsedTime = elapsedTime;
        positionManager = new PositionManager();

        // Initialize desired states.
        desiredCarouselState = CarouselState.STOPPED;
        desiredSlidesState = SlidesState.RETRACTED;
        desiredClawState = ClawState.OPEN;

        // Initialize hardware.
        carousel = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.CAROUSEL));
        slidesLeft = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_LEFT));
        slidesRight = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_RIGHT));
        claw = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.CLAW));

        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            driveMotors.put(motor, hardwareMap.get(DcMotor.class, RobotConfig.DriveMotorNames.get(motor)));
            driveMotors.get(motor).setDirection(RobotConfig.DriveMotorsDirections.get(motor));
            driveMotors.get(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors.get(motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotors.get(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    enum Motors {CAROUSEL, SLIDES_LEFT, SLIDES_RIGHT}
    public enum DriveMotors {REAR_LEFT, REAR_RIGHT, FRONT_LEFT, FRONT_RIGHT};
    enum Servos {CLAW}

    public static final Map<Motors, String> MotorNames = new HashMap<Motors, String>() {{
        put(Motors.CAROUSEL, "carousel");
        put(Motors.SLIDES_LEFT, "slides_left");
        put(Motors.SLIDES_RIGHT, "slides_right");
    }};

    public static final Map<DriveMotors, String> DriveMotorNames = new HashMap<DriveMotors, String>() {{
        put(DriveMotors.REAR_LEFT, "rear_left");
        put(DriveMotors.REAR_RIGHT, "rear_right");
        put(DriveMotors.FRONT_LEFT, "front_left");
        put(DriveMotors.FRONT_RIGHT, "front_right");
    }};

    public static final Map<DriveMotors, DcMotor.Direction> DriveMotorsDirections = new HashMap<DriveMotors, DcMotor.Direction>() {{
        put(DriveMotors.FRONT_LEFT, DcMotor.Direction.FORWARD);
        put(DriveMotors.REAR_LEFT, DcMotor.Direction.FORWARD);
        put(DriveMotors.FRONT_RIGHT, DcMotor.Direction.REVERSE);
        put(DriveMotors.REAR_RIGHT, DcMotor.Direction.REVERSE);
    }};

    public static final Map<Servos, String> ServoNames = new HashMap<Servos, String>() {{ put(Servos.CLAW, "claw"); }};
}
