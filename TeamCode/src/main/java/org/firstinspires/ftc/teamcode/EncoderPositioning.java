package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.HashMap;

/** Estimates the robot's position based on the encoders on the drivetrain's motors. This will require a baseline
 *  position to add onto.
 */
public class EncoderPositioning {
    static int COUNTS_PER_ROTATION = 280;
    static int MAGICAL_FACTOR = 1;

    static HashMap<Robot.WheelConfiguration, Double> RollerAngles = new HashMap<Robot.WheelConfiguration, Double> (){{
        put(Robot.WheelConfiguration.FRONTRIGHT, Math.PI / 4.d);
        put(Robot.WheelConfiguration.FRONTLEFT, 3 * Math.PI / 4.d);
        put(Robot.WheelConfiguration.REARLEFT, Math.PI / 4.d);
        put(Robot.WheelConfiguration.REARRIGHT, 3 * Math.PI / 4.d);
    }};


    /** Checks the robot's encoders to get an estimate of the distance and direction traveled.
     *  @return a position in the form of a vector from the origin that can be added to an existing measurement
     */
    public void updateDeltaEstimate(Robot robot) {
        // Read encoder values and use them to create a Position that represents the robot's movement relative to the last time the encoders were read.
        // Reset encoder values to zero.
        // call submitEstimate
        // 280 encoder counts per revolution

//
//        double frontLeftDelta = 0, frontRightDelta = 0, rearRightDelta = 0, rearLeftDelta = 0;
//
//        for (HashMap.Entry<Robot.WheelConfiguration, Double> rollerAngle : RollerAngles.entrySet())
//        {
//
//        }



        double ADEncCount = robot.frontLeftDrive.getCurrentPosition() + robot.rearRightDrive.getCurrentPosition();
        double BCEncCount = robot.rearLeftDrive.getCurrentPosition() + robot.frontRightDrive.getCurrentPosition();

        double ADangle = Math.acos((2 * ADEncCount + Math.sqrt(8 - 4 * (ADEncCount * ADEncCount))) / 4);
        double BCangle = Math.acos((2 * BCEncCount + Math.sqrt(8 - 4 * (BCEncCount * BCEncCount))) / 4);

        double secondXcor = BCEncCount * Math.cos(BCangle);
        double secondYcor = BCEncCount * Math.sin(BCangle);

        double thirdXcor = secondXcor - ADEncCount * Math.cos(ADangle);
        double thirdYcor = secondYcor + ADEncCount * Math.sin(ADangle);
        double orientation = Math.atan(thirdXcor / thirdYcor);

        submitEstimate(robot, new Position(thirdXcor, thirdYcor, 0.0));
        resetEncoders(robot);
    }


    /** Updates the encoder's position estimate in the robot's PositionManager
     */
    private void submitEstimate(Robot robot, Position delta) {
        robot.positionManager.updateEncoderPosition(delta);
    }


    /** Resets the encoder values to zero.
     */
    private void resetEncoders(Robot robot) {
        robot.frontLeftDrive.setTargetPosition(0);
        robot.frontRightDrive.setTargetPosition(0);
        robot.rearLeftDrive.setTargetPosition(0);
        robot.rearRightDrive.setTargetPosition(0);
    }
}
