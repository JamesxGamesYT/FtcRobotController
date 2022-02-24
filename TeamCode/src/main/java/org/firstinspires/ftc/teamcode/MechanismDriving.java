package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.Range;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private static int desiredSlidePosition;

    public static final int RETRACTED_POS = 0, LEVEL1_POS = 1500, LEVEL2_POS = 3500, LEVEL3_POS = 6000, CAPPING_POS = 8000;
    public static final double CLAW_CLOSED_POS = 100.0, CLAW_OPEN_POS = -0.4; //These are not final values
    // How long it takes for the claw servo to be guaranteed to have moved to its new position.
    public static final long CLAW_SERVO_TIME = 500;
    public static final int EPSILON = 30;  // slide encoder position tolerances
    double slideRampDownDist=1000, maxSpeedCoefficient =0.8, reducedSpeedCoefficient =0.7;
    public static final double CAROUSEL_SPEED = 0.625;

    public double carouselStartTime = 0.0;
    public static final double CAROUSEL_PAUSED_TIME = 1000;
    public static final double CAROUSEL_SPIN_TIME = 2000;

    public static final int slidesAdjustmentSpeed = 2;

    private double carouselPower = CAROUSEL_SPEED;

    MechanismDriving(RobotManager.AllianceColor allianceColor) {
        if (allianceColor == RobotManager.AllianceColor.BLUE) {
            carouselPower = -CAROUSEL_SPEED;
        }
    }

    /** Sets the claw position to the robot's desired state.
     */
    public void updateClaw(Robot robot) {
        switch (robot.desiredClawState) {
            case CLOSED:
                robot.claw.setPower(CLAW_CLOSED_POS);//closed
//                robot.clawLEDs.setPower(0);
                robot.clawIndicator.setPosition(0);
                break;
            case OPEN:
                robot.claw.setPower(CLAW_OPEN_POS);//open
//                robot.clawLEDs.setPower(1);
                robot.clawIndicator.setPosition(1);
                break;
        }
    }

    /** Activates or stops carousel depending on robot's desired state.
     */
    public void updateCarousel(Robot robot) {
        switch(robot.desiredCarouselState) {
            case STOPPED:
                robot.carousel.setPower(0);
                break;
            case SPINNING:
                robot.carousel.setPower(carouselPower);
                break;
            case AUTO_SPIN:
                double carouselTime = robot.elapsedTime.milliseconds() - carouselStartTime;
                if (carouselTime % (CAROUSEL_PAUSED_TIME + CAROUSEL_SPIN_TIME) < CAROUSEL_SPIN_TIME)
                    robot.carousel.setPower(carouselPower);
                else
                    robot.carousel.setPower(0);
        }
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(Robot robot, int position) {
        desiredSlidePosition = position;
    }

    /** Sets desired states of slide motor powers using joystick.
     *
     * TODO: implement this
     */
    public void adjustDesiredSlideHeight(AnalogValues analogValues, Robot robot) {
//        if (analogValues.gamepad2LeftStickY < -RobotManager.JOYSTICK_DEAD_ZONE_SIZE) {
//            Robot.desiredSlidesState
//        }
    }

    /** Sets slide motor powers to move in direction of desired position, if necessary.
     *
     * @return whether the slides are in the desired position.
     */
    public boolean updateSlides(Robot robot) {

        if(Robot.desiredSlidesState != Robot.SlidesState.UNREADY){
            switch(Robot.desiredSlidesState){
                case RETRACTED:
                    setSlidePosition(robot, RETRACTED_POS);
                    break;
                case L1:
                    setSlidePosition(robot, LEVEL1_POS);
                    break;
                case L2:
                    setSlidePosition(robot, LEVEL2_POS);
                    break;
                case L3:
                    setSlidePosition(robot, LEVEL3_POS);
                    break;
                case CAPPING:
                    setSlidePosition(robot, CAPPING_POS);
                    break;
            }

            double mainSpeed,reducedSpeed;//"ramp" the motor speeds down based on how far away from the destination the motors are
            mainSpeed= maxSpeedCoefficient *Range.clip(Math.abs(desiredSlidePosition - robot.slidesRight.getCurrentPosition())/slideRampDownDist, 0.1, 1);
            reducedSpeed= reducedSpeedCoefficient *Range.clip(Math.abs(desiredSlidePosition - robot.slidesRight.getCurrentPosition())/slideRampDownDist, 0.1, 1);
            mainSpeed=Range.clip(mainSpeed,0.05,1);//limit the max speed to 1 and the min speed to 0.05
            reducedSpeed=Range.clip(reducedSpeed,0.04,1);

            // If the current position is less than desired position then move it up
            if (desiredSlidePosition - robot.slidesRight.getCurrentPosition() > EPSILON) {
                // Ensures that one motor does not go beyond the other too much
                if (robot.slidesLeft.getCurrentPosition() == robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(mainSpeed);
                    robot.slidesRight.setPower(mainSpeed);
                }
                else if(robot.slidesLeft.getCurrentPosition() > robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(reducedSpeed);
                    robot.slidesRight.setPower(mainSpeed);
                }
                else if(robot.slidesLeft.getCurrentPosition() < robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(mainSpeed);
                    robot.slidesRight.setPower(reducedSpeed);
                }
            }

            // If the current position is above the current position, move these downwards
            if (robot.slidesRight.getCurrentPosition() - desiredSlidePosition > EPSILON) {
                // Ensures that one motor does not go beyond the other too much
                if (robot.slidesLeft.getCurrentPosition() == robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(-mainSpeed); // Go in the opposite direction
                    robot.slidesRight.setPower(-mainSpeed);
                }
                else if (robot.slidesLeft.getCurrentPosition() < robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(-reducedSpeed);
                    robot.slidesRight.setPower(-mainSpeed);
                }
                else if (robot.slidesLeft.getCurrentPosition() > robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(-mainSpeed);
                    robot.slidesRight.setPower(-reducedSpeed);
                }
            }
//            robot.telemetry.addData("slides: target: ",desiredSlidePosition+" current pos right: "+robot.slidesRight.getCurrentPosition()+ " current pos left: "+robot.slidesLeft.getCurrentPosition());
            // Stop motors when we have reached the desired position
            if (Math.abs(robot.slidesRight.getCurrentPosition() - desiredSlidePosition) < EPSILON) {
                robot.slidesLeft.setPower(0);
                robot.slidesRight.setPower(0);
                return true;
            }
            else {
                return false;
            }
        }
        return false;
    }
}
