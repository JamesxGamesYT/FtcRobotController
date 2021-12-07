package org.firstinspires.ftc.teamcode;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private int desiredSlidePosition;

    // TODO: get the exact values the slides will need to move to inorder to be be at the correct levels for the shipping hub.
    //       get exact values for the claw as well when open, holding a sphere, and holding a cube
    public static final int RETRACTED_POS = 0, LEVEL1_POS = 1000, LEVEL2_POS = 2000, LEVEL3_POS = 3000, CAPPING_POS = 4000;
    public static final double CLAW_OPEN_POS = 0.0, CLAW_CUBE_POS = 0.0, CLAW_SPHERE_POS = 0.0;
    // How long the carousel motor must be spinning for in order to deliver the duck.
    public static final long DUCK_SPIN_TIME = 1000;  // Milliseconds
    // How long it takes for the claw servo to be guaranteed to have moved to its new position.
    public static final long CLAW_SERVO_TIME = 500;
    public static final int EPSILON = 30;  // slide encoder position tolerances

    MechanismDriving() {}

    // TODO: rewrite this class to deal with a continuous rotation servo

    /** Sets the claw position to the robot's desired state.
     */
    public void updateClaw(Robot robot) {
        switch (robot.desiredClawState) {
            case OPEN:
                robot.claw.setPosition(CLAW_OPEN_POS);
                break;
            case CUBE:
                robot.claw.setPosition(CLAW_CUBE_POS);
                break;
            case SPHERE:
                robot.claw.setPosition(CLAW_SPHERE_POS);
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
                robot.carousel.setPower(1);
                break;
        }
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(Robot robot, int position) {
        desiredSlidePosition = position;
    }

    /** Sets slide motor powers to move in direction of desired position, if necessary.
     *
     * @return whether the slides are in the desired position.
     */
    public boolean updateSlides(Robot robot) {

        switch(robot.desiredSlidesState){
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

        // If the current position is less than desired position then move it up
        if (desiredSlidePosition - robot.slidesRight.getCurrentPosition() > EPSILON) {
            // Ensures that one motor does not go beyond the other too much
            if (robot.slidesLeft.getCurrentPosition() == robot.slidesRight.getCurrentPosition()) {
                robot.slidesLeft.setPower(0.5);
                robot.slidesRight.setPower(0.5);
            }
            else if(robot.slidesLeft.getCurrentPosition() > robot.slidesRight.getCurrentPosition()) {
                robot.slidesLeft.setPower(0.25);
                robot.slidesRight.setPower(0.5);
            }
            else if(robot.slidesLeft.getCurrentPosition() < robot.slidesRight.getCurrentPosition()) {
                robot.slidesLeft.setPower(0.5);
                robot.slidesRight.setPower(0.25);
            }
        }

        // If the current position is above the current position, move these downwards
        if (robot.slidesRight.getCurrentPosition() - desiredSlidePosition > EPSILON) {
            // Ensures that one motor does not go beyond the other too much
            if (robot.slidesLeft.getCurrentPosition() == robot.slidesRight.getCurrentPosition()) {
                robot.slidesLeft.setPower(-0.5); // Go in the opposite direction
                robot.slidesRight.setPower(-0.5);
            }
            else if (robot.slidesLeft.getCurrentPosition() < robot.slidesRight.getCurrentPosition()) {
                robot.slidesLeft.setPower(-0.25);
                robot.slidesRight.setPower(-0.5);
            }
            else if (robot.slidesLeft.getCurrentPosition() > robot.slidesRight.getCurrentPosition()) {
                robot.slidesLeft.setPower(-0.5);
                robot.slidesRight.setPower(-0.25);
            }
        }

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
}
