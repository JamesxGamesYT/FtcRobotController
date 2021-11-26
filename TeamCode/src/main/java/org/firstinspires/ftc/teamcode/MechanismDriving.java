package org.firstinspires.ftc.teamcode;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private boolean carouselActive = false;
    private int slidePosition;

    MechanismDriving() {}

    // Each of the following methods should use the current state to determine motor inputs, and change the state once a
    // task is complete. They should serve an analogous function to that of the switch statement in the second code
    // block of (https://gm0.org/en/latest/docs/software/finite-state-machines.html)

    /** Opens the claw all the way
     */
    public void openClaw(Robot robot) {
        robot.claw.setPosition(1);
    }

    /**Opens the claw to a specified amount
     *
     * @param amount - Position of servo between 0 and 1
     */
    public void openClaw(Robot robot, double amount) {
        robot.claw.setPosition(amount);
    }

    /** Closes the claw all the way
     */
    public void closeClaw(Robot robot) {
        robot.claw.setPosition(0);
    }

    /** Turn the carousel motor on until you turn it off
     *
     *  @param active - Determines whether the carousel motor should be spinning
     */
    public void activateCarousel(Robot robot, boolean active) {
        if (active) {
            robot.carousel.setPower(1);
        }
        else {
            robot.carousel.setPower(0);
            // setMode maybe
        }

        carouselActive = active;
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(Robot robot, int position) {
        slidePosition = position;
    }

    /** This function gets called repeatedly from the operation mode.
     *  Responsible for moving slides to preferred position.
     */
    public void update(Robot robot) {
        // If the current position is less than desired position then move it up
        if (robot.slidesRight.getCurrentPosition() < slidePosition) {
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
        if (robot.slidesRight.getCurrentPosition() > slidePosition) {
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
        if (robot.slidesRight.getCurrentPosition() == slidePosition) {
            robot.slidesLeft.setPower(0);
            robot.slidesRight.setPower(0);
        }
    }
}
