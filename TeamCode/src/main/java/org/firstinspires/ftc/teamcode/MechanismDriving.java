package org.firstinspires.ftc.teamcode;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private boolean carouselActive = false;
    private int slidePosition;
    //TODO get the exact values the slides will; need to move to inorder to be be at the correct levels for the shipping hub
    public static final int EXTEND1POS=0,EXTEND2POS=0,EXTEND3POS=0,EXTEND4POS=0;
    public static final double CLAW_CUBE_POS = 0.0, CLAW_SPHERE_POS = 0.0;
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
        switch(robot.carouselMotorState){//check the carousel motor state and then use the information to activate or deactivate it
            case CHECK_START:
                activateCarousel(robot,false);
                break;
            case SPIN:
                activateCarousel(robot,true);
                break;
        }

        switch (robot.clawMotorState){//check the claw motor state and move the claw accoringly
            case CHECK_OPEN:
            case CHECK_CLOSE:

                break;
            case OPEN:
                openClaw(robot);//TODO find the exact value the claw will need to be opened to
                robot.clawMotorState= Robot.ClawMotorState.CHECK_CLOSE;
                break;
            case CLOSE_CUBE:
                openClaw(robot, CLAW_CUBE_POS);
                robot.clawMotorState= Robot.ClawMotorState.CHECK_OPEN;
                break;
            case CLOSE_SPHERE:
                openClaw(robot, CLAW_SPHERE_POS);
                robot.clawMotorState= Robot.ClawMotorState.CHECK_OPEN;
                break;

        }
        switch(robot.slidesMotorsState){//check the state of the slide motors and tell them to move to that position
            case CHECK_EXTEND:
            case CHECK_RETRACT:

                break;
            case EXTEND_1:
                setSlidePosition(robot,EXTEND1POS);
                robot.slidesMotorsState= Robot.SlidesMotorsState.CHECK_SET_LEVEL;
                break;
            case EXTEND_2:
                setSlidePosition(robot,EXTEND2POS);
                robot.slidesMotorsState= Robot.SlidesMotorsState.CHECK_SET_LEVEL;
                break;
            case EXTEND_3:
                setSlidePosition(robot,EXTEND3POS);
                robot.slidesMotorsState= Robot.SlidesMotorsState.CHECK_SET_LEVEL;
                break;
            case EXTEND_4:
                setSlidePosition(robot,EXTEND4POS);
                robot.slidesMotorsState= Robot.SlidesMotorsState.CHECK_SET_LEVEL;
                break;
            case RETRACT:
                setSlidePosition(robot,0);
                robot.slidesMotorsState= Robot.SlidesMotorsState.CHECK_SET_LEVEL;
                break;
        }

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
