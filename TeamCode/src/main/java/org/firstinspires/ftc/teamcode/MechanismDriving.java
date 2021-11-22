package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {
    private DcMotor carousel,slidesLeft,slidesRight;
    private Servo claw;
    private boolean carouselActive=false;

    private int slidePosition;

    MechanismDriving(DcMotor carousel, DcMotor slidesLeft, DcMotor slidesRight, Servo clawServo){
        this.carousel=carousel;
        this.slidesLeft=slidesLeft;
        this.slidesRight=slidesRight;
        claw=clawServo;
    }

    /** Opens the claw all the way
     */
    public void openClaw(){
        claw.setPosition(1);
    }

    /**Opens the claw to a specified amount
     *
     * @param amount - Position of servo between 0 and 1
     */
    public void openClaw(double amount) {
        claw.setPosition(amount);
    }

    /**Closes the claw all the way
     *
     */
    public void closeClaw(){
        claw.setPosition(0);
    }

    /**Turn the carousel motor on until you turn it off
     *
     * @param active - Determines whether the carousel motor should be spinning
     */
    public void activateCarousel(boolean active){
        if(active){
            carousel.setPower(1);
        }else{
            carousel.setPower(0);
            //setMode maybe
        }

        carouselActive=active;
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(int position){
        slidePosition=position;
    }

    /** This function gets called repeatedly from the operation mode
     * Responsible for moving slides to preferred position
     */
    public void update(){
        //If the current position is less than desired position then move it up
        if(slidesRight.getCurrentPosition()<slidePosition){
            //Ensures that one motor does not go beyond the other too much
            if(slidesLeft.getCurrentPosition()==slidesRight.getCurrentPosition()){
                slidesLeft.setPower(0.5);
                slidesRight.setPower(0.5);
            } else if(slidesLeft.getCurrentPosition()>slidesRight.getCurrentPosition()){
                slidesLeft.setPower(0.25);
                slidesRight.setPower(0.5);
            } else if(slidesLeft.getCurrentPosition()<slidesRight.getCurrentPosition()){
                slidesLeft.setPower(0.5);
                slidesRight.setPower(0.25);
            }
        }
        //If the current position is above the current position, move these downwards
        if(slidesRight.getCurrentPosition()>slidePosition){
            //Ensures that one motor does not go beyond the other too much
            if(slidesLeft.getCurrentPosition()==slidesRight.getCurrentPosition()){
                slidesLeft.setPower(-0.5); //Go in the opposite direction
                slidesRight.setPower(-0.5);
            } else if(slidesLeft.getCurrentPosition()<slidesRight.getCurrentPosition()){
                slidesLeft.setPower(-0.25);
                slidesRight.setPower(-0.5);
            } else if(slidesLeft.getCurrentPosition()>slidesRight.getCurrentPosition()){
                slidesLeft.setPower(-0.5);
                slidesRight.setPower(-0.25);
            }
        }

        //Stop motors when we have reached the desired position
        if(slidesRight.getCurrentPosition()==slidePosition){
            slidesLeft.setPower(0);
            slidesRight.setPower(0);
        }
    }

}
