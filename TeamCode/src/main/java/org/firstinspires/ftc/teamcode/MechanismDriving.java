package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    //Opens the claw all the way
    public void openClaw(){
        claw.setPosition(1);
    }

    //Opens the claw to a specified amount
    public void openClaw(double amount) {
        claw.setPosition(amount);
    }

    //Closes the claw all the way
    public void closeClaw(){
        claw.setPosition(0);
    }

    //Turn the carousel motor on until you turn it off
    public void activateCarousel(boolean active){
        if(active){
            carousel.setPower(1);
        }else{
            carousel.setPower(0);
            //setMode maybe
        }

        carouselActive=active;
    }
    public void setSlidePosition(int position){
        slidePosition=position;
    }

    //this functoin gets called repeatedly
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
