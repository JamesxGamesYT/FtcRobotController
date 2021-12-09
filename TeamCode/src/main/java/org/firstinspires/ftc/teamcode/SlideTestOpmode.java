package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="slide test", group="Linear Opmode")//register this op mode in the op mode list on the phone
public class SlideTestOpmode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Robot robot = new Robot(hardwareMap, telemetry);
        MechanismDriving mechs=new MechanismDriving();

        waitForStart();//wait for the play button to be pressed

        while (opModeIsActive()) {//loop this until stop button is pressed
            mechs.updateSlides(robot);
            if(gamepad1.a){
                robot.desiredSlidesState= Robot.SlidesState.RETRACTED;
            }
            if(gamepad1.right_bumper){
                robot.desiredSlidesState= Robot.SlidesState.L1;
            }
            if(gamepad1.left_trigger!=0){
                robot.desiredSlidesState= Robot.SlidesState.L2;
            }
            if(gamepad1.right_trigger!=0){
                robot.desiredSlidesState= Robot.SlidesState.L3;
            }
            if(gamepad1.left_bumper){
                robot.desiredSlidesState= Robot.SlidesState.CAPPING;
            }

            telemetry.addData("encoders", "carousel "+robot.carousel.getCurrentPosition()+" slidesLeft "+robot.slidesLeft.getCurrentPosition()+" slidesRight "+robot.slidesRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
