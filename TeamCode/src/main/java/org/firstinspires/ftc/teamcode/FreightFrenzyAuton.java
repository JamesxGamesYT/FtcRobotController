package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="FreightFrenzyAuton", group="Linear OpMode")
public class FreightFrenzyAuton extends LinearOpMode {

    private DcMotor carousel;

    @Override
    public void runOpMode() {
        carousel = hardwareMap.get(DcMotor.class, "carousl");

        waitForStart(); // wait for the play button to be pressed

        carousel.setPower(0.3);
        while (opModeIsActive()) { // loop this until stop button is pressed
            sleep(1);
        }
    }

}
