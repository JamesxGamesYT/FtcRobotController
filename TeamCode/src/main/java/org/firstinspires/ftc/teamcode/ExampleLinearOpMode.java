package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="op mode name here", group="Linear Opmode")//register this op mode in the op mode list on the phone
//@Disabled//don't include this line in your code
public class ExampleLinearOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {


        waitForStart();//wait for the play button to be pressed

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "rear_right");

        frontLeft.setPower(1.0);
        sleep(5000);
        frontLeft.setPower(0.0);

        frontRight.setPower(1.0);
        sleep(5000);
        frontRight.setPower(0.0);

        rearLeft.setPower(1.0);
        sleep(5000);
        rearLeft.setPower(0.0);

        rearRight.setPower(1.0);
        sleep(5000);
        rearRight.setPower(0.0);

        while (opModeIsActive()) {//loop this until stop button is pressed

        }
    }
}
