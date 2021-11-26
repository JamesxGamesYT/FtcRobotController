package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="op mode name here", group="Linear Opmode")//register this op mode in the op mode list on the phone
@Disabled//don't include this line in your code
public class ExampleLinearOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {


        waitForStart();//wait for the play button to be pressed

        while (opModeIsActive()) {//loop this until stop button is pressed

        }
    }
}
