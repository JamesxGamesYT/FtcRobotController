package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="claw test", group="Linear Opmode")//register this op mode in the op mode list on the phone
public class ClawTestOpMode extends LinearOpMode{
    @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            Robot robot = new Robot(hardwareMap, telemetry);
            MechanismDriving mechs=new MechanismDriving();

            waitForStart();//wait for the play button to be pressed

            while (opModeIsActive()) {//loop this until stop button is pressed
                mechs.updateClaw(robot);
                if(gamepad1.a ){
                    robot.desiredClawState = Robot.ClawState.OPEN;
                }

                if(gamepad1.b) {
                    robot.desiredClawState = Robot.ClawState.CUBE;
                }

//                telemetry.addData("Claw info", "State: " + robot.clawMotorState + "; Servo position: " + robot.claw.getPosition() + ";"); //Was there something about how a servo with a position could not be used here?
                telemetry.update();
            }
        }
}
