package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="claw test", group="Linear Opmode")//register this op mode in the op mode list on the phone
public class ClawTestOpMode extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            Robot robot = new Robot(hardwareMap, telemetry,runtime);
            MechanismDriving mechs=new MechanismDriving();

            waitForStart();//wait for the play button to be pressed

            while (opModeIsActive()) {//loop this until stop button is pressed
                mechs.updateClaw(robot);
                if(gamepad1.a ){
                    robot.desiredClawState = Robot.ClawState.CLOSED;
                }

                if(gamepad1.b) {
                    robot.desiredClawState = Robot.ClawState.OPEN;
                }

//                telemetry.addData("Claw info", "State: " + robot.clawMotorState + "; Servo position: " + robot.claw.getPosition() + ";"); //Was there something about how a servo with a position could not be used here?
                telemetry.update();
            }
        }
}
