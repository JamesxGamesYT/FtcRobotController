/* Authors: Nisky Robotics 6460 2021-2022 Programming Team
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/** Autonomous OpMode for Freight Frenzy. Uses a finite state machine.
 */
@TeleOp(name="Freight Frenzy Tele-Op", group="TeleOp OpMode")
public class FreightFrenzyTeleOp extends OpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void init() {
        robotManager = new RobotManager(hardwareMap, telemetry, elapsedTime, gamepad1, gamepad2,
                                        Navigation.NavigationMode.TELEOP, Navigation.AllianceColor.BLUE);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        robotManager.readControllerInputs();
        robotManager.driveMechanisms();
        robotManager.maneuver();
        robotManager.robot.positionManager.updatePosition(robotManager.robot);

        telemetry.addData("Pos X", robotManager.robot.positionManager.position.getX());
        telemetry.addData("Pos Y", robotManager.robot.positionManager.position.getY());
        telemetry.addData("Pos R", robotManager.robot.positionManager.position.getRotation());
        telemetry.update();
    }

    @Override
    public void stop() {}
}
