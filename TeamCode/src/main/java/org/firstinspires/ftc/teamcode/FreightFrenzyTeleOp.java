/* Authors: Nisky Robotics 6460 2021-2022 Programming Team
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;


/** Autonomous OpMode for Freight Frenzy. Uses a finite state machine.
 */
@TeleOp(name="Autonomous", group="Iterative Opmode")
public class FreightFrenzyTeleOp extends OpMode {
    private RobotManager robotManager;

    @Override
    public void init() {
    }

    @Override
    public void start() {
        robotManager = new RobotManager(hardwareMap);
    }

    @Override
    public void loop() {
        robotManager.readControllerInputs(gamepad1, gamepad2);
        robotManager.driveMechanisms();
        robotManager.maneuver(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
    }

    @Override
    public void stop() {

    }
}
