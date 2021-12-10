/* Authors: Nisky Robotics 6460 2021-2022 Programming Team
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/** Autonomous OpMode for Freight Frenzy. Uses a finite state machine.
 */
public class FreightFrenzyTeleOp extends OpMode {
    private RobotManager robotManager;

    @Override
    public void init() {
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }
}
