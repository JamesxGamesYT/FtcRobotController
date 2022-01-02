/* Authors: Nisky Robotics 6460 2021-2022 Programming Team
 */

package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.logging.Logger;


/** Autonomous OpMode for Freight Frenzy. Uses a finite state machine.
 */
@TeleOp(name="Freight Frenzy Tele-Op", group="TeleOp OpMode")
public class FreightFrenzyTeleOp extends OpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private static final Logger LOGGER = Logger.getLogger(FreightFrenzyTeleOp.class.getName());

    @Override
    public void init() {
        initSharedPreferences();
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, RobotManager.NavigationMode.TELEOP,
                                        allianceColor, telemetry, elapsedTime);
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

    // ANDROID SHARED PREFERENCES
    // ==========================

    // I have no idea if this works. Adapted from https://github.com/ver09934/twentytwenty/blob/ian-dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SkystoneAuton.java

    private static SharedPreferences sharedPrefs;

    private static RobotManager.AllianceColor allianceColor;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String allianceColor = sharedPrefs.getString("alliance_color", "ERROR");

        if (allianceColor.equals("BLUE")) {
            FreightFrenzyTeleOp.allianceColor = RobotManager.AllianceColor.BLUE;
        }
        else if (allianceColor.equals("RED")) {
            FreightFrenzyTeleOp.allianceColor = RobotManager.AllianceColor.RED;
        }
    }
}
