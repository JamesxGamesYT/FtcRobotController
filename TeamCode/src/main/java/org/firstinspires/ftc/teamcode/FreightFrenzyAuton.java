package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;

@Autonomous(name="FreightFrenzyAuton", group="Linear OpMode")
public class FreightFrenzyAuton extends LinearOpMode {

    private RobotManager robotManager;

    @Override
    public void runOpMode() {
        initSharedPreferences();
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, navigationMode, allianceColor, telemetry);

        waitForStart(); // wait for the play button to be pressed

        while (opModeIsActive()) { // loop this until stop button is pressed

        }
    }

    // ANDROID SHARED PREFERENCES
    // ==========================

    // I have no idea if this works. Adapted from https://github.com/ver09934/twentytwenty/blob/ian-dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SkystoneAuton.java

    private static SharedPreferences sharedPrefs;

    private static Navigation.AllianceColor allianceColor;
    private static Navigation.NavigationMode navigationMode;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String allianceColor = sharedPrefs.getString("alliance_color", "ERROR");
        String autonMode = sharedPrefs.getString("auton_mode", "ERROR");

        if (allianceColor.equals("BLUE")) {
            this.allianceColor = Navigation.AllianceColor.BLUE;
        }
        else if (allianceColor.equals("RED")) {
            this.allianceColor = Navigation.AllianceColor.RED;
        }

        if (autonMode.equals("DUCK")) {
            navigationMode = Navigation.NavigationMode.DUCK;
        }
        else if (autonMode.equals("FREIGHT")) {
            navigationMode = Navigation.NavigationMode.FREIGHT;
        }
    }
}
