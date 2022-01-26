package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name="FreightFrenzyAuton", group="Linear OpMode")
public class FreightFrenzyAuton extends LinearOpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initSharedPreferences();
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, AutonomousPaths.PRELOAD_BOX_ONLY,
                                        FreightFrenzyAuton.allianceColor, telemetry, elapsedTime);

        IMUPositioning.Initialize(this);
        robotManager.computerVision.startStreaming();

//        while (!isStarted()) {
//         Warning: the following is blocking; it can probably be made non-blocking, if necessary
        Robot.SlidesState hubLevel = robotManager.readBarcode();
        robotManager.computerVision.stopStreaming();
//       }

        telemetry.addData("level", hubLevel.name());
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart(); // Wait for the play button to be pressed

        robotManager.closeClaw();
        robotManager.travelToNextPOI();  // Go to alliance shipping hub.
        robotManager.deliverToShippingHub(hubLevel);

//        if (navigationMode == RobotManager.NavigationMode.DUCK_CAROUSEL || navigationMode == RobotManager.NavigationMode.DUCK_WAREHOUSE) {
//            robotManager.travelToNextPOI();  // Go to carousel.
//            robotManager.deliverDuck();
//            robotManager.travelToNextPOI();  // Park in alliance storage unit.
//        }
//        else {
//            robotManager.travelToNextPOI();  // Park in warehouse.
//        }

        while (opModeIsActive()) {}
    }

    // ANDROID SHARED PREFERENCES
    // ==========================

    // Adapted from https://github.com/ver09934/twentytwenty/blob/ian-dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SkystoneAuton.java

    private static SharedPreferences sharedPrefs;

    private static RobotManager.AllianceColor allianceColor;
    private static ArrayList<Position> navigationPath;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String allianceColor = sharedPrefs.getString("alliance_color", "ERROR");
        String autonMode = sharedPrefs.getString("auton_mode", "ERROR");

        if (allianceColor.equals("BLUE")) {
            FreightFrenzyAuton.allianceColor = RobotManager.AllianceColor.BLUE;
        }
        else if (allianceColor.equals("RED")) {
            FreightFrenzyAuton.allianceColor = RobotManager.AllianceColor.RED;
        }

        switch (autonMode) {
            case "DUCK_CAROUSEL":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.DUCK_CAROUSEL_PATH.clone();
                break;
            case "DUCK_WAREHOUSE":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.DUCK_WAREHOUSE_PATH.clone();
                break;
            case "NO_DUCK_CAROUSEL":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.NO_DUCK_CAROUSEL_PATH.clone();
                break;
            case "NO_DUCK_WAREHOUSE":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.NO_DUCK_WAREHOUSE_PATH.clone();
                break;
            case "PRELOAD_BOX_ONLY":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.PRELOAD_BOX_ONLY.clone();
                break;
            case "PRELOAD_BOX_AND_PARK":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.PRELOAD_BOX_AND_PARK.clone();
                break;
            case "PARK":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.PARK.clone();
                break;
            case "MOVE_STRAIGHT":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.MOVE_STRAIGHT.clone();
                break;
            case "ROTATE_180":
                FreightFrenzyAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.ROTATE_180.clone();
                break;
        }
    }
}
