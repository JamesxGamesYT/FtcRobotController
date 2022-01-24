package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryWrapper {
    TelemetryWrapper(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Telemetry telemetry;

    public void updateTelemetry(Robot robot) {
        if (robot.barcodeScanResult != null) telemetry.addData("Scan result", robot.barcodeScanResult.name());
        else telemetry.addData("Scan result", "null");

        telemetry.addData("Angle", robot.getPosition().getRotation());
        telemetry.addData("X", robot.getPosition().getX());
        telemetry.addData("Y", robot.getPosition().getY());

        telemetry.update();
    }
}
