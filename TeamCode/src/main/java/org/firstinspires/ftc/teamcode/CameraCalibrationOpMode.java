package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class CameraCalibrationOpMode extends LinearOpMode {
    ComputerVision cv = new ComputerVision(hardwareMap, new CalibrationPipeline());

    @Override
    public void runOpMode() {

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        cv.startStreaming();


        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}