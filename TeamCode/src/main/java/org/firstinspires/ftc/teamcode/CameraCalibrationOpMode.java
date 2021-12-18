package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class CameraCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        ComputerVision cv = new ComputerVision(hardwareMap, new CalibrationPipeline());

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        cv.startStreaming();


        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}