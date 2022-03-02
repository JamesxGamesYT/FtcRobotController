package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class CameraCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
//        ComputerVision cv = new ComputerVision(hardwareMap, new CalibrationPipeline());

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

//        cv.startStreaming();
//        CalibrationPipeline pl = (CalibrationPipeline) cv.pipeline;
//
//
//        while (opModeIsActive()) {
//            if (pl.calibrated) {
//                CalibrationPipeline.MatToFile(pl.cameraMatrix,  ComputerVision.DataDir + "/camera-matrix.json");
//                CalibrationPipeline.MatToFile(pl.distortionMatrix,  ComputerVision.DataDir + "/distortion-matrix.json");
//            }
//
//            telemetry.update();
//        }
    }
}
