package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.os.Environment;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;


@TeleOp
public class CameraCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        ComputerVision cv = new ComputerVision(hardwareMap, new CalibrationPipeline());

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        cv.startStreaming();


        CalibrationPipeline pl = ((CalibrationPipeline)cv.pipeline);

        while (opModeIsActive()) {
            if (pl.calibrated) {
                // write matrices to file
                try (FileOutputStream stream = new FileOutputStream(Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/navres/calibration.txt")) {
                    for (int i = 0; i < pl.cameraMatrix.rows(); i++) {
                        for (int j = 0; j < pl.cameraMatrix.cols(); j++) {
                            for (double x : pl.cameraMatrix.get(i, j)) {
                                stream.write(String.valueOf(x).getBytes());
                                stream.write(" ".getBytes());
                            }
                        }
                        stream.write("\n".getBytes());
                    }
                } catch (IOException e) {}

                break;
            }

            telemetry.update();
        }
    }
}