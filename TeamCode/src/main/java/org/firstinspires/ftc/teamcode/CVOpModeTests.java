package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class CVOpModeTests extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotManager robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, Navigation.NavigationMode.DUCK, Navigation.AllianceColor.BLUE);
        ComputerVision cv = robotManager.computerVision;

        telemetry.addLine("Waiting for start");
        telemetry.update();

        cv.startStreaming();
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Frame Count", cv.camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", cv.camera.getFps()));
            telemetry.addData("Total frame time ms", cv.camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", cv.camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", cv.camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", cv.camera.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}