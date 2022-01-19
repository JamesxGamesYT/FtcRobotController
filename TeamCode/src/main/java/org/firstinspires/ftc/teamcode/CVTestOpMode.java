package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class CVTestOpMode extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        RobotManager robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, RobotManager.NavigationMode.DUCK_WAREHOUSE,
                                                     RobotManager.AllianceColor.BLUE, telemetry, elapsedTime);
        ComputerVision cv = robotManager.computerVision;
        cv.startStreaming();

        waitForStart();

        Robot.SlidesState result = robotManager.readBarcode();


        while (opModeIsActive()) {
            telemetry.addData("result", result.name());
            telemetry.addData("Barcode frequencies", robotManager.robot.barcodeScanResultMap.toString());

//            telemetry.addData("Frame Count", cv.camera.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", cv.camera.getFps()));
//            telemetry.addData("Total frame time ms", cv.camera.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", cv.camera.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", cv.camera.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", cv.camera.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}