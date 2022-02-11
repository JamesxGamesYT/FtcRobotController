package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;

//@Disabled
@TeleOp
public class CVTestOpMode extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
//        DcMotor clawLEDs=hardwareMap.get(DcMotor.class,"LED");
//        clawLEDs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        clawLEDs.setDirection(DcMotorSimple.Direction.FORWARD);
//        clawLEDs.setPower(1.0);
//        while (opModeIsActive()) {
//
//        }
        RobotManager robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, new ArrayList<>(Collections.emptyList()),
                                                     RobotManager.AllianceColor.BLUE, RobotManager.StartingSide.CAROUSEL, Navigation.MovementMode.STRAFE, telemetry, elapsedTime);
        ComputerVision cv = robotManager.computerVision;
//        cv.startStreaming();
        waitForStart();
//        Robot.SlidesState level = robotManager.readBarcode();
//        waitForStart();
//
        cv.startStreaming();
////        Robot.SlidesState result = robotManager.readBarcode();
//
//        IMUPositioning.Initialize(this);
//
        while (opModeIsActive()) {
//            telemetry.addData("result", result.name());
//            telemetry.addData("Barcode frequencies", robotManager.robot.barcodeScanResultMap.toString());

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