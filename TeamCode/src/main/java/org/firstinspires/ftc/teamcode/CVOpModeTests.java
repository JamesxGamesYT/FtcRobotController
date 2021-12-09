/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp
public class CVOpModeTests extends LinearOpMode {

    CVPositioning cvPositioning;

    @Override
    public void runOpMode() {
        RobotManager robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, Navigation.NavigationMode.DUCK,
                Navigation.AllianceColor.BLUE, telemetry);

        cvPositioning = new CVPositioning(hardwareMap);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        cvPositioning.startStreaming(robotManager.robot);



        while (opModeIsActive())
        {
            telemetry.addData("Frame Count", cvPositioning.camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", cvPositioning.camera.getFps()));
            telemetry.addData("Total frame time ms", cvPositioning.camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", cvPositioning.camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", cvPositioning.camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", cvPositioning.camera.getCurrentPipelineMaxFps());
            telemetry.update();
            sleep(100);
        }
    }
}


