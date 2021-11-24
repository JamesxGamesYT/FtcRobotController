/* Abstract base OpMode class with motors specific to the Nisky RoboWarriors 6460 robot.
 *
 * Authors: Arin Khare
 */


package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public abstract class FreightFrenzyOpMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor carousel = null, slidesLeft = null, slidesRight = null, frontRightDrive = null,
            rearRightDrive = null, frontLeftDrive = null, rearLeftDrive = null;
    private Servo claw = null;
}