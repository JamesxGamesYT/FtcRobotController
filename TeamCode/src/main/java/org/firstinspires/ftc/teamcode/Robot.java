/* Authors: Arin Khare
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CVPositioning;
import org.firstinspires.ftc.teamcode.EncoderPositioning;
import org.firstinspires.ftc.teamcode.MechanismDriving;
import org.firstinspires.ftc.teamcode.Navigation;


/** A completely encompassing class of all functionality of the robot. An OpMode should interface through an instance of
 *  this class in order to send or receive any data with the real robot.
 */
public class Robot {

    private DcMotor carousel, slidesLeft, slidesRight, frontRightDrive, rearRightDrive, frontLeftDrive, rearLeftDrive;
    private Servo claw;

    private EncoderPositioning encoderPositioning;
    private CVPositioning cvPositioning;
    private MechanismDriving mechanismDriving;
    private Navigation navigation;

    public Robot()
}
