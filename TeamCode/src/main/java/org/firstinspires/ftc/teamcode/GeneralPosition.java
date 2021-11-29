package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//more than 2 motors
/*import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;
import java.util.List;
*/
// 280 encoder counts per revolution
public class Drive{
    private static DcMotor LEFT_FRONT_DRIVE_MOTOR = null;
    private static DcMotor LEFT_BACK_DRIVE_MOTOR = null;
    private static DcMotor RIGHT_FRONT_DRIVE_MOTOR = null;
    private static DcMotor RIGHT_BACK_DRIVE_MOTOR = null;

    public static int LEFT_FRONT_DRIVE_ENCODER = 0;
    public static int LEFT_BACK_DRIVE_ENCODER = 0;
    public static int RIGHT_FRONT_DRIVE_ENCODER = 0;
    public static int RIGHT_BACK_DRIVE_ENCODER = 0;

    double ADPower = LEFT_FRONT_DRIVE_MOTOR.getPower()+RIGHT_BACK_DRIVE_MOTOR.getPower();
    double BCPower = LEFT_BACK_DRIVE_MOTOR.getPower()+RIGHT_FRONT_DRIVE_MOTOR.getPower();
    double ADangle = Math.acos((2*ADPower+Math.sqrt(8-4(ADPower*ADPower)))/4);
    double BCangle = Math.acos((2*BCPower+Math.sqrt(8-4(BCPower*BCPower)))/4);
    double SecondXcor = PositionManager.getXcor() + BCPower * Math.cos(BCangle);
    double SecondYcor = PositionManager.getYcor() + BCPower * Math.sin(BCangle);
    double ThirdXcor = SecondXcor - ADPower*Math.cos(ADangle);
    double ThirdYcor = SecondYcor + ADPower*Math.sin(ADangle);
    PositionManager.setXcor(ThirdXcor);
    PositionManager.setYcor(ThirdYcor);
}
