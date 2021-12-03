package org.firstinspires.ftc.teamcode;

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
//public class Drive{
//    private static motor LEFT_FRONT_DRIVE_MOTOR = null;
//    private static motor LEFT_BACK_DRIVE_MOTOR = null;
//    private static motor RIGHT_FRONT_DRIVE_MOTOR = null;
//    private static motor RIGHT_BACK_DRIVE_MOTOR = null;
//
//    public static Encoder LEFT_FRONT_DRIVE_ENCODER = null;
//    public static Encoder LEFT_BACK_DRIVE_ENCODER = null;
//    public static Encoder RIGHT_FRONT_DRIVE_ENCODER = null;
//    public static Encoder RIGHT_BACK_DRIVE_ENCODER = null;
//    public static MedianPIDSource DRIVE_ENCODERS = null;
//
//    DRIVE_GYRO = new AHRS(RobotMap.MXP_PORT);
//    public void initDefaultCommand(){
//
//    }
//    double ADPower = LEFT_FRONT_DRIVE_MOTOR.getPower()+RIGHT_BACK_DRIVE_MOTOR.getPower();
//    double BCPower = LEFT_BACK_DRIVE_MOTOR.getPower()+RIGHT_FRONT_DRIVE_MOTOR.getPower();
//    double ADangle = Math.acos((2x+Math.sqrt(8-4(Math.pow(ADPower,2))))/4);
//    double BCangle = Math.acos((2x+Math.sqrt(8-4(Math.pow(BCPower,2))))/4);
//}
//static void setMechanumDrive(double translationAngle, double translationPower, double turnPower){
//    double ADPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) + Math.cos(translationAngle));
//    double BCPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) - Math.cos(translationAngle));
//}
//public static class GeneralPosition {
//    public static double TICKS_PER_REV = 0;
//    public static double WHEEL_RADIUS = 2;
//    public static double GEAR_RATION = 1;
//
//    public static double PARALLEL_X = 0;
//    public static double PARALLEL_Y = 0;
//
//    public static double PERPENDICULAR_X = 0;
//    public static double PERPENDICULAR_Y = 0;
//
//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
//
//    static private double xcor;
//    static private double ycor;
//    static private double rotation;
//
//
//    General Position(){
//        xcor = 0;
//        ycor = 0;
//        rotation = 0;
//    }
//    GeneralPosition(double x, double, double r){
//        xcor = x;
//        ycor = y;
//        rotation = r;
//    }
//    xcor = leftDrive.getCurrentPosition();
//    rotation = 0;
//
//
//    static double getX(){
//        return xcor;
//    }
//    static double getY(){
//        return ycor;
//    }
//    static double getRotation(){
//        return rotation;
//    }
//
//}
