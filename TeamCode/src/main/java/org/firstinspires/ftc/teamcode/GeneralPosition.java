
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class GeneralPosition {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    static private double xcor;
    static private double ycor;
    static private double rotation;


    GeneralPosition(DcMotor motor1){
        xcor = motor1.getCurrentPosition();
        ycor =  ;
        rotation = 0;
    }



    static double getX(){
        return xcor;
    }
    static double getY(){
        return ycor;
    }
    static double getRotation(){
            return rotation;
        }

}
