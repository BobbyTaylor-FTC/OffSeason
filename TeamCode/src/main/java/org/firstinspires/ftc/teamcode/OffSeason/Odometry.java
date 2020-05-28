package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.atan2;


public class Odometry {
    LinearOpMode opModeObj;
    public DcMotorEx leftDeadWheel = null;
    public DcMotorEx rightDeadWheel = null;
    public DcMotorEx strafeDeadWheel = null;

    double leftDeadWheelPrev = 0;
    double rightDeadWheelPrev = 0;
    double strafeDeaddWheelPrev = 0;

    int leftDeadWheelCurrent = 0;
    int rightDeadWheelCurrent = 0;
    int strafeDeadWheelCurrent = 0;

    double globalPositionX = 0;
    double globalPositionY = 0;
    double globalPositionTheta = 0;

    double perpMovement = 0;
    double fwdMovement = 0;
    double deltaTheta = 0;

    public static double trackradius = 16;
    public static double thetaConstant = .5;




    public Odometry(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap) {
        opModeObj = opmode;
        leftDeadWheel = hardwareMap.get(DcMotorEx.class, "left_Dead");
        rightDeadWheel = hardwareMap.get(DcMotorEx.class, "right_Dead");
        strafeDeadWheel = hardwareMap.get(DcMotorEx.class, "strafe_Dead");
    }

    public void UpdateGlobalPosition() {

        leftDeadWheelCurrent = leftDeadWheel.getCurrentPosition();
        rightDeadWheelCurrent = rightDeadWheel.getCurrentPosition();
        strafeDeadWheelCurrent = strafeDeadWheel.getCurrentPosition();

        deltaTheta = (rightDeadWheelCurrent-rightDeadWheelPrev)-(leftDeadWheelCurrent-leftDeadWheelPrev)/(2*trackradius);
        globalPositionTheta += deltaTheta;
        perpMovement = strafeDeadWheelCurrent-strafeDeaddWheelPrev-thetaConstant*deltaTheta;
        fwdMovement = ((leftDeadWheelCurrent-leftDeadWheelPrev)+(rightDeadWheelCurrent-rightDeadWheelPrev))/2;

        globalPositionX+=fwdMovement*cos(globalPositionTheta-deltaTheta+deltaTheta/2)-perpMovement*sin(globalPositionTheta-deltaTheta+deltaTheta/2);
        globalPositionY+=fwdMovement*sin(globalPositionTheta-deltaTheta+deltaTheta/2)+perpMovement*cos(globalPositionTheta-deltaTheta+deltaTheta/2);
        leftDeadWheelPrev = leftDeadWheelCurrent;
        rightDeadWheelPrev = rightDeadWheelCurrent;
        strafeDeaddWheelPrev = strafeDeadWheelCurrent;


        globalPositionX+=fwdMovement*cos(deltaTheta/2)-perpMovement*sin(deltaTheta/2);
        globalPositionY+=fwdMovement*sin(deltaTheta/2)+perpMovement*cos(deltaTheta/2);
    }
    public double getGlobalPositionX() {
        return globalPositionX;
    }
    public double getGlobalPositionY() {
        return globalPositionY;
    }
    public double getGlobalPositionTheta() {
        return globalPositionTheta;
    }




}

