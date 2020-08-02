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
    double strafeDeadWheelPrev = 0;

    double leftMMPerTick;
    double rightMMPerTick;
    double strafeMMPerTick;

    double leftWheelDelta;
    double rightWheelDelta;
    double strafeWheelDelta;

    double leftWheelTotal;
    double rightWheelTotal;
    double strafeWheelTotal;

    double globalPositionX = 0;
    double globalPositionY = 0;
    double globalPositionTheta = 0;

    double xChange;
    double yChange;

    double perpMovement = 0;
    double fwdMovement = 0;
    double deltaTheta = 0;

    double sinTerm;
    double cosTerm;

    double drive, strafe;

    double cTerm, sTerm;


    public static double trackwidth = 16;
    public static double thetaConstant = .5;



    public Odometry(LinearOpMode opmode) {
        opModeObj = opmode;
        leftDeadWheel = opModeObj.hardwareMap.get(DcMotorEx.class, "left_Dead");
        rightDeadWheel = opModeObj.hardwareMap.get(DcMotorEx.class, "right_Dead");
        strafeDeadWheel = opModeObj.hardwareMap.get(DcMotorEx.class, "strafe_Dead");
    }

    /**
     *
     * @param leftDeadWheelCurrent current position of the left deadwheel, to be passed from the opmode to allow for bulk reads
     * @param rightDeadWheelCurrent current position of the right deadwheel, to be passed from the opmode to allow for bulk reads
     * @param strafeDeadWheelCurrent current position of the strafe deadwheel, to be passed from the opmode to allow for bulk reads
     */

    public void UpdateGlobalPosition(int leftDeadWheelCurrent, int rightDeadWheelCurrent, int strafeDeadWheelCurrent) {

        leftWheelDelta = (leftDeadWheelCurrent-leftDeadWheelPrev)*leftMMPerTick;
        rightWheelDelta = (rightDeadWheelCurrent-rightDeadWheelPrev)*rightMMPerTick;
        strafeWheelDelta = (strafeDeadWheelCurrent-strafeDeadWheelPrev)*strafeMMPerTick;



        //change in angle since the previous update
        deltaTheta = (leftWheelDelta-rightWheelDelta)/(trackwidth);

        // absolute angle
        leftWheelTotal = leftDeadWheelCurrent*leftMMPerTick;
        rightWheelTotal = rightDeadWheelCurrent*rightMMPerTick;


        //changes in x and y movement
        perpMovement = strafeWheelDelta-thetaConstant*deltaTheta;
        fwdMovement = (leftWheelDelta+rightWheelDelta)/2.0;



        //update theta AFTER rotation matrix has been multiplied in
        globalPositionTheta = (leftWheelTotal-rightWheelTotal)/trackwidth;

        if(deltaTheta <pow(10,-6)){
            sinTerm = 1-(deltaTheta*deltaTheta)/6;
            cosTerm = deltaTheta/2.0;
        }
        else{
            sinTerm = sin(deltaTheta)/deltaTheta;
            cosTerm = (1-cos(deltaTheta))/deltaTheta;
        }
        strafe = cosTerm*fwdMovement + sinTerm* perpMovement;
        drive = sinTerm*fwdMovement - cosTerm*perpMovement;

        cTerm = cos(globalPositionTheta);
        sTerm = sin(globalPositionTheta);

        globalPositionY += drive*cTerm-strafe*sTerm;
        globalPositionX += drive*sTerm+strafe*cTerm;

        /*
        //product of the rotation matrix and the [x,y] matrices
        globalPositionX+=fwdMovement*cos(globalPositionTheta)-perpMovement*sin(globalPositionTheta);
        globalPositionY+=fwdMovement*sin(globalPositionTheta)+perpMovement*cos(globalPositionTheta);
        */

        //storing deadwheel encoder values for future use
        leftDeadWheelPrev = leftDeadWheelCurrent;
        rightDeadWheelPrev = rightDeadWheelCurrent;
        strafeDeadWheelPrev = strafeDeadWheelCurrent;



        opModeObj.telemetry.addData("Current X:", getGlobalPositionX());
        opModeObj.telemetry.addData("Current Y:", getGlobalPositionY());
        opModeObj.telemetry.addData("Current theta:", getGlobalPositionTheta());

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

