package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OffSeason.Point;
import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.atan2;
@Disabled

public class Odometry {
    LinearOpMode opModeObj;
    public DcMotorEx leftDeadWheel = null;
    public DcMotorEx rightDeadWheel = null;
    public DcMotorEx strafeDeadWheel = null;

    private double leftDeadWheelPrev = 0;
    private double rightDeadWheelPrev = 0;
    private double strafeDeadWheelPrev = 0;

    private double leftMPerTick;
    private double rightMPerTick;
    private double strafeMPerTick;

    private double leftWheelDelta;
    private double rightWheelDelta;
    private double strafeWheelDelta;

    private double leftWheelTotal;
    private double rightWheelTotal;
    private double strafeWheelTotal;

    double globalPositionX = 0;
    double globalPositionY = 0;
    double globalPositionTheta = 0;

    private double xChange;
    private double yChange;

    private double perpMovement = 0;
    private double fwdMovement = 0;
    private double deltaTheta = 0;

    private double sinTerm;
    private double cosTerm;

    private double drive, strafe;

    private double cTerm, sTerm;

    double leftWheelVelo = 0;
    double rightWheelVelo = 0;

    public Point robotLocation;

    private double cpr = 8192;
    private double wheelDiameter = .058;

    public static double trackwidth = 16;
    public static double thetaConstant = .5;

    ElapsedTime veloTime = new ElapsedTime();


    public Odometry(LinearOpMode opmode, hardwareGenerator gen) {
        opModeObj = opmode;
        leftDeadWheel = gen.frontLeft;
        rightDeadWheel = gen.frontRight;
        strafeDeadWheel = gen.backLeft;
        robotLocation = new Point(0,0);
    }

    /**
     *
     * @param leftDeadWheelCurrent current position of the left deadwheel, to be passed from the opmode to allow for bulk reads
     * @param rightDeadWheelCurrent current position of the right deadwheel, to be passed from the opmode to allow for bulk reads
     * @param strafeDeadWheelCurrent current position of the strafe deadwheel, to be passed from the opmode to allow for bulk reads
     */

    public void UpdateGlobalPosition(int leftDeadWheelCurrent, int rightDeadWheelCurrent, int strafeDeadWheelCurrent) {
        leftWheelDelta = (leftDeadWheelCurrent-leftDeadWheelPrev)*leftMPerTick;
        rightWheelDelta = (rightDeadWheelCurrent-rightDeadWheelPrev)*rightMPerTick;
        strafeWheelDelta = (strafeDeadWheelCurrent-strafeDeadWheelPrev)*strafeMPerTick;



        //change in angle since the previous update
        deltaTheta = (leftWheelDelta-rightWheelDelta)/(trackwidth);

        // absolute angle
        leftWheelTotal = leftDeadWheelCurrent*leftMPerTick;
        rightWheelTotal = rightDeadWheelCurrent*rightMPerTick;


        //changes in x and y movement
        perpMovement = strafeWheelDelta-thetaConstant*deltaTheta;
        fwdMovement = (leftWheelDelta+rightWheelDelta)/2.0;



        //update theta AFTER rotation matrix has been multiplied in
        globalPositionTheta = (leftWheelTotal-rightWheelTotal)/trackwidth;

        if(deltaTheta < pow(10,-6)){
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

        leftWheelVelo = leftMPerTick*leftWheelDelta/veloTime.seconds();
        rightWheelVelo = rightMPerTick*rightWheelDelta/veloTime.seconds();


        //storing deadwheel encoder values for future use
        leftDeadWheelPrev = leftDeadWheelCurrent;
        rightDeadWheelPrev = rightDeadWheelCurrent;
        strafeDeadWheelPrev = strafeDeadWheelCurrent;



        opModeObj.telemetry.addData("Current X:", globalPositionX);
        opModeObj.telemetry.addData("Current Y:", globalPositionY);
        opModeObj.telemetry.addData("Current theta:", globalPositionTheta);
        robotLocation = new Point (globalPositionX,globalPositionY);
        veloTime.reset();
    }
    /*
    public double getGlobalPositionX() {
        return globalPositionX;
    }
    public double getGlobalPositionY() {
        return globalPositionY;
    }
    public double getGlobalPositionTheta() {
        return globalPositionTheta;
    }
    public Point getRobotPoint(){
        return robotLocation;
    }
    public double getLeftWheelVelo() {
        return leftWheelVelo;
    }
    public double getRightWheelVelo() {
        return rightWheelVelo;
    }
*/


}

