package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class hood {
    LinearOpMode opModeObj;
    private double curAngle, goalAngle;
    private double curVelo, goalVelo;
    private double ticksPerRev = 537.6;
    private double wheelDiameter = 100;
    private DcMotorEx flyWheel1   = null;
    private DcMotorEx flyWheel2  = null;

    private FFFBMath flyWheel;
    public hood(LinearOpMode opMode, hardwareGenerator gen, double fireSpeed, double kV, double kA, double kP){
        opModeObj = opMode;
        flyWheel = new FFFBMath(kV,kA, kP);
        goalVelo = fireSpeed;
        flyWheel1 = gen.flyWheel1;
        flyWheel2 = gen.flyWheel2;
    }
    public void raiseToAngle(double angle){ //raise the hood to a specific angle

    }
    public boolean atAngle(){ //return whether the hood is at the correct angle for firing
        if(Math.abs(curAngle-goalAngle)<4)
            return true;
        return false;
    }
    public void calculateCurAngle(){ //calculate the current angle of the hood

    }
    public double calculateTargetAngle(double distance){

        return distance;
    }
    public double calculateWheelVelo(double encoderTicks){
        return .1*(encoderTicks/ticksPerRev);
    }
    public void upToSpeed(double curVelo){
        double gain = flyWheel.calculateFFFBGain(goalVelo, 100, curVelo);
        flyWheel1.setPower(gain);
        flyWheel2.setPower(gain);

    }
    public boolean atSpeed(){
        if(Math.abs(curVelo-goalVelo)<4)
            return true;
        return false;
    }
    public void fire(){

    }
}
