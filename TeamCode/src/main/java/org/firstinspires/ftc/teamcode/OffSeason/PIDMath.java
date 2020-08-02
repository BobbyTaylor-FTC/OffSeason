package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class PIDMath {
    private double P, I, D;
    double pastError;
    double totalError;
    double prevTime;
    public PIDMath(double kP, double kI, double kD){
     P = kP;
     I = kI;
     D = kD;
    }
    public double calculateGain(double error, double nowtime){
        totalError += error;
        double gain = error * P + totalError * I + D * (error-pastError)/(nowtime-prevTime);
        prevTime = nowtime;
        pastError = error;
        return gain;
    }
    public void resetPID(){
        pastError = 0;
        totalError = 0;
        prevTime = 0;
    }
}
