package org.firstinspires.ftc.teamcode.OffSeason;

public class FFFBMath {
    private double kV, kA, kP;

    public FFFBMath(double V, double A, double P){
        kV = V;
        kA = A;
        kP = P;
    }
    public double calculateFFFBGain(double targetVelo, double targetAccel, double curVelo){
        double gain = kV*targetVelo+kA*targetAccel+kP*(targetVelo-curVelo);
        return gain;
    }
}
