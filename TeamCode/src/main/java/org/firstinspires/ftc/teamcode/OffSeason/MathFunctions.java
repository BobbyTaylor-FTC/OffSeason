package org.firstinspires.ftc.teamcode.OffSeason;

import static java.lang.Math.*;

public class MathFunctions {
    public static double AngleWrap(double angle){
        while(angle<-PI){
            angle+=2*PI;
        }
        while(angle>PI){
            angle-=2*PI;
        }
        return angle;
    }
}
