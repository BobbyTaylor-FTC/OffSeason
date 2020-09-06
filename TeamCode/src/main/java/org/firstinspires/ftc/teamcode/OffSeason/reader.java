package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class reader {

    LinearOpMode opModeObj;
    hardwareGenerator gen;

    public int leftDeadCur, rightDeadCur, strafeDeadCur;



    public reader(LinearOpMode opMode, hardwareGenerator hard){
        gen = hard;
        opModeObj = opMode;
    }

    public void autonRead(){
        leftDeadCur = gen.frontLeft.getCurrentPosition();
        rightDeadCur = gen.frontRight.getCurrentPosition();
        strafeDeadCur = gen.backRight.getCurrentPosition();
    }
    public void teleRead(){

    }
}
