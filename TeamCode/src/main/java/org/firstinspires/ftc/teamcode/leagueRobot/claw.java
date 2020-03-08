package org.firstinspires.ftc.teamcode.leagueRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class claw
{

    LinearOpMode opModeObj;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public claw(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
        opModeObj = opmode;
        leftClaw = hardwareMap.get(Servo.class,"left_claw");
        rightClaw = hardwareMap.get(Servo.class,"right_claw");

    }
    public void close(){
        rightClaw.setPosition(.833333);  //1
        leftClaw.setPosition(.63666667); //.5
    }
    public void release(){
        rightClaw.setPosition(.5); //.5
        leftClaw.setPosition(1); //1
    }


}
