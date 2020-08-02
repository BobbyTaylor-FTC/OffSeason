package org.firstinspires.ftc.teamcode.OffSeason;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class subsystemGenerator {

    private revIMU gyro;
    private Odometry missile;
    private drive vroom;
    private bulk reader;
    private LinearOpMode opModeObj;
    public subsystemGenerator(LinearOpMode opMode){
        missile = new Odometry(opMode);
        gyro = new revIMU(opMode);
        vroom = new drive(opMode, missile, gyro);
        reader = new bulk(opMode);
        opModeObj = opMode;
    }

    public revIMU getInstancerevIMU() {
        return gyro;
    }

    public Odometry getInstanceOdo() {
        return missile;
    }

    public drive getInstanceDrive() {
        return vroom;
    }
}
