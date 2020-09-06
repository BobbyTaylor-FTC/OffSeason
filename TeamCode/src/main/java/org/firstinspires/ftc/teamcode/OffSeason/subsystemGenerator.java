package org.firstinspires.ftc.teamcode.OffSeason;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
public class subsystemGenerator {

    public revIMU gyro;
    public Odometry missile;
    public drive vroom;
    public bulk reader;
    public hood flipper;
    public reader hardReader;
    public hardwareGenerator support, hard;

    public subsystemGenerator(LinearOpMode opMode, hardwareGenerator hard){
        support = hard;
        missile = new Odometry(opMode,support);
        gyro = new revIMU(opMode, support);
        flipper = new hood(opMode, support, .5, .5,0,.5);
        vroom = new drive(opMode, support, missile, gyro, flipper);
        reader = new bulk(opMode);
        hardReader = new reader(opMode,hard);
    }

/*
    public revIMU getInstancerevIMU() {
        return gyro;
    }

    public Odometry getInstanceOdo() {
        return missile;
    }

    public drive getInstanceDrive() {
        return vroom;
    }

    public bulk getInstanceBulk() {
        return reader;
    }
*/

}
