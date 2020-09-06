package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class hardwareGenerator {
    public DcMotorEx frontLeft   = null;
    public DcMotorEx frontRight  = null;
    public DcMotorEx backLeft   = null;
    public DcMotorEx backRight  = null;
    public DcMotorEx flyWheel1   = null;
    public DcMotorEx flyWheel2  = null;
    BNO055IMU imu;
    private LinearOpMode opModeObj;

    public hardwareGenerator(LinearOpMode opMode){
        opModeObj = opMode;
        frontLeft  = opModeObj.hardwareMap.get(DcMotorEx.class,"front_left");
        frontRight = opModeObj.hardwareMap.get(DcMotorEx.class,"front_right");
        backLeft = opModeObj.hardwareMap.get(DcMotorEx.class,"back_left");
        backRight = opModeObj.hardwareMap.get(DcMotorEx.class,"back_right");
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        imu = opModeObj.hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //note: run the calibration mode in SensorBNO055IMUCalibration
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        flyWheel1 =opModeObj.hardwareMap.get(DcMotorEx.class, "flyWheel1");
        flyWheel2 = opModeObj.hardwareMap.get(DcMotorEx.class, "flyWheel2");

    }
}
