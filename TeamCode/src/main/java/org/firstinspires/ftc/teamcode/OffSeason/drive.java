package org.firstinspires.ftc.teamcode.OffSeason;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Disabled

public class drive
{

    static final double COUNTS_PER_MOTOR_REV = 537.5;    // eg: Hex HD Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    LinearOpMode opModeObj;


    //variables for taking the derivative of error
    public double lasterror;
    public double error;
    public double output;
    public double scaledoutput;
    public double nowtime, thentime;

    double correction;
    double avg = 0;
    double cp;
    double tp;


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime motortime = new ElapsedTime();


    //for gyro strafing
    public static double kPstrafeangle = 0.015;
    public static double kDstrafeangle = 0.006;

    //for gyro driving
    public static double kPdriveangle = 0.010;
    public static double kDdriveangle = 0.006;

    //for turning
    public static double kPturn = .036; //.03
    public static double kDturn = 0.003;




    public double strafeHeading = 0;
    public double currentheading;

    public int frontLefttarget, frontRighttarget,backRighttarget,backLefttarget;

    public DcMotorEx frontLeft   = null;
    public DcMotorEx  frontRight  = null;
    public DcMotorEx  backLeft   = null;
    public DcMotorEx  backRight  = null;
    public revIMU gyroObj;
    public Odometry missileObj;
    public MoveToPoint forwardsObj;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public drive(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap, Odometry missile, MoveToPoint forwards, revIMU gyro){
        opModeObj = opmode;
        gyroObj = gyro;
        missileObj=missile;
        forwardsObj = forwards;
        frontLeft  = hardwareMap.get(DcMotorEx.class,"front_left");
        frontRight = hardwareMap.get(DcMotorEx.class,"front_right");
        backLeft = hardwareMap.get(DcMotorEx.class,"back_left");
        backRight = hardwareMap.get(DcMotorEx.class,"back_right");
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

    }

    public void driveT(double percentSpeed){
        double drive = -this.opModeObj.gamepad1.left_stick_y; // inputs
        double strafe = -this.opModeObj.gamepad1.left_stick_x;
        double turn = .7*this.opModeObj.gamepad1.right_stick_x;

        double lfP = drive + strafe - turn; // determining wheel proportions
        double lbP = drive - strafe - turn;
        double rfP = drive - strafe + turn;
        double rbP = drive + strafe + turn;

        double max = Math.max(1.0, Math.abs(lfP)); // smooth out a little - can be deleted
        max = Math.max(max, Math.abs(lbP));
        max = Math.max(max, Math.abs(rfP));
        max = Math.max(max, Math.abs(rbP));

        frontLeft.setPower(lfP * percentSpeed); // set powers
        backLeft.setPower(lbP * percentSpeed);
        frontRight.setPower(rfP * percentSpeed);
        backRight.setPower(rbP * percentSpeed);
        //  strafeHeading = gyro.getAngle();

    }

}
