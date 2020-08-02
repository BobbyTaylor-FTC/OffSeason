package org.firstinspires.ftc.teamcode.OffSeason;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.MoveToPoint;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

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
    PIDMath translation;
    PIDMath rotation;
    public drive(LinearOpMode opMode, Odometry missile, revIMU gyro){
        opModeObj = opMode;
        gyroObj = gyro;
        missileObj=missile;
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
        translation = new PIDMath (.5,0,.5);
        rotation = new PIDMath (.5,0,.5);
        runtime.reset();
    }

    public void driveT(double percentSpeed){
        double drive = -opModeObj.gamepad1.left_stick_y; // inputs, may require this.opModeObj.
        double strafe = -opModeObj.gamepad1.left_stick_x;
        double turn = opModeObj.gamepad1.right_stick_x;
        dst(drive, strafe, turn);
    }
    public void dst(double drive, double strafe, double turn){
        double lfP = drive + strafe - turn; // determining wheel proportions
        double lbP = drive - strafe - turn;
        double rfP = drive - strafe + turn;
        double rbP = drive + strafe + turn;

        double max = Math.max(1.0, Math.abs(lfP)); // smooth out a little - can be deleted
        max = Math.max(max, Math.abs(lbP));
        max = Math.max(max, Math.abs(rfP));
        max = Math.max(max, Math.abs(rbP));

        frontLeft.setPower(lfP); // set powers
        backLeft.setPower(lbP);
        frontRight.setPower(rfP);
        backRight.setPower(rbP);
        runtime.seconds();
    }

    public void driveT(double leftPower, double rightPower){
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
    }

    public void turnTo(double angle){
        while(abs(error)>.5) {
            double wrappingError = -360 - gyroObj.getAngle() + angle;
            double straightError = gyroObj.getAngle() - angle;
            if (abs(wrappingError) < abs(straightError)) {
                error = -360 - gyroObj.getAngle() + angle;
                error *= -1;
            } else {
                error = gyroObj.getAngle() - angle;
            }
            if (abs(error) > .5) {
               double gain = rotation.calculateGain(error, runtime.milliseconds());
               driveT(gain,-gain);
            }
            else{
                break;
            }
        }
        driveT(0,0);
    }

    /**
     * Code from FTCLib
     *
     * @param leftStickX   horizontal of the left stick of the main gamepad
     * @param leftStickY   vertical of the left stick of the main gamepad
     * @param rightStickX  horizontal of the right stick of the main gamepad
     * @param turnSpeed    how fast it turns
     */

    public void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double turnSpeed){
        Vector2d initial = new Vector2d(leftStickX,leftStickY);
        initial.rotateBy(gyroObj.getAngle());
        double theta = Math.atan2(leftStickX,leftStickY);
        frontLeft.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) + turnSpeed);
        frontRight.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) - turnSpeed);
        backLeft.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) + turnSpeed);
        backRight.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) - turnSpeed);

    }

    public void moveToPoint(double X, double Y, double theta){
        double xGain = translation.calculateGain(missileObj.getGlobalPositionX()-X, runtime.seconds());
        double yGain = translation.calculateGain(missileObj.getGlobalPositionY()-Y, runtime.seconds());

        double wrappingError = -360 - gyroObj.getAngle() + theta;
        double straightError = gyroObj.getAngle() - theta;
        if (abs(wrappingError) < abs(straightError)) {
            error = -360 - gyroObj.getAngle() + theta;
            error *= -1;
        } else {
            error = gyroObj.getAngle() - theta;
        }

        double tGain = rotation.calculateGain(error, runtime.seconds());
        double drive = xGain*cos(missileObj.getGlobalPositionTheta())-yGain*sin(missileObj.getGlobalPositionTheta());
        double strafe = xGain*sin(missileObj.getGlobalPositionTheta())+yGain*cos(missileObj.getGlobalPositionTheta());
        dst(drive, strafe, tGain);

    }

}
