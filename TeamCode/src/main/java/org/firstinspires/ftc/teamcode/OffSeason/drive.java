package org.firstinspires.ftc.teamcode.OffSeason;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private double lasterror;
    private double error;
    private double output;
    private double scaledoutput;
    private double nowtime, thentime;

    private double correction;
    private double avg = 0;
    private double cp;
    private double tp;

    private static double acceptableShootingAngleError = 10;


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

    private double strafeHeading = 0;
    private double currentheading;

    private int frontLefttarget, frontRighttarget,backRighttarget,backLefttarget;

    public DcMotorEx frontLeft   = null;
    public DcMotorEx frontRight  = null;
    public DcMotorEx backLeft   = null;
    public DcMotorEx backRight  = null;
    public revIMU gyroObj;
    public Odometry missileObj;
    public hood hoodObj;
    PIDMath translation;
    PIDMath rotation;
    public drive(LinearOpMode opMode, hardwareGenerator gen, Odometry missile, revIMU gyro, hood flipper){
        frontLeft = gen.frontLeft;
        frontRight = gen.frontRight;
        backLeft = gen.backLeft;
        backRight = gen.backRight;
        opModeObj = opMode;
        gyroObj = gyro;
        missileObj=missile;
        hoodObj = flipper;
        translation = new PIDMath (.5,0,.5);
        rotation = new PIDMath (.5,0,.5);
        runtime.reset();
    }

    public void driveT(double percentSpeed, double leftStickY, double leftStickX, double rightStickX){
        double drive = -leftStickY; // inputs, may require this.opModeObj.
        double strafe = -leftStickX;
        double turn = rightStickX;
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
     * @param rightStickX    how fast it turns
     */

    public void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double angle){
        Vector2d initial = new Vector2d(leftStickX,leftStickY);
        initial.rotateBy(angle);
        double theta = Math.atan2(leftStickX,leftStickY);
        frontLeft.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) + rightStickX);
        frontRight.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) - rightStickX);
        backLeft.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) + rightStickX);
        backRight.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) - rightStickX);

    }

    public void driveTurnToShoot(double leftStickX, double leftStickY, Point robotLocation, double curAngle, Point goalLocation){
        Vector2d initial = new Vector2d(leftStickX,leftStickY);
        initial.rotateBy(curAngle);
        double theta = Math.atan2(leftStickX,leftStickY);
        double smallAngleError = smallestAngleError(Math.atan2(robotLocation.y-goalLocation.y,robotLocation.x-goalLocation.x),curAngle);
        double gain = rotation.calculateGain(smallAngleError,runtime.seconds());
        if(smallAngleError<acceptableShootingAngleError&&hoodObj.atAngle()&&hoodObj.atSpeed()){
            hoodObj.fire();
        }
        hoodObj.raiseToAngle(hoodObj.calculateTargetAngle(distanceFormula(robotLocation,goalLocation)));
        frontLeft.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) + gain);
        frontRight.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) - gain);
        backLeft.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) + gain);
        backRight.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) - gain);



    }

    public void moveToPoint(double X, double Y, double theta){
        double xGain = translation.calculateGain(missileObj.globalPositionX-X, runtime.seconds());
        double yGain = translation.calculateGain(missileObj.globalPositionY-Y, runtime.seconds());

        double tGain = rotation.calculateGain(smallestAngleError(theta,missileObj.globalPositionTheta), runtime.seconds());
        double drive = xGain*cos(missileObj.globalPositionTheta)-yGain*sin(missileObj.globalPositionTheta);
        double strafe = xGain*sin(missileObj.globalPositionTheta)+yGain*cos(missileObj.globalPositionTheta);
        dst(drive, strafe, tGain);

    }
    private double smallestAngleError(double goalAngle, double curAngle){
        double wrappingError = -360 - curAngle + goalAngle;
        double straightError = curAngle - goalAngle;
        if (abs(wrappingError) < abs(straightError)) {
            error = -360 - curAngle + goalAngle;
            error *= -1;
        } else {
            error = curAngle - goalAngle;
        }
        return error;
    }

    public void moveToPoint(Point goalPoint, double theta){
        moveToPoint(goalPoint.x,goalPoint.y,theta);
    }

    private double distanceFormula(Point a, Point b){
        return Math.sqrt(Math.pow(a.x-b.x,2)+Math.pow(a.y-b.y,2));
    }

}
