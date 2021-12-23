package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrainMecanumBasic {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;


    private static final double TICKS_PER_MOTOR_REV     = 537.7; //Yellow Jacket motor on rev 320 gear box
    private static final double WHEEL_DIAMETER_INCHES   = 1.8898*2;
    public static final double TICKS_PER_INCH = TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final int POSITIONING_TOLERANCE = 30; //The amount of ticks the DriveByInch method should be allowed to deviate by
    private Telemetry telemetry;
    private ElapsedTime runtime;

    public DriveTrainMecanumBasic(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        this.telemetry = telemetry;
        runtime = new ElapsedTime();
    }

    public void autoInit() {
        resetMode();
    }

    public void humanControl(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        drive(forward, strafe, turn);

    }

    public double getFL() {return frontLeft.getPower();}
    public double getBL() {return backLeft.getPower();}
    public double getFR() {return frontRight.getPower();}
    public double getBR() {return backRight.getPower();}

    public void stop() {
        setDrivePowers(0.0, 0.0, 0.0, 0.0);
    }

    public void setDrivePowers(double fl, double bl, double br, double fr) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontRight.setPower(fr);
    }

    public void drive (double forward, double strafe, double turn)
    {
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        setDrivePowers(fl, bl, br, fr);
    }

    public void forwardByInch (double inches, double power)
    {
        int targetPos = (int)(inches * TICKS_PER_INCH);

        setDrivePowers(power, power, power, power);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetPos);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetPos);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetPos);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetPos);

        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backLeft.isBusy()){
            positionReport();
        }
        stop();
        //resetMode();
    }

    public void strafeByInch (double inches, double power){
        final int targetPos = (int)(inches * TICKS_PER_INCH);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - targetPos);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetPos);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetPos);
        backRight.setTargetPosition(backRight.getCurrentPosition() - targetPos);

        setDrivePowers(power, power, power, power);

        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backLeft.isBusy()){
            positionReport();
        }
        stop();
        //resetMode();
    }

    public void positionReport() {
        telemetry.addData("frontleft ", "%d %d", frontLeft.getTargetPosition(), frontLeft.getCurrentPosition());
        telemetry.addData("frontright ", "%d %d", frontRight.getTargetPosition(), frontRight.getCurrentPosition());
        telemetry.addData("backleft ", "%d %d", backLeft.getTargetPosition(), backLeft.getCurrentPosition());
        telemetry.addData("backright ", "%d %d", backRight.getTargetPosition(), backRight.getCurrentPosition());
        telemetry.update();
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void sleep(int second) {
        runtime.reset();
        while (runtime.seconds() < second) {}
    }

    public void resetMode() {
        setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        positionReport();
    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power){
        double  leftPower, rightPower;



        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;
        // set power to rotate.
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}
        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(1000);


        resetAngle();


    }

}