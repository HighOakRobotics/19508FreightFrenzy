package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/EddyAutoBlue.java
public class EddyAutoBlue {
    /*
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
=======
public class EddyRotator {
    DcMotorEx rotator;
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/EddyRotator.java

    private static final double TICKS_PER_MOTOR_REV     = 537.7; //Yellow Jacket motor on rev 320 gear box
    private static final double WHEEL_DIAMETER_INCHES   = 1.8898*2;
    public static final double TICKS_PER_INCH = TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final int POSITIONING_TOLERANCE = 30; //The amount of ticks the DriveByInch method should be allowed to deviate by
    private Telemetry telemetry;
    private ElapsedTime runtime;

    public EddyRotator(HardwareMap hardwareMap, Telemetry telemetry) {
        rotator = hardwareMap.get(DcMotorEx.class, "rotator");


        this.telemetry = telemetry;
        runtime = new ElapsedTime();
    }

    public EddyRotator() {
    }

    public void autoInit() {
        resetMode();
    }

    public void humanControl(Gamepad gamepad) {

    }

    public double getR() {return rotator.getPower();}

    public void stop() {
        setDrivePowers(0.0);
    }

    public void setDrivePowers(double r) {
        rotator.setPower(r);
    }

    public void forwardByInch (double inches, double power)
    {
        int targetPos = (int)(inches * TICKS_PER_INCH);

        setDrivePowers(power);

        rotator.setTargetPosition(rotator.getCurrentPosition() + targetPos);

        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (rotator.isBusy()){
            positionReport();
        }
        stop();
        //resetMode();
    }

    public void strafeByInch (double inches, double power){
        final int targetPos = (int)(inches * TICKS_PER_INCH);
        rotator.setTargetPosition(rotator.getCurrentPosition() - targetPos);

        setDrivePowers(power);

        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (rotator.isBusy()){
            positionReport();
        }
        stop();
        //resetMode();
    }

    public void positionReport() {
        telemetry.addData("rotator", "%d %d", rotator.getTargetPosition(), rotator.getCurrentPosition());
        telemetry.update();
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        rotator.setMode(runMode);
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
    */

}