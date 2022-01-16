package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SwingArmS extends Subsystem {
    DcMotorEx wrist;
    private static final double TICKS_PER_MOTOR_REV     = 1120; // andymark neverest 40 gear
    private static final double WHEEL_DIAMETER_INCHES   = 1;
    public static final double TICKS_PER_INCH = TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    int kWristHigh = -450;
    int kWristLow = 0;

    Servo hand,  shoulder;
    double hIntake = 0.68; //0.6; //hold position 0.3 intake
    double hHome = 0.85;//home position too much
    double hRelease3 = 0.05; //depend on the wrist position
    double hRelease2 = 0.15;
    double hRelease1 = 0.35;
    double hHold3 = 0.4;
    double hHold2 = 0.35;
    double hHold1 = 0.7;
    double hAHold = 0.5;

    double sMiddle = 0.54; // middle   - more;
    double sLeft = 0.22;//lower
    double sRight = 0.85; // no wire now 0.9;


    double setpoint;
    double setpointTarget;
    boolean busy;

    public void update(double hPos, double sPos) {
        hand.setPosition(hPos);
        shoulder.setPosition(sPos);
    }

    public void center() {
        if (astate != ArmState.LIFT && astate != ArmState.HOME && astate != ArmState.INTAKE) return;
        sstate = ShoulderState.MIDDLE;
        update(hHome, sMiddle);
    }

    public void left() {
        if (astate != ArmState.LIFT) return;
        update (hand.getPosition(), sLeft);
        sstate = ShoulderState.LEFT;
    }

    public void right() {
        if (astate != ArmState.LIFT) return;
        update (hand.getPosition(), sRight);
        sstate = ShoulderState.RIGHT;
    }

    public void moveToTarget(int target, double power) {
        wrist.setPower(power);
        wrist.setTargetPosition(target);
        if (wrist.isBusy()) busy = true;
        else busy = false;
    }
    public void wristHome(){
        if(astate != ArmState.LIFT && astate != ArmState.INTAKE && astate != ArmState.HOME) return;
        astate = ArmState.HOME;
        center();
        moveToTarget(0, 0.1);
        if (!busy) wrist.setPower(0);
    }

    public void wristLift(){
        astate = ArmState.LIFT;
        center();
        moveToTarget(-300, 0.2);
    }

    public void release() {
        if (wstate == WristState.LEVEL3) hand.setPosition(hRelease3);
        else if (wstate == WristState.LEVEL2) hand.setPosition(hRelease2);
        else if (wstate == WristState.LEVEL1) hand.setPosition(hRelease1);
    }

    public void retrieve(){
        if (wstate == WristState.LEVEL3) hand.setPosition(hHold3);
        if (wstate == WristState.LEVEL2) hand.setPosition(hHold2);
        if (wstate == WristState.LEVEL1) hand.setPosition(hHold1);
    }

    public void intake() {
        if (astate != ArmState.HOME) return; //intake can only go from home state
        update(hIntake, sMiddle);
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.INTAKE;
    }

    public void deliver3 () {
        if (astate != ArmState.LIFT) return; //
        astate = ArmState.DELIVER3;
        update(hHold3, shoulder.getPosition());
        wstate = WristState.LEVEL3;
        moveToTarget(-430, 0.3);
    }

    public void deliver1() {
        if (astate != ArmState.LIFT) return;
        astate = ArmState.DELIVER1;
        update(hHold1, shoulder.getPosition());
        wstate = WristState.LEVEL1;
        moveToTarget(-220, 0.2);
    }

    public void deliver2() {
        if (astate != ArmState.LIFT) return;
        astate = ArmState.DELIVER2;
        update(hHold2, shoulder.getPosition());
        wstate = WristState.LEVEL2;
        moveToTarget(-350, 0.2);
    }

    public double getHandPos() {return hand.getPosition(); }
    public double getShoulderPos() {return shoulder.getPosition(); }
    public double getMWristPosition() { return wrist.getCurrentPosition(); }

    public void modifySetpoint(double amount) {
        setpointTarget = setpoint+amount;
    }

    public ArmState getState() {
        return astate;
    }

    public void setMode(ArmState state) {
        this.astate = state;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        astate = ArmState.HOME;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;

        hand = hardwareMap.get(Servo.class, "hand");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        update(hHome, sMiddle);

        wrist = hardwareMap.get(DcMotorEx.class, "mWrist");
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristHome();

        setpoint = 0;
    }

    @Override
    public void initPeriodic() {
    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        switch (astate) {
            case HOME:
                wristHome();
                break;
            case LIFT:
                wristLift();
                break;
            case INTAKE:
                intake();
                break;
            case DELIVER1:
                deliver1();
                break;
            case DELIVER2:
                deliver2();
                break;
            case DELIVER3:
                deliver3();
                break;
            case LEFT:
                left();
            case RIGHT:
                right();
            case RELEASE:
                release();
            case RETRIVE:
                retrieve();
        }

        setpoint = (setpointTarget - setpoint) / 2 + setpoint;

        telemetry.addLine("[ARM]")
                .addData("State ", astate)
                .addData("Current ", "h:%f.2 w:%d s%f.2", getHandPos(), wrist.getCurrentPosition(), getShoulderPos());
    }

    @Override
    public void stop() {
        wrist.setPower(0);
    }

    public enum WristState{
        HOME,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        TEAM
    }
    public enum ShoulderState {
        LEFT,
        RIGHT,
        MIDDLE
    }
    public enum ArmState {
        HOME,
        INTAKE,
        LIFT,
        DELIVER1,
        DELIVER2,
        DELIVER3,
        LEFT,
        RIGHT,
        RELEASE,
        RETRIVE
    }

    WristState wstate;
    ShoulderState sstate;
    ArmState astate;
}
