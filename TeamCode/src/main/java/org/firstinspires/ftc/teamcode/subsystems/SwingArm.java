package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwingArm {
    protected HardwareMap hwMap;
    ElapsedTime timer;
    double curr, target, increament;
    boolean firstCall;
    Servo hand,  shoulder;
    DcMotorEx mWrist;
    private static final double TICKS_PER_MOTOR_REV     = 1120; // andymark neverest 40 gear
    private static final double WHEEL_DIAMETER_INCHES   = 1;
    public static final double TICKS_PER_INCH = TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    CRServo cwrist;
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

    boolean busy = false;
    double currH, currW, currS;

    public enum HandState {
        HOLD,
        PICKUP
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
        DELIVER
    }
    HandState hstate;
    WristState wstate;
    ShoulderState sstate;
    ArmState astate;

    public SwingArm (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        hand = hwMap.get(Servo.class, "hand");
        cwrist = hwMap.get(CRServo.class, "cwrist");
        shoulder = hwMap.get(Servo.class, "shoulder");
        mWrist = hwMap.get(DcMotorEx.class, "mWrist");
        mWrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        astate = ArmState.HOME;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        timer = new ElapsedTime();
        firstCall = true;
        mWrist.setPower(0);
        update(hHome, sMiddle);
    }

    public double getHandPos() {return hand.getPosition(); }
    public double getShoulderPos() {return shoulder.getPosition(); }
    public double getMWristPosition() { return mWrist.getCurrentPosition(); }

    public void update(double hPos, double sPos) {
        hand.setPosition(hPos);
        shoulder.setPosition(sPos);
    }

    public boolean isBusy() {return busy;}

    public void intake() {
        if (astate != ArmState.HOME) return; //intake can only go from home state
        update(hIntake, sMiddle);
        hstate = HandState.PICKUP;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.INTAKE;
    }

    public void deliver3 () {
        if (astate != ArmState.LIFT) return; //
        astate = ArmState.DELIVER;
        update(hHold3, shoulder.getPosition());
        wstate = WristState.LEVEL3;
        moveToTarget(-430, 0.3);
    }

    public void deliver1() {
        if (astate != ArmState.LIFT) return;
        astate = ArmState.DELIVER;
        update(hHold1, shoulder.getPosition());
        wstate = WristState.LEVEL1;
        moveToTarget(-220, 0.2);
    }

    public void deliver2() {
        if (astate != ArmState.LIFT) return;
        astate = ArmState.DELIVER;
        update(hHold2, shoulder.getPosition());
        wstate = WristState.LEVEL2;
        moveToTarget(-350, 0.2);
    }

    public void mWristHome(){
        if(astate != ArmState.LIFT && astate != ArmState.INTAKE) return;
        astate = ArmState.HOME;
        center();
        moveToTarget(0, 0.1);
        mWrist.setPower(0);
        //mWrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void mWristLift(){
        astate = ArmState.LIFT;
        center();
        moveToTarget(-300, 0.2);
    }
    public void moveToTarget(int target, double power) {
        mWrist.setPower(power);
        mWrist.setTargetPosition(target);
        mWrist.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (mWrist.isBusy()){ busy = true; }
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

    public void stop() {mWrist.setPower(0);}

    public boolean outOfRange() {
        double h = getHandPos();
        double s = getShoulderPos();
        return (h < 0.005 || h > 0.995 || s < 0.005 || s > 0.995);
    }

    public void move (Arm3D pos, double increament, int ms) {
        double w = 0.0;
        if (firstCall) {
            firstCall = false;
            timer.reset();
            curr3D = new Arm3D(hand.getPosition(), w, shoulder.getPosition());
            increament3D = new Arm3D (0.01, 0.01, 0.01) ;
            if (curr3D.handP > pos.handP) increament3D.handP = -0.01;
            else if (curr3D.handP == pos.handP) increament3D.handP = 0;
            if (curr3D.wristP > pos.wristP) increament3D.wristP = -0.01;
            else if (curr3D.wristP == pos.wristP) increament3D.wristP = 0;
            if (curr3D.shoulderP > pos.shoulderP) increament3D.shoulderP = -0.01;
            else if (curr3D.shoulderP == pos.shoulderP) increament3D.shoulderP = 0;
            return;
        }
        if (timer.milliseconds() >=  ms) {
            curr3D.add(increament3D);
            //update(curr3D.handP, curr3D.wristP, curr3D.shoulderP);
            timer.reset();
        }
        if (curr3D.reached(pos)) {
            //update(pos.handP, pos.wristP, pos.shoulderP);
            firstCall = true;
            busy = false;
        }
    }

    Arm3D curr3D, increament3D;

    Arm3D[] positions = {
            new Arm3D(0.85, 0.71, 0.57),  // 0  home
            new Arm3D(0.68, 0.71, 0.57),  // 1  intake down
            new Arm3D(0.70, 0.71, 0.57),  // 2  intake flat
            new Arm3D(0.70, 0.73, 0.57),  // 3  intake hold
            new Arm3D(0.75, 0.50, 0.57),  // 4  lift
            new Arm3D(0.40, 0.25, 0.22),  // 6  left level 3 hold
            new Arm3D(0.35, 0.31, 0.22),  // 7  left level 2 hold
            new Arm3D(0.50, 0.38, 0.22),  // 8  left level 1 hold
            new Arm3D(0.00, 0.25, 0.22),  // 9  left level 3 release
            new Arm3D(0.15, 0.31, 0.22),  // 10 left level 2 release
            new Arm3D(0.20, 0.38, 0.22),  // 11 left level 1 release
            new Arm3D(0.75, 0.50, 0.85),  // 6 right

    };

    class Arm3D {
        double handP;
        double wristP;
        double shoulderP;

        public Arm3D(double h, double w, double s) {
            handP = h;
            wristP = w;
            shoulderP = s;
        }
        public String toString() {
            return handP + ", " + wristP + ", " + shoulderP;
        }
        public void add(Arm3D a) {
            handP += a.handP;
            wristP += a.wristP;
            shoulderP += a.shoulderP;
        }
        public boolean reached(Arm3D target) {
            return  Math.abs(target.handP - handP) <= 0.02 &&
                    Math.abs(target.wristP - wristP) <= 0.02 &&
                    Math.abs(target.shoulderP - shoulderP) <= 0.02;
        }

    }

    public void forwardByInch (double inches, double power) {
        int targetPos = (int)(inches * TICKS_PER_INCH);

        mWrist.setPower(power);

        mWrist.setTargetPosition(mWrist.getCurrentPosition() + targetPos);

        mWrist.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if(mWrist.isBusy()){
            busy = true;

        }
        //while (mWrist.isBusy()){
        //    mWrist.getCurrentPosition();
        //}
        //mWrist.setPower(0);
        //resetMode();
        //mwrist position dpad up -480, -844, dpad down -485,
    }
    public void setCwrist(double power) {
        cwrist.setPower(power);
    }

}

