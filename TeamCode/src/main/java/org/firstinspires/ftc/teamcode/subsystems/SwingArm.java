package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwingArm {
    protected HardwareMap hwMap;
    Servo hand, wrist, shoulder;
    double hIntake = 0.65; //0.6; //hold position 0.3 intake
    double hHome = 0.9;//home position too much
    double hRelease3 = 0.0; //depend on the wrist position
    double hRelease2 = 0.2;
    double hRelease1 = 0.25;
    double hHold3 = 0.4;
    double hHold2 = 0.55;
    double hHold1 = 0.6;
    double hAHold = 0.5;


    double wHome= 0.67;
    double wIntake = 0.65;
    double wLevel1 = 0.4; //lowest;
    double wLevel2 = 0.3; // up 0 position
    double wLevel3 = 0.15;
    double wALevel1 = 0.38;
    double wLift = wLevel2;
    double wTeam = 0; //team element up 0 pposition

    double sMiddle = 0.57; // middle   - more;
    double sLeft = 0.22;//lower
    double sRight = 0.85; // no wire now 0.9;

    boolean busy = false;

    public enum HandState {
        HOLD,
        PICKUP,

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
        wrist = hwMap.get(Servo.class, "wrist");
        shoulder = hwMap.get(Servo.class, "shoulder");
        astate = ArmState.HOME;
    }

    public double getHandPos() {return hand.getPosition(); }
    public double getWristPos() {return wrist.getPosition();}
    public double getShoulderPos() {return shoulder.getPosition(); }

    public void autoDeliver(int level, boolean left) {
        hand.setPosition(0.5);
        double wPos = 0.5;
        double sPos = 0.5;
        if (level == 1) {
            wPos = 0.7;
        }
        else if (level == 2) {
            wPos = 0.5;
        }
        else if (level == 3) {
            wPos = 0.3;
        }
        wrist.setPosition(wPos);
        if (left) {
            sPos = 0.2;
        }
        else sPos = 0.8;

        while (wrist.getPosition() > wPos) {}
        if  (wrist.getPosition() > wPos) {}
        else shoulder.setPosition(sPos);
        while (Math.abs(shoulder.getPosition() - sPos) > 0.01) {}
        hand.setPosition(0.1);

    }

    public void update(double hPos, double wPos, double sPos) {
        if (busy) return;

        hand.setPosition(hPos);
        //slowSetPostion(hand, hand.getPosition(), hPos);
        wrist.setPosition(wPos);
        shoulder.setPosition(sPos);
        if ( Math.abs(hand.getPosition() - hPos ) > 0.01
                || Math.abs(wrist.getPosition() - wPos ) > 0.01
                || Math.abs(shoulder.getPosition() - sPos ) > 0.01) {
            busy = true;
        }
        else
            busy = false; //done
    }

    public boolean isBusy() {return busy;}

    public void home(){
        if (astate != ArmState.HOME && astate != ArmState.INTAKE && Math.abs(getWristPos() - wLift) > 0.01) return;
        if (busy) return;

        update(hHome, wHome, sMiddle);
        hstate = HandState.HOLD;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.HOME;
    }

    public void intake() {
        if (astate != ArmState.HOME) return;

        update(hIntake, wIntake, sMiddle);
        hstate = HandState.PICKUP;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.INTAKE;
    }

    public void lift(int level) {
        if (level == -1) { // lift for swing
            update(hHome, wLift, sMiddle);
            //if (getWristPos() < 0.4) update(hHome - 0.15, wLift, sMiddle);
        }
        else if (level == 1) {
            wstate = WristState.LEVEL1;
            update(hHold1, wLevel1, shoulder.getPosition());
        }
        else if (level == 2) {
            wstate = WristState.LEVEL2;
            update(hHold2 , wLevel2, shoulder.getPosition());
        }
        else if (level == 3) {
            wstate = WristState.LEVEL3;
            update(hHold3 , wLevel3, shoulder.getPosition());
        }
        else if (level == 4) {
            update(hand.getPosition(), wTeam, shoulder.getPosition());
        }
        else if (level == 0) {
            update(hand.getPosition(), wHome, shoulder.getPosition());
        }
        else if(level == 11){
            wstate = WristState.LEVEL1;
            update(hAHold, wALevel1, shoulder.getPosition());
        }

    }

    public void left() {
        while  (wrist.getPosition() > wLift) lift(-1); //make sure lift before swing
        update (hand.getPosition(), wrist.getPosition(), sLeft);
        sstate = ShoulderState.LEFT;
    }

    public void right() {//change because of cable 12/12
        while  (wrist.getPosition() > wLift) lift(-1);
        update (hand.getPosition(), wrist.getPosition(), sRight);
        sstate = ShoulderState.RIGHT;
    }

    public void deliver3 () {
        lift(3);
        astate = ArmState.DELIVER;

    }

    public void deliver1() {
        lift(1);
        astate = ArmState.DELIVER;
    }

    public void deliver2() {
        lift(2);
        astate = ArmState.DELIVER;
    }

    public void autoDeliver1() {
        lift(11);
        astate = ArmState.DELIVER;
    }

    public void release() {
        if (wstate == WristState.LEVEL3) hand.setPosition(hRelease3);
        if (wstate == WristState.LEVEL2) hand.setPosition(hRelease2);
        if (wstate == WristState.LEVEL1) hand.setPosition(hRelease1);
    }
    
    public void retrieve(){
        hand.setPosition(hHome);
        //if (wstate == WristState.LEVEL1) wrist.setPosition(wLevel2);
    }

    public void test(int indicator, double amount) {
        double hpos = hand.getPosition();
        double wpos = wrist.getPosition();
        double spos = shoulder.getPosition();

        if (indicator == 1) {
            //test hand position
            hand.setPosition(hHome + amount);
        }
        if (indicator == 2) {
            //test wrist position
            wrist.setPosition(wHome + amount);
        }
        if (indicator == 3) {
            //test shoulder position
            shoulder.setPosition(sMiddle + amount);
        }
    }

    public void slowSetPostion(Servo s, double begin, double end )  {
        while (Math.abs(s.getPosition() - end) > 0.00001 ) {
            s.setPosition(begin + (end - begin) / 1000);
        }
    }


}

