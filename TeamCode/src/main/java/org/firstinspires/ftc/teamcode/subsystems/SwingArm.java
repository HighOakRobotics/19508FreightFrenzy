package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwingArm {
    protected HardwareMap hwMap;
    Servo hand, wrist, shoulder;
    double hRelease = 0.1; //depend on the wrist position
    double hIntake = 0.3; //hold position 0.3 intake
    double hHome = 0.65;//home position too much

    double wHome= 0.7;
    double wLift = 0.5;
    double wLevel1 = 0.6; //lowest;
    double wLevel2 = 0.45; //
    double wLevel3 = 0.2; //highest
    double wTeam = 0; //team element up 0 pposition

    double sMiddle = 0.57; // middle   - more;
    double sLeft = 0.22;//lower
    double sRight = 0.9;

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
        DELIVER1L,
        DELIVER2L,
        DELIVER3L,
        DELIVER1R,
        DELIVER2R,
        DELIVER3R
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

    public void handClose() {hand.setPosition(0.7);}

    public void update(double hPos, double wPos, double sPos) {
        double pos = hand.getPosition();
        hand.setPosition(hPos);
        while (Math.abs(pos - hPos) > 0.05) {}

        pos = wrist.getPosition();
        wrist.setPosition(wPos);
        while (Math.abs(pos - wPos) > 0.001) {}

        shoulder.setPosition(sPos);
        while (Math.abs(pos - sPos) > 0.001) {}
    }

    public void home(){
        if (astate != ArmState.HOME && astate != ArmState.INTAKE)
            update(hHome, wLift, sMiddle);

        update(hHome, wHome, sMiddle);
        hstate = HandState.HOLD;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.HOME;
    }

    public void intake() {
        update(hIntake, wHome, sMiddle);
        hstate = HandState.PICKUP;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.INTAKE;
    }

    public void lift(int level) {
        if (level == -1) { // lift for swing
            update(hand.getPosition(), wLift, shoulder.getPosition());
        }
        else if (level == 1) {
            update(hand.getPosition(), wLevel1, shoulder.getPosition());
        }
        else if (level == 2) {
            update(hand.getPosition(), wLevel2, shoulder.getPosition());
        }
        else if (level == 3) {
            wstate = WristState.LEVEL3;
            for (int i = 1; i <= 4; i++) {
                update(hand.getPosition() - 0.05 , wrist.getPosition() - 0.05, shoulder.getPosition());
            }
        }
        else if (level == 4) {
            update(hand.getPosition(), wTeam, shoulder.getPosition());
        }
        else if (level == 0) {
            update(hand.getPosition(), wHome, shoulder.getPosition());
        }

    }

    public void left() {
        while  (wrist.getPosition() > wLift) lift(-1); //make sure lift before swing
        update (hand.getPosition(), wrist.getPosition(), sLeft);
        sstate = sstate = ShoulderState.LEFT;
    }

    public void right() {
        while  (wrist.getPosition() > wLift) lift(-1);
        update (hand.getPosition(), wrist.getPosition(), sRight);
        sstate = sstate = ShoulderState.RIGHT;
    }

    public void deliver3 (boolean left) {
        if (astate == ArmState.HOME) lift(-1);

        if (left) {
            left();
            sstate = ShoulderState.LEFT;
            astate = ArmState.DELIVER3L;
        }
        else {
            right();
            sstate = ShoulderState.RIGHT;
            astate = ArmState.DELIVER3R;
        }
        lift(3);
    }

    public void deliver1(boolean left) {
        if (left) {

        }
        else {

        }
        hand.setPosition(0.5);
        wrist.setPosition(wHome);
        if(left){ shoulder.setPosition(sLeft); }
        else{shoulder.setPosition(sRight); }

    }

    public void deliver2(boolean left) {

    }

    public void release() {
        hand.setPosition(hRelease);

    }
    public void retrieve(){
        hand.setPosition(hHome);
    }

    public void back() {

    }

    public void test() {
        autoDeliver(2, true);
    }


}

