package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwingArm {
    protected HardwareMap hwMap;
    Servo hand, wrist, shoulder;
    double handPos = 0.5; //hold position 0.3 intake
    double wristPos = 0.5; // up 0 pposition
    double sPos = 0.45; // middle;
    public SwingArm (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        hand = hwMap.get(Servo.class, "hand");
        wrist = hwMap.get(Servo.class, "wrist");
        shoulder = hwMap.get(Servo.class, "shoulder");

    }

    public void start() {
        
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

    public void intake() {
        hand.setPosition(0.7);
        wrist.setPosition(0.5);
    }

    public void lift() {
        hand.setPosition(0.5);
        wrist.setPosition(0.5);
        if  (wrist.getPosition() > 0.5) {}
        else shoulder.setPosition(0.5);
    }

    public void left() {
        if  (wrist.getPosition() > 0.5) {}
        else shoulder.setPosition(0.3);
    }

    public void right() {
        if  (wrist.getPosition() > 0.5) {}
        else shoulder.setPosition(0.7);
    }

    public void level1() {

    }

    public void level2() {

    }

    public void level3() {

    }

    public void release() {
        hand.setPosition(0);
    }

    public void back() {

    }

    public void test() {
        autoDeliver(2, true);
    }


}

