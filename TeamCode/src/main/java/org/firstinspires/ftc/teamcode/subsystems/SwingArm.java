package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwingArm {
    protected HardwareMap hwMap;
    Servo hand, wrist, shoulder;
    double hIntake = 0.3; //hold position 0.3 intake
    double hHome = 0.65;//home position too much
    double hRelease = 0.1;
    double wLevel2 = 0.45; // up 0 pposition
    double wHome= 0.7;
    double wLevel3 = 0.2;
    double sPos = 0.57; // middle   - more;
    double sLeft = 0.22;//lower
    double sRight = 0.9;
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

    public void home(){
        hand.setPosition(hHome);
        wrist.setPosition(wHome);
        shoulder.setPosition(sPos);
    }
    public void intake() {
        hand.setPosition(hIntake);
        wrist.setPosition(wHome + 0.5);//higher
        shoulder.setPosition(sPos);
    }
    public void hold(){
        hand.setPosition(0.4);
        wrist.setPosition(0.55);
        shoulder.setPosition(sPos);
    }

    public void lift() {
        hand.setPosition(hHome);
        wrist.setPosition(wLevel2);
        if  (wrist.getPosition() > wLevel2) {}
        else shoulder.setPosition(sPos);
    }

    public void left() {
        if  (wrist.getPosition() > wLevel2) {}
        else shoulder.setPosition(sLeft);
    }

    public void right() {
        if  (wrist.getPosition() > wLevel2) {}
        else shoulder.setPosition(sRight);
    }

    public void level1(boolean left) {
        hand.setPosition(0.5);
        wrist.setPosition(wHome);
        if(left){ shoulder.setPosition(sLeft); }
        else{shoulder.setPosition(sRight); }

    }

    public void level2() {

    }

    public void level3(boolean left) {
        hand.setPosition(0.5);
        wrist.setPosition(wLevel3);
        if(left){ shoulder.setPosition(sLeft); }
        else{shoulder.setPosition(sRight); }

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

