package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeamShipping {
    protected HardwareMap hwMap;
    Servo twrist,tshoulder ;
    //CRServo tshoulder;
    boolean busy;
    double wPick = 1.0;
    double wHold = 0.7;
    double wRelease = 0.2;
    double sPick = 0.78;
    double sHold = 0.25;
    double sRelease = 0.35;

    double [] sPos = {0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.25, 0.275,0.275,
                      0.3,  0.325, 0.35, 0.375, 0.4, 0.425, 0.45, 0.475, 0.5, 0.5, 0.5};
    double [] wPos = { 1.0, 1.0,   0.95, 0.95,  0.9, 0.9,  0.9,  0.9,   0.85,  0.85,
                       0.75, 0.75, 0.7,  0.7,  0.6,  0.6,  0.5, 0.4,   0.4,   0.4, 0.2};
    public TeamShipping (HardwareMap hwMap){
        this.hwMap = hwMap;
        busy = false;
    }

    public void init() {
        twrist = hwMap.get(Servo.class, "twrist");
        tshoulder = hwMap.get(Servo.class, "tshoulder");

    }

    public double getWristPos() {return twrist.getPosition();}
    public double getShoulderPos() {return tshoulder.getPosition(); }
    //public double getShoulderPos() {return tshoulder.getPower(); }

    public void pickup() {
        tshoulder.setPosition(sPick + 0.1);
    }

    public void up() {
        tshoulder.setPosition(sHold);
    }
    public void down() {
        tshoulder.setPosition(sPick);
        twrist.setPosition(wPick);
    }

    public void release() {
        twrist.setPosition(wRelease);
        tshoulder.setPosition(sRelease);
    }

    public void detach(){
        tshoulder.setPosition(sRelease + 0.15);
    }

    public void wrist(double amount) {
        if (amount >= -0.001 ) twrist.setPosition(wPick - amount);
        else twrist.setPosition(wRelease + amount);
    }

    public void shoulder(double amount) {
        if (amount >= -0.001) tshoulder.setPosition(sPick + amount);
        else tshoulder.setPosition(sHold - amount);
    }

    public void comb(double amount) {
        int i = (int) (amount * 20 );
        twrist.setPosition(wPos[i]);
        tshoulder.setPosition(sPos[i]);
    }
}
