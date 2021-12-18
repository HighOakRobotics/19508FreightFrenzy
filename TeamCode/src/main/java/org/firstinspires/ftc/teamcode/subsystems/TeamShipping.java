package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeamShipping {
    protected HardwareMap hwMap;
    Servo twrist, tshoulder;
    boolean busy;
    double wPick = 0.5;
    double sPick = 0;
    double wHold = 0.7;
    double sHold = 0.8;
    double wRelease = 0.2;
    double sRelease = 0.7;

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

    public void update(double hPos, double wPos, double sPos) {
        if (busy) return;

        twrist.setPosition(wPos);
        tshoulder.setPosition(sPos);
        if ( Math.abs(twrist.getPosition() - wPos ) > 0.01
                || Math.abs(tshoulder.getPosition() - sPos ) > 0.01) {
            busy = true;
        }
        else
            busy = false; //done
    }

    public void pickup() {
        twrist.setPosition(wPick);
        tshoulder.setPosition(sPick);
    }

    public void hold() {
        twrist.setPosition(wHold);
        tshoulder.setPosition(sHold);
    }

    public void release() {
        twrist.setPosition(wRelease);
        tshoulder.setPosition(sRelease);
    }
}
