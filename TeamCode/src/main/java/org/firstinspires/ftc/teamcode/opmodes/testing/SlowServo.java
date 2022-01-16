package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlowServo {
    protected HardwareMap hwMap;
    ElapsedTime timer;
    double curr;
    boolean firstCall;
    Servo wrist;
    public boolean busy;

    double increment;
    int cycle_ms;

    //static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp;


    public SlowServo (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        wrist = hwMap.get(Servo.class, "wrist");
        timer = new ElapsedTime();
        firstCall = true;
        wrist.setPosition(position);
        rampUp = true;
        busy = false;
    }

    public double getPosition() {return wrist.getPosition();}

    public void slowSetPostion(double end, double increment, int cycle )  { //did not work
        Servo s = this.wrist;
        this.increment = increment;
        this.cycle_ms = cycle;

        if (firstCall) {
            firstCall = false;
            timer.reset();
            curr = s.getPosition();
            busy = true;
        }
        else {
            if (timer.milliseconds() >= cycle) {
                curr += increment;
                s.setPosition(curr);
                timer.reset();
            }
            if (Math.abs(curr - end) < 0.001) {
                firstCall = true;
                busy = false;
            }
        }
    }
}
