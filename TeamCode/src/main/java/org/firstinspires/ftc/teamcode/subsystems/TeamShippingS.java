package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeamShippingS extends Subsystem {
    Servo tshoulder;
    double sPick = 0.78;
    double sHold = 0.25;
    double sRelease = 0.35;
    TSState state;
    double currPos;

    public TSState getState() {
        return state;
    }
    public void setState(TSState state) {
        this.state = state;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        tshoulder  = hardwareMap.get(Servo.class, "tshoulder");
        state = TSState.START;
        currPos = 0;
        //twrist.setPosition(0); // should we pick up at the beginning?
    }

    @Override
    public void initPeriodic() {
    }

    @Override
    public void start() {
    }

    @Override
    public void runPeriodic() {
        currPos = tshoulder.getPosition();
        switch (state) {
            case PICK:
                tshoulder.setPosition(sPick);
                break;
            case HOLD:
                tshoulder.setPosition(sHold);
                break;
            case RELEASE:
                tshoulder.setPosition(sRelease);
                break;
            case START:
                tshoulder.setPosition(currPos + 0.01);
                break;
        }
        telemetry.addLine("[TeamShipping]")
                .addData("State", state)
                .addData("Current", "%f.2", currPos);
    }

    @Override
    public void stop() {

    }

    public enum TSState {
        START, PICK, HOLD, RELEASE;
    }
}
