package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeamShippingS extends Subsystem {
    Servo tshoulder;
    double sStart = 0.88; //???
    double sPick = 0.78;
    double sHold = 0.25;
    double sRelease = 0.35;
    TSState state;
    double currPos;
    double target;

    public TSState getState() {
        return state;
    }
    public void setState(TSState state) {
        update(state);
    }

    public void update(TSState state) {
        if(state != TeamShippingS.TSState.UP || state != TeamShippingS.TSState.DOWN)
            this.state = state;


        switch (state) {
            case PICK:
                target = sPick;
               break;
            case HOLD:
                target = sHold;
                break;
            case RELEASE:
                target = sRelease;
                break;
            case START:
                target = sStart;
                break;
            case UP:
                target -= 0.02;
                break;
            case DOWN:
                target += 0.02;
                break;
        }
    }
    @Override
    public void initialize(HardwareMap hardwareMap) {
        tshoulder  = hardwareMap.get(Servo.class, "tshoulder");
        state = TSState.START;
        currPos = sStart;
        target = sStart;
        tshoulder.setPosition(sStart);
    }

    @Override
    public void initPeriodic() {
    }

    @Override
    public void start() {
    }

    @Override
    public void runPeriodic() {
        tshoulder.setPosition(target);

        telemetry.addLine("[TeamShipping]")
                .addData("State", state)
                .addData("Current", "%f.2", target);
    }

    @Override
    public void stop() {

    }

    public enum TSState {
        START, PICK, HOLD, RELEASE, UP, DOWN;
    }
}
