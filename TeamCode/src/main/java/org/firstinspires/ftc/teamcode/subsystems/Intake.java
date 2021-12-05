package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    protected HardwareMap hwMap;
    DcMotorEx intake;
    double power = 0.0;

    public Intake (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        intake = hwMap.get(DcMotorEx.class, "intake");
    }

    public void in() {
        intake.setPower(-0.6);
    }

    public void inSlow() {
        intake.setPower(-.2);
    }

    public void out() {
        intake.setPower(0.6);
    }

    public void stop() {
        intake.setPower(0);
    }

}

