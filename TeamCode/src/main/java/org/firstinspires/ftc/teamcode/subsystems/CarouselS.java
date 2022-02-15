package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CarouselS extends Subsystem {
    DcMotorEx carousel;
    static final double ratio = 5; // Carousel is 15 in, wheel is 3 in diameter.
    double setpoint; //?
    double power;

    public double getSetpoint() {
        return setpoint;
    }
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void blue() { power = 0.6; }
    public void inSlow() { power = -.2; }
    public void red() { power = -0.6; }
    public void pause() { power = 0.0; }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //carousel.setVelocity(0, AngleUnit.DEGREES);
        power = 0;
        setpoint = 0;
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        carousel.setPower(power);
        //carousel.setVelocity( ratio * setpoint, AngleUnit.DEGREES);
    }

    @Override
    public void stop() {
        carousel.setPower(0.0);
        //carousel.setVelocity(0);
    }
}