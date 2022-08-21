package org.firstinspires.ftc.teamcode.afterseason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.CarouselS;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeS;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
//import org.firstinspires.ftc.teamcode.subsystems.Rotator;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmS;
//import org.firstinspires.ftc.teamcode.subsystems.TeamShipping;
//import org.firstinspires.ftc.teamcode.subsystems.TeamShippingS;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;
//import org.firstinspires.ftc.teamcode.tasks.TeamShippingCycleTask;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "After Season Gun")
//@Disabled
public class gun extends OpMode {

    DcMotorEx gun;

    @Override
    public void init() {
        gun = hardwareMap.get(DcMotorEx.class, "gun");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            gun.setPower(1.0);
        }
        if (gamepad1.b) {
            gun.setPower(0);
        }
    }
}