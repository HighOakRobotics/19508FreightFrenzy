package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.CarouselS;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeS;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Rotator;
import org.firstinspires.ftc.teamcode.subsystems.SwingArm;
import org.firstinspires.ftc.teamcode.subsystems.TeamShipping;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "QuackDelivery TeleOp", group = "Quackology")
//@Disabled
public class TeleOperated extends SequoiaOpMode {
    private final Mecanum drivetrain = new Mecanum();
    private IntakeS intake = new IntakeS();
    private SwingArm arm;
    private TeamShipping teamShipping;
    private CarouselS carousel = new CarouselS();

    @Override
    public void initTriggers() {
    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPressWithCancel(new GamepadDriveTask(gamepad1, drivetrain));

        gamepad1H.rightButton().onPress(new InstantTask(() -> {intake.in();}));
        gamepad1H.leftButton().onPress(new InstantTask(() -> {intake.out();}));
        gamepad1H.downButton().onPress(new InstantTask(() -> {intake.pause();}));

    }
}
