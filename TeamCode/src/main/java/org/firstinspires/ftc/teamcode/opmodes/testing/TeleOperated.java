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
import org.firstinspires.ftc.teamcode.subsystems.SwingArmS;
import org.firstinspires.ftc.teamcode.subsystems.TeamShipping;
import org.firstinspires.ftc.teamcode.subsystems.TeamShippingS;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.tasks.TeamShippingCycleTask;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "QuackDelivery TeleOp", group = "Quackology")
//@Disabled
public class TeleOperated extends SequoiaOpMode {
    private final Mecanum drivetrain = new Mecanum();
    private IntakeS intake = new IntakeS();
    private SwingArmS arm = new SwingArmS();
    private TeamShippingS teamShipping = new TeamShippingS();
    private CarouselS carousel = new CarouselS();

    @Override
    public void initTriggers() {
    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPressWithCancel(new GamepadDriveTask(gamepad1, drivetrain));

        gamepad1H.aButton().onPress(new InstantTask( () -> {carousel.pause();}));
        gamepad1H.bButton().onPress(new InstantTask( () -> carousel.red()));
        gamepad1H.xButton().onPress(new InstantTask( () -> carousel.blue()));

        gamepad1H.rightButton().onPress(new InstantTask(() -> {intake.in();}));
        gamepad1H.leftButton().onPress(new InstantTask(() -> {intake.out();}));
        gamepad1H.downButton().onPress(new InstantTask(() -> {intake.pause();}));

        gamepad1H.rightBumperButton().onPress(new InstantTask(() -> {teamShipping.setState(TeamShippingS.TSState.UP);}));
        gamepad1H.leftBumperButton().onPress(new InstantTask(() -> {teamShipping.setState(TeamShippingS.TSState.DOWN);}));
        gamepad1H.yButton().onPress(new TeamShippingCycleTask(teamShipping));

        gamepad2H.downButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.HOME);}));
        gamepad2H.upButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.LIFT);}));
        gamepad2H.rightButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.RIGHT);}));
        gamepad2H.leftButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.LEFT);}));
        gamepad2H.aButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.INTAKE);}));
        gamepad2H.bButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.DELIVER1);}));
        gamepad2H.xButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.DELIVER2);}));
        gamepad2H.yButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.DELIVER3);}));
        gamepad2H.rightBumperButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.RELEASE);}));
        gamepad2H.leftBumperButton().onPress(new InstantTask( () -> {arm.setMode(SwingArmS.ArmState.RETRIVE);}));
    }
}
