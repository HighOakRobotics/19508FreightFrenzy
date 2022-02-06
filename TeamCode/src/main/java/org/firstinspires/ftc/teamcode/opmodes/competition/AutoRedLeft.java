package org.firstinspires.ftc.teamcode.opmodes.competition;


import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.CarouselS;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeS;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Rotator;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmS;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;

@Autonomous(name = "Auto Red Left", group = "Quackology")
//@Disabled
public class AutoRedLeft extends SequoiaOpMode {
    DuckDetector duckDetector = new DuckDetector(0, 140, 280);
    private final Mecanum drive = new Mecanum();
    private CarouselS carousel = new CarouselS();
    private IntakeS intake = new IntakeS();
    private SwingArmS arm = new SwingArmS();

    ElapsedTime runtime = new ElapsedTime();

    Pose2d startPos = new Pose2d(12,-36, 0);
    Pose2d intakePos = new Pose2d(48,-63.5, 0);
    Pose2d deliver3Pos = new Pose2d(-12,-32,0);


    Map<Object, Task> positionMap = new HashMap<Object, Task>(){{
        put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER1);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10, deliver3Pos.getY()-7,0))
                        .build())

        ));
        put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER2);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10, deliver3Pos.getY()-7,0))
                        .build())
        ));
        put(DuckDetector.DuckPipeline.DuckPosition.RIGHT, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER3);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(deliver3Pos)
                        .build())
        ));
    }};

    @Override
    public void initTriggers() {
        drive.mecanum().setPoseEstimate(new Pose2d(-33,-63.5));
    }

    @Override
    public void runTriggers() {

        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-51,-63.5,0))
                        .build()),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-62.5,-62.5,Math.PI/4))
                        .build()),
                new InstantTask(() -> carousel.red()),
                new WaitTask(2000,TimeUnit.MILLISECONDS),
                new InstantTask(() -> carousel.pause()),

                new WaitTask(30000,TimeUnit.MILLISECONDS),

                new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LIFT) ),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LEFT) ),

                new SwitchTask(positionMap, () -> position),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.RELEASE) ),
                new WaitTask(200, TimeUnit.MILLISECONDS),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(startPos)
                        .build()),

                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.DELIVER2)),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-66.5,-59.5,-Math.PI/2))
                        .build()),
                new InstantTask(() -> carousel.red()),
                new WaitTask(3),
                new InstantTask(() -> carousel.stop()),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0,-72.5,Math.PI))
                        .build()),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(48,-72.5,Math.PI))
                        .build()),
                new InstantTask(this::requestOpModeStop)
        ));
    }
}
//are we using rotator or carousel
//1. deliver duck
//2. spin carousel
//3. park in warehouse