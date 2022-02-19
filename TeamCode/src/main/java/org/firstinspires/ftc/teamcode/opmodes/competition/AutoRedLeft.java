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

    Pose2d gatePos = new Pose2d(0,-65, 0);
    Pose2d intakePos = new Pose2d(35,-63.5, 0);
    Pose2d deliver3Pos = new Pose2d(-16,-36,0);


    Map<Object, Task> positionMap = new HashMap<Object, Task>(){{
        put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER1);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(deliver3Pos.getX(), deliver3Pos.getY()-10.5,0))
                        .build())

        ));
        put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER2);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(deliver3Pos.getX(), deliver3Pos.getY()-7.5,0))
                        .build())
                //y-position is too much
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
        drive.mecanum().setPoseEstimate(new Pose2d(-30,-65, 0));
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
                        .lineToLinearHeading(new Pose2d(-59,-61.5,Math.PI/4))
                        .build()),
                //new InstantTask(() -> drive.mecanum().turn(Math.PI/4)),
                new InstantTask(() -> carousel.red()),
                new WaitTask(2500,TimeUnit.MILLISECONDS),
                new InstantTask(() -> carousel.pause()),

//                new WaitTask(3000,TimeUnit.MILLISECONDS),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-50,-56,0))
                        .build()),
//                new WaitTask(30000,TimeUnit.MILLISECONDS),
                new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LIFT) ),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LEFT) ),

                new SwitchTask(positionMap, () -> position),
                new WaitTask(1000, TimeUnit.MILLISECONDS),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.RELEASE) ),
                new WaitTask(1000, TimeUnit.MILLISECONDS),
                //new WaitTask(30000,TimeUnit.MILLISECONDS),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(gatePos)
                        .build()),

                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.RETRIVE) ),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.DELIVER2) ),
                new WaitTask(100, TimeUnit.MILLISECONDS),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.LIFT) ),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.HOME) ),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(intakePos)
                        .build()),

                new InstantTask(this::requestOpModeStop)
        ));
    }
}
//are we using rotator or carousel
//1. deliver duck
//2. spin carousel
//3. park in warehouse