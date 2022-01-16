package org.firstinspires.ftc.teamcode.opmodes.testing;


import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.subsystems.CarouselS;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeS;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Rotator;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmS;
import org.firstinspires.ftc.teamcode.subsystems.TeamShippingS;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.TimedDriveTask;

@Autonomous(name = "S Red Right", group = "Quackology")
//@Disabled

public class AutoRedRight extends SequoiaOpMode {
    DuckDetector duckDetector = new DuckDetector(0, 105, 185);
    private final Mecanum drive = new Mecanum();
    private IntakeS intake = new IntakeS();
    private SwingArmS arm = new SwingArmS();

    ElapsedTime runtime = new ElapsedTime();

    Map<Object, Task> positionMap = new HashMap<Object, Task>(){{
        put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.deliver1();
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-12,-48,0))
                        .build())
        ));
        put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.deliver2();
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-12,-46,0))
                        .build())
        ));
        put(DuckDetector.DuckPipeline.DuckPosition.RIGHT, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.deliver3();
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-12,-44,0))
                        .build())
        ));
    }};


    @Override
    public void initTriggers() {
        arm.wristHome();
        drive.mecanum().setPoseEstimate(new Pose2d(12,-63.5)); // ???need to reset it
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                //new WaitTask(8, TimeUnit.SECONDS),
                new InstantTask( () -> arm.wristLift() ),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new InstantTask( () -> arm.left() ),

                new SwitchTask(positionMap, () -> position),
                new InstantTask(() -> arm.release() ),
                new WaitTask(2, TimeUnit.SECONDS),
                new InstantTask(() -> arm.retrieve() ),
                new InstantTask(() -> arm.wristLift() ),
                new WaitTask(1),
                new InstantTask(() -> arm.wristHome() ),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12,-63.5,0))
                        .build()),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36,-63.5,0))
                        .build()),
                new InstantTask(() -> arm.intake() ),
                new WaitTask(1),
                new InstantTask(() -> intake.in() ),
                new WaitTask(2),
                new InstantTask(() -> arm.wristHome() ),
                new InstantTask(() -> intake.pause()),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12,-63.5, 0))
                        .build()),
                new InstantTask(() -> arm.wristLift() ),
                new WaitTask(1),
                new InstantTask(() -> arm.left() ),
                new InstantTask(() -> arm.deliver3() ),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-12,-44, 0))
                        .build()),
                //again
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12,-63.5,0))
                        .build()),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36,-63.5,0))
                        .build()),
                new InstantTask(() -> arm.intake() ),
                new WaitTask(1),
                new InstantTask(() -> intake.in() ),
                new WaitTask(2),
                new InstantTask(() -> arm.wristHome() ),
                new InstantTask(() -> intake.pause()),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12,-63.5, 0))
                        .build()),
                new InstantTask(() -> arm.wristLift() ),
                new WaitTask(1),
                new InstantTask(() -> arm.left() ),
                new InstantTask(() -> arm.deliver3() ),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-12,-44, 0))
                        .build()),
                new InstantTask(this::requestOpModeStop)
        ));
    }
}
