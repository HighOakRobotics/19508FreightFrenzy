/*package org.firstinspires.ftc.teamcode.opmodes.competition;


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

@Autonomous(name = "Auto Blue Left", group = "Quackology")
//@Disabled

public class AutoBlueLeft extends SequoiaOpMode {
    DuckDetector duckDetector = new DuckDetector(0, 140, 280);
    private final Mecanum drive = new Mecanum();
    private IntakeS intake = new IntakeS();
    private SwingArmS arm = new SwingArmS();

    ElapsedTime runtime = new ElapsedTime();
    Pose2d gatePos = new Pose2d(8.5,67.5, Math.PI);
    Pose2d startPos = new Pose2d(8.5,65, Math.PI);
    Pose2d intakePos = new Pose2d(42,67.5, Math.PI);
    Pose2d deliver3Pos = new Pose2d(-13,36,Math.PI);
    Pose2d parkPos = new Pose2d(42,40,Math.PI);

    Map<Object, Task> positionMap = new HashMap<Object, Task>(){{
        put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER1);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(deliver3Pos.getX(), deliver3Pos.getY()+5.5,Math.PI))
                        .build())
        ));
        put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setMode(SwingArmS.ArmState.DELIVER2);
                }),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(deliver3Pos.getX(),deliver3Pos.getY()+3.5,Math.PI))
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

    SequentialTaskBundle deliverTask = new SequentialTaskBundle(
            new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LIFT) ),
            new WaitTask(500, TimeUnit.MILLISECONDS),
            new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LEFT) ),
            new InstantTask(() -> {
                arm.setMode(SwingArmS.ArmState.DELIVER3);
            }),
            new FollowTrajectoryTask(drive, () -> drive.mecanum()
                    .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                    .lineToLinearHeading(gatePos)
                    .lineToLinearHeading(deliver3Pos)
                    .build()),
            new InstantTask(() -> arm.setMode(SwingArmS.ArmState.RELEASE) ),
            new WaitTask(200, TimeUnit.MILLISECONDS),

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
                    .build())
    );

    SequentialTaskBundle intakeTask = new SequentialTaskBundle(
            new InstantTask( () -> arm.setMode(SwingArmS.ArmState.INTAKE)),
            new InstantTask(() -> intake.in() ),
            new WaitTask(1000, TimeUnit.MILLISECONDS),
            new InstantTask( () -> arm.setMode(SwingArmS.ArmState.HOME)),
            new WaitTask(100, TimeUnit.MILLISECONDS),
            new InstantTask(() -> intake.pause() )
    );

    @Override
    public void initTriggers() {
        drive.mecanum().setPoseEstimate(startPos);
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                //WaitTask(200, TimeUnit.MILLISECONDS),
                new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LIFT) ),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new InstantTask( () -> arm.setMode(SwingArmS.ArmState.LEFT) ),

                new SwitchTask(positionMap, () -> position),
                new WaitTask(1000, TimeUnit.MILLISECONDS),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.RELEASE) ),
                new WaitTask(1000, TimeUnit.MILLISECONDS),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(gatePos)
                        .build()),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.RETRIVE) ),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.DELIVER2) ),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.LIFT) ),
                new InstantTask(() -> arm.setMode(SwingArmS.ArmState.HOME) ),

                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(intakePos)
                        .build()),
                new FollowTrajectoryTask(drive, () -> drive.mecanum()
                        .trajectoryBuilder(drive.mecanum().getPoseEstimate())
                        .lineToLinearHeading(parkPos)
                        .build()),

                new InstantTask(this::requestOpModeStop)
        ));
    }


}*/
