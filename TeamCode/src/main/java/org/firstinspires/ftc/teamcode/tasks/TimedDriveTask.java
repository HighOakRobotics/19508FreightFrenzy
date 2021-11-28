package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

public class TimedDriveTask  extends StartEndTask {


    public TimedDriveTask(ElapsedTime runtime, Mecanum drive) {
        super(() -> {
            drive.mecanum().setDriveDST(
                    () -> runtime.seconds() < 5 ? 0.5 : 0,
                    () -> true ? 0 : 0,
                    () -> true ? 0 : 0
            );
        }, () -> {
            drive.mecanum().idle();
        });
    }
}