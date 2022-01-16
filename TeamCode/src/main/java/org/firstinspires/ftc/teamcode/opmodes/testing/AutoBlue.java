package org.firstinspires.ftc.teamcode.opmodes.testing;


import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Blue", group = "Working Title")
@Disabled
public class AutoBlue extends SequoiaOpMode {
    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                new WaitTask(365, TimeUnit.DAYS)
        ));
    }
}
