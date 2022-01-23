package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.InstantTask;

import org.firstinspires.ftc.teamcode.subsystems.TeamShippingS;

import java.util.HashMap;
import java.util.Map;

public class TeamShippingCycleTask extends InstantTask {
    static Map<TeamShippingS.TSState, TeamShippingS.TSState> map = new HashMap<TeamShippingS.TSState, TeamShippingS.TSState>() {{
        put(TeamShippingS.TSState.START, TeamShippingS.TSState.PICK);
        put(TeamShippingS.TSState.PICK, TeamShippingS.TSState.HOLD);
        put(TeamShippingS.TSState.HOLD, TeamShippingS.TSState.RELEASE);
        put(TeamShippingS.TSState.RELEASE, TeamShippingS.TSState.PICK);
        put(TeamShippingS.TSState.UP, TeamShippingS.TSState.HOLD);
        put(TeamShippingS.TSState.DOWN, TeamShippingS.TSState.PICK);
    }};

    public TeamShippingCycleTask(TeamShippingS teamShippingS) {
        super(() -> {
            teamShippingS.setState(map.get(teamShippingS.getState()));
        });
    }
}
