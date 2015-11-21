package org.robocracy.ftcrobot.DriverStation;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class DriverCommand {
    public static enum MoveCommands {MOVE_FWD, MOVE_BWD, STRAFE_LEFT, STRAFE_RIGHT,
                    MOVE_FED_LEFT, MOVE_FWD_RIGHT, MOVE_BWD_LEFT, MOVE_BWD_RIGHT};
    public static enum TurnCommands {TURN_LEFT, TURN_RIGHT}

    MoveCommands mvCmd;
    double moveSpeed;
    TurnCommands turnCmd;
    double turnAngle;
}
