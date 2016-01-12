package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * @author Team Robocracy
 *
 * Operates latches on robot that hold on to churros
 */
public class RightClimber {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo rightClimber;

    public RightClimber(FTCRobot robot, Servo rightClimber, LinearOpMode curOpMode){
        this.curOpMode = curOpMode;
        this.rightClimber = rightClimber;
        this.robot = robot;
    }

    /**
     * Moves right climber servo based on {@code enum rightClimberDirection} value set in {@link DriverStation#getNextClimberCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        switch (drvrcmd.rightClimberCmd.rightClimberDirection){
            case DOWN:
                rightClimber.setPosition(0.5);
                break;
            case UP:
                rightClimber.setPosition(0);
                break;
            case NONE:
                break;
            default:
                break;
        }
    }
}
