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
    boolean rightClimberAvailable = false;

    public RightClimber(FTCRobot robot, Servo rightClimber, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        if (rightClimber != null) {
            rightClimberAvailable = true;
            this.rightClimber = rightClimber;
            this.rightClimber.scaleRange(0.1, 0.5);
            this.rightClimber.setPosition(1.0); // 1.0 == 0.5 because of scaleRange() call above
            DbgLog.msg(String.format("RightClimber Position = %f", this.rightClimber.getPosition()));
        }
    }

    /**
     * Moves right climber servo based on {@code enum rightClimberDirection} value set in {@link DriverStation#getNextClimberCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!rightClimberAvailable) {
            return;
        }
        double rightClimberPosition = rightClimber.getPosition();
        DbgLog.msg(String.format("RightClimber Position = %f", rightClimberPosition));
        switch (drvrcmd.rightClimberCmd.rightClimberDirection){
            case DOWN:
                rightClimber.setPosition(Range.clip(rightClimberPosition+0.01, 0, 1));
                break;
            case UP:
                rightClimber.setPosition(Range.clip(rightClimberPosition-0.01, 0, 1));
                break;
            case NONE:
                break;
            default:
                break;
        }
    }
}
