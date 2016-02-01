package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Operates climber dispenser on robot
 * @author Team Robocracy
 */
public class ClimberDispenser {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo climberDispenserServo;
    boolean climberDispenserServoAvailable = false;

    public ClimberDispenser(FTCRobot robot, Servo climberDispenserServo, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.climberDispenserServo = climberDispenserServo;
        if (climberDispenserServo != null) {
            DbgLog.msg(String.format("Left climber position = %f", this.climberDispenserServo.getPosition()));
            this.climberDispenserServo.scaleRange(0.156, 0.705);
            this.climberDispenserServo.setDirection(Servo.Direction.REVERSE);
            this.climberDispenserServo.setPosition(0);
            this.climberDispenserServoAvailable = true;
        }
    }

    /**
     * Moves climber dispenser servo based on {@code enum climerDispenserStatus} value set in {@link DriverStation#getNextClimberCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!climberDispenserServoAvailable) {
            return;
        }
        double climberDispenserServoPosition = climberDispenserServo.getPosition();
        DbgLog.msg(String.format("climberDispenser Position = %f", climberDispenserServoPosition));
        switch (drvrcmd.climberDispenserCommand.climberDispenserStatus){
            case -1:
                climberDispenserServo.setPosition(1);
                break;
            case 1:
                climberDispenserServo.setPosition(0);
                break;
            case 0:
                break;
            default:
                break;
        }
    }
}
