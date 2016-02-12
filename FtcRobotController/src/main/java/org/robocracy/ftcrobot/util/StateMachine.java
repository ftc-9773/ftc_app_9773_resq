package org.robocracy.ftcrobot.util;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.robocracy.ftcrobot.RobotPosition.NavX;
import org.robocracy.ftcrobot.RobotPosition.RobotPosition;

/**
 * Created by Robocracy on 2/10/2016.
 */

enum AutonomousStates {AtDriverStation , OnTheWayToRepairBeacon, IamLost, InTheRepairBeaconZone, InFieldGoal}
enum AutonomousEvents {EnteredRepairZone, DeviatedFromPath, EnteredFieldGoal}

public class StateMachine {
    AutonomousStates initialState;
    AutonomousStates finalState;
    AutonomousStates currentState;

    public StateMachine(AutonomousStates initialState, AutonomousStates finalState) {
        this.initialState = initialState;
        this.currentState = initialState;
        this.finalState = finalState;
    }

    public void process (AutonomousEvents event) {
        switch (this.currentState) {
            case AtDriverStation:
                break;
            case OnTheWayToRepairBeacon:
                break;
            case IamLost:
                break;
            case InTheRepairBeaconZone:
                break;
            case InFieldGoal:
                break;
            default:
                break;
        }
    }

    public AutonomousEvents getNextEvent(RobotPosition expectedPosition) {
        //
    }
}
