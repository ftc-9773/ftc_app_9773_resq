package org.robocracy.ftcrobot.DriveSystem;

/**
 * @author Team Robocracy
 */
public class OmniWheel extends Wheel {
    public OmniWheel(double diameter, double frictionCoeffMatStraight, double frictionCoeffMatStrafe){
        super(WheelType.Omni, diameter, frictionCoeffMatStraight, frictionCoeffMatStrafe);
    }
}
