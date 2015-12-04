package org.robocracy.ftcrobot.util;


/**
 * Created by Robocracy on 11/17/2015.
 */
public class PIDController {
    double Kp, Ki,Kd;
    CircularQueue cirQ;
    double setPoint, errorMax;

    public PIDController(double Kp, double Ki, double Kd, int ArrayLen,
                         double setPoint, double errorMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.cirQ = new CircularQueue(ArrayLen);
        this.setPoint = setPoint;
        this.errorMax = errorMax;
    }

    public double getCorrection(double curValue) {
        double correction = 0.0;
        // Calculate error
        double error = setPoint - curValue;
        // Save error to queue
        this.cirQ.add(error);
        // Calculate Correction
        correction = (Kp * error) + (Ki * this.cirQ.average()) + (Kd * (error - this.cirQ.getLatestValue()));
        // Return the correction
        return (correction);
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void reset(){
        this.Kp = 0.0;
        this.Ki = 0.0;
        this.Kd = 0.0;
        this.setPoint = 0.0;
        this.errorMax = 0.0;
        this.cirQ.reset();
    }
}
