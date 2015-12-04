package org.robocracy.ftcrobot.util;

/**
 * Created by Robocracy on 11/17/2015.
 */
public class CircularQueue {
    // This is a placeholder for implementing circular queue of double type values
    // in Java.  This will be used by PID controller.
    private double[] circ_queue;
    private int index;
    private double sum;

    public  CircularQueue(int len) {
        this.circ_queue = new double[len+1];
        for (int i=0; i<len; i++) {
            this.circ_queue[i] = 0.0;
        }
        this.index = 0; // Index of the next available element
        this.sum = 0.0;
    }

    public void add(double elem) {
        double newSum = this.sum - this.circ_queue[this.index] + elem;
        this.circ_queue[this.index] = elem;
        this.index = (this.index + 1) % this.circ_queue.length;
        this.sum = newSum;
    }

    public double average() {
        return (this.sum / this.circ_queue.length);
    }

    public void reset() {
        this.circ_queue = null;
        this.index = 0;
        this.sum = 0.0;
    }

    public double getLatestValue() {
        int latestIndex = (this.index + this.circ_queue.length - 1) % this.circ_queue.length;
        return (this.circ_queue[latestIndex]);
    }
}
