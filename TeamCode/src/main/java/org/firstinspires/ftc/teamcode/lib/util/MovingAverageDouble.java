package org.firstinspires.ftc.teamcode.lib.util;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverageDouble {
    private final int size;
    private final Queue<Double> window = new LinkedList<>();
    private double sum = 0.0;

    /*** Initializes the MovingAverage object with a specified window size.
     * @param size The maximum number of elements in the window.
     */
    public MovingAverageDouble(int size) {
        this.size = size;
    }

    /**
     * Adds a new value to the stream and returns the average of the most recent
     * 'size' values.
     * @param val The new integer value to add.
     * @return The current moving average.
     */
    public double next(double val) {
        // If the window is full, remove the oldest element from the sum and queue
        if (window.size() == size) {
            sum -= window.remove();
        }

        // Add the new value to the queue and the sum
        window.add(val);
        sum += val;

        // The average is the current sum divided by the current number of elements in the window
        return sum / window.size();
    }

    public void clear() {
        window.clear();
        sum = 0.0;
    }
}

