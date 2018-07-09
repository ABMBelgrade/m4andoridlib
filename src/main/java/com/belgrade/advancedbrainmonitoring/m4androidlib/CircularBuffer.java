package com.belgrade.advancedbrainmonitoring.m4androidlib;

import java.util.concurrent.locks.ReentrantLock;

/**
 * This is a model class of a circular buffer
 * @author milosh@b-alert.com
 * @author pilic@b-alert.com
 */

public class CircularBuffer {

    private M4Sample [] buffer;
    private int tail;
    private int head;
    private ReentrantLock lock = new ReentrantLock();

    /**
     * CircularBuffer constructor
     * @param n is a buffer size.
     */
    public CircularBuffer(int n) {
        buffer = new M4Sample[n];
        tail = 0;
        head = 0;
    }

    /**
     * Add M4Sample to the buffer.
     * Rewrites oldest element in the buffer is buffer size is reached.
     * @param toAdd M4Sample to be added to buffer.
     */
    public void add(M4Sample toAdd) {
        lock.lock();

        try {
            if (head != (((tail-1) % buffer.length)+buffer.length)%buffer.length) {
                buffer[head++] = toAdd;
            } else {

                tail = (tail+1) % buffer.length;
                buffer[head++] = toAdd;
            }
            head = head % buffer.length;
        } finally {
            lock.unlock();
        }
    }

    /**
     * Get all samples in buffer and empties buffer.
     * @return array of M4Samples
     */
    public M4Sample [] get(){
        lock.lock();
            int size;
            if (head == (((tail-1) % buffer.length)+buffer.length)%buffer.length) {
                size = buffer.length - 1;
                M4Sample [] data =  new M4Sample[size];
                for (int i = 0; i < size; i++) {
                    data[i] = buffer[(tail+i)%buffer.length];
                }
                clearBuffer();
                lock.unlock();
                return data;
            } else {
                size = head - tail;
                M4Sample [] data =  new M4Sample[size];

                for (int i = 0; i < size; i++) {
                    data[i] = buffer[tail+i];
                }
                clearBuffer();
                lock.unlock();
                return data;
            }



    }

    /**
     * Clear buffer, resets head and tail to 0.
     */
    private void clearBuffer(){
        tail = 0;
        head = 0;
    }

}
