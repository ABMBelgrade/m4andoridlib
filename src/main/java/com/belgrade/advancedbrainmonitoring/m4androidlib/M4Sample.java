package com.belgrade.advancedbrainmonitoring.m4androidlib;

/**
 * This is a model class of a sample for M4 device.
 * @author milosh@b-alert.com
 * @author pilic@b-alert.com
 */
public class M4Sample {
    /**
     * 2 byte counter, range 0 - 65535.
     */
    public int sampleCounter;
    public int battery;
    /**
     * Contains EEG values
     * "Alpha Threshold","LpRp","LfLp","RfRp","PulseRate","Ired", "IRed Movement", "Alpha power", "Theta Movement", "Movement Threshold"
     */
    public int[] val;

}
