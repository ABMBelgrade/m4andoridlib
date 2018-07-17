package com.belgrade.advancedbrainmonitoring.m4androidlib;

import android.bluetooth.BluetoothSocket;
import android.os.SystemClock;


import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Class for communicating with the M4 device via bluetooth
 * and collecting acquisition data.
 * Class extends Thread and it is required to call .start() for data collecting to begin.
 *
 * @author milosh@b-alert.com
 * @author pilic@b-alert.com
 */
public class M4DataProducer extends Thread {

    public static final int V1_M4_PACKET_SIZE = 51;
    public static int V1_M4_SLOW_CH_INDEX = 49;

    private final BluetoothSocket mmSocket;
    private final InputStream mmInStream;
    private final OutputStream mmOutStream;
    private boolean mAcqStarted = false;
    private CircularBuffer mCircularBuffer;
    private ReentrantLock lock = new ReentrantLock();
    private int mbBatteryVoltageLOW = 0;
    private int mbBatterVoltageHIGH = 0;
    //Class for communicating with the device
    //Constructor params:
    //            - socket : reference to created bluetooth socket
    //
    //         - bufferSize : declares how many samples do we store in the buffer, before buffer overwrites it self (1000 ~ 4sec of acq data)

    /**
     * M4DataProducer constructor
     * @param socket is a reference to created bluetooth socket that pairs M4 device with android device.
     * @param bufferSize is a number of M4 samples to be stored in a circular buffer.
     */
    public M4DataProducer(BluetoothSocket socket, int bufferSize) {
        mmSocket = socket;
        InputStream tmpIn = null;
        OutputStream tmpOut = null;
        mCircularBuffer = new CircularBuffer(bufferSize);
        // Get the input and output streams, using temp objects because
        // member streams are final
        try {
            tmpIn = socket.getInputStream();
            tmpOut = socket.getOutputStream();
        } catch (IOException e) { }

        mmInStream = tmpIn;
        mmOutStream = tmpOut;
    }

    /**
     * Thread body that reads bytes sent from M4 device to android device bluetooth module.
     * Parses bytes to M4Sample type and adds sample to the circular buffer.
     */
    public void run() {
        int bytes; // bytes returned from read()
        // Keep listening to the InputStream until an exception occurs
        while (true) {
            try {
                // Read from the InputStream
                lock.lock();
                bytes = mmInStream.available();
                if (bytes != 0) {
                    byte[] buffer = new byte[10000];
                    //SystemClock.sleep(50); //pause and wait for rest of data. Adjust this depending on your sending speed.
                    bytes = mmInStream.available(); // how many bytes are ready to be read?
                    bytes = mmInStream.read(buffer, 0, bytes); // record how many bytes we actually read

                    //Parse raw byte data to M4Samples
                    List<M4Sample> samples = ParseBytesToM4Sample(buffer);
                    //Add samples to Circular Buffer
                    for(M4Sample s: samples){
                        mCircularBuffer.add(s);
                    }

                    this.mmOutStream.write(addCheckSumToBaseCommand(new byte[]{0x43, 0x08, 0x00, 0x74, 0x11, 0x00, 0x00}));
                }


                lock.unlock();

            } catch (IOException e) {
                e.printStackTrace();
                lock.unlock();
                break;
            }
        }
    }

    /**
     * Gets data from buffer
     * @return M4Sample[] - returns array of M4Sample
     */

    public M4Sample[] getData(){

        return mCircularBuffer.get();

    }

    /**
     * Sends M4 command to device. Use for custom commands, querying device.
     * See X Series Communication Protocol V2 Specification document, Execute M4 Command for info about M4 commands
     * @param command is a specific code of a M4 command to send to device (example: 0x7424)
     * @param param is a specific parameter for the command (example: 0x01)
     */
    public void SendCommand(int command, int param){
        byte[] mArrSend = new byte[10];
        mArrSend[0] = 0x43;
        mArrSend[1] = 0x08;
        mArrSend[2] = 0x00;

        byte L0Cmd = (byte)(command);
        byte HICmd = (byte)(command >> 8);

        byte L0CmdPa = (byte)(param);
        byte HICmdPa = (byte)(param >> 8);

        mArrSend[3] = HICmd;
        mArrSend[4] = L0Cmd;

        mArrSend[5] = L0CmdPa; //little-endian
        mArrSend[6] = HICmdPa;

        byte [] bytes = addCheckSumToBaseCommand(mArrSend);
        try {
            mmOutStream.write(bytes);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Call this from the main activity to start acquisition
     */
    public void StartAcq() {
        byte[] startAcq;
        //V1 start acquisition command
        startAcq = addCheckSumToBaseCommand(new byte[]{0x56, 0x55, 0x04});
        try {
            mmOutStream.write(startAcq);
            mAcqStarted = true;
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    /**
     * Call this to get channel impedance values.
     * If impedance measure fails, returns null.
     * If acquisition is in progress, returns null.
     * @return array of 4 int values (LP, RP, LF, RF).
     */
    public M4Impedance MeasureImpedances(){
        if(mAcqStarted)
            return null;
        byte[] impedanceCommand;
        M4Impedance impedances = new M4Impedance();
        impedanceCommand = addCheckSumToBaseCommand(new byte[]{0x43, 0x08, 0x00, 0x61, 0x04, 0x04, 0x04});
        try {
            lock.lock();
            mmOutStream.write(impedanceCommand);
            int bytes = 0;
            int timeout = 0;
            while(bytes == 0) {
                SystemClock.sleep(30);
                bytes = mmInStream.available();
                timeout+=30;
                if (timeout>10000)
                    break;
            }
            if (bytes != 0) {
                byte[] response = new byte[50];
                //SystemClock.sleep(50); //pause and wait for rest of data. Adjust this depending on your sending speed.
                bytes = mmInStream.available(); // how many bytes are ready to be read?
                bytes = mmInStream.read(response, 0, bytes); // record how many bytes we actually read

                impedances.LP = Math.min(Math.min(unsignedByte(response[7+2]),unsignedByte(response[7+4])), unsignedByte(response[7+6])); //lp
                impedances.RP = Math.min(unsignedByte(response[7+0]), unsignedByte(response[7+5])); //rp
                impedances.LF = unsignedByte(response[7+1]); //lf
                impedances.RF = unsignedByte(response[7+3]); //rf
                lock.unlock();
                return impedances;

            }
            lock.unlock();
            return null;


        } catch (IOException e) {
            e.printStackTrace();
            lock.unlock();
            return null;
        }
    }

    /**
     * Call this from the main activity to stop acquisition
     */
    public void StopAcq() {

        mAcqStarted = false;
        byte[] stopAcq;
        //V2 stop acquisition command (works with V1 start)
        stopAcq = addCheckSumToBaseCommand(new byte[]{0x43, 0x08, 0x00, 0x74, 0x12, 0x00, 0x00});

        try {
            mmOutStream.write(stopAcq);
        } catch (IOException e) {
            e.printStackTrace();
        }


    }

    /**
     *  Call this from the main activity to shutdown the connection
     */
    public void cancel() {
        try {
            mmSocket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Private method
     * Adds checksum byte to packet that is going to be sent to M4 device
     * @param command is a byte array that needs checksum added
     * @return byte array with added checksum
     */
    private byte[] addCheckSumToBaseCommand(byte[] command){

        byte[] commandWithCS = new byte[command.length+1];

        System.arraycopy(command, 0, commandWithCS, 0, command.length);

        // Calculate check sum for command
        byte sum = 0;
        for (int i = 0; i< command.length-1; i++){
            sum= (byte) (sum + command[i]);
        }

        commandWithCS[command.length] = (byte) (sum ^ 0xFF);
        return commandWithCS;
    }

    /**
     * Converts byte to unsigned int
     * @param b
     * @return
     */
    private int unsignedByte(byte b) {
        return b & 0xFF;
    }

    /**
     * Calculates battery percentage based on battery voltage
     * @param batteryVoltage
     * @return int 0-100
     */
    private int getBatteryPercentage(int batteryVoltage){
        int batteryPercentage = 0;
        batteryPercentage = (int)(100*(((float)batteryVoltage - (float)3600)/(4100-3600)));
        if (batteryPercentage<0)
        {
            batteryPercentage=0;
        }
        if (batteryPercentage>100)
        {
            batteryPercentage=100;
        }
        return batteryPercentage;
    }
    /**
     * Parses M4 acquisition packet to M4Samples
     * @param buffer - packets received from M4 device
     * @return list of M4Samples
     * @see M4Sample
     */
    private List<M4Sample> ParseBytesToM4Sample(byte[] buffer){
        List<M4Sample> samples = new ArrayList<M4Sample>();
        for (int i = 0; i<buffer.length - V1_M4_PACKET_SIZE;i++){
            if (buffer[i]==86 && buffer[i+1] == 85)
            {

            M4Sample sample1 = new M4Sample();
            M4Sample sample2 = new M4Sample();
                int counterHigh = unsignedByte(buffer[i + 48]);
                int counterLow = unsignedByte(buffer[i + 47]);
                sample1.sampleCounter = ((counterHigh <<8) | counterLow)*64 + unsignedByte(buffer[i+2]);
            sample2.sampleCounter = sample1.sampleCounter;
                if (unsignedByte(buffer[i + 2])==33) {
                    mbBatterVoltageHIGH = unsignedByte(buffer[i + V1_M4_SLOW_CH_INDEX]);
            }
                if (unsignedByte(buffer[i + 2])==32) {
                    mbBatteryVoltageLOW = unsignedByte(buffer[i + V1_M4_SLOW_CH_INDEX]);
            }

            int batteryPerc = getBatteryPercentage(mbBatterVoltageHIGH << 8 | mbBatteryVoltageLOW);
            sample1.battery = batteryPerc;
            sample2.battery = batteryPerc;

            int [] val1 = new int [10];
            int [] val2 = new int [10];
                for(int j = 0; j<10; j++)
            {

                    int high1 = unsignedByte(buffer[i + j*2 + 4]);
                    int low1 = unsignedByte(buffer[i + j*2 + 3]);
                    int high2 = unsignedByte(buffer[i + j*2 + 24]);
                    int low2 = unsignedByte(buffer[i + j*2 + 23]);

                    if (j==1 | j == 2 | j == 3){
                        val1[j] = -0x8000 + high1*0x0100 + low1;
                        val2[j] = -0x8000 + high2*0x0100 + low2;
                }
                else {
                        val1[j] = high1*0x0100 + low1;
                        val2[j] = high2*0x0100 + low2;
                }
            }
            sample1.val = val1;
            sample2.val = val2;
            samples.add(sample1);
            samples.add(sample2);
                i+=50;
        }
        }

        return samples;
    }

    /**
     * Get acquisition state
     * @return true - acq running, false - acq stopped.
     */
    public boolean getAcqusitionStatus(){
        return mAcqStarted;
    }
}
