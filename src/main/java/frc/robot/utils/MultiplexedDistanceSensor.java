//                                                 @                             
//                                                  &@@                           
//                          * .                    * @@@                          
//                           * (@   ,                 @@@@                        
//                               @@@*       /          @@@@                       
//                                @@@@@@    @@(     ,* ,@@@@@                     
//                         %@@@@/*  @@@@@@@@       ,**. @@@@@@                    
//                      #********,    @@@@@@@@@@    ***  @@@@@@                   
//                   **********    /    @@@@@@@@@@@@   ,  @@@@@@                  
//                              &@@/  (@  (@@@@@@@@@@@@   @@@@@@@                 
//                            @@@@@//  @@@@@@@@@@@@@@@@@@& @@@@@@@                
//                          @@@@@@@//  @@@@@@@@# .@@@@@@@@@@@@@@@@                
//                         @@@@@@&///  %@@@@@@@@(  *  @@@@@@@@@@@@                
//                       *@@@@@//   @@@@@@@@@@@@@@%     @@@@@@@@@@@               
//                      .@@@@@@@@@@//   .@@@@@@@@@@@@@@  @@@@@@@@@@@              
//                      @@@@@@@@@@@@@@(/     @@@@@@@@@@@@@@@@@@@@@@@@@            
//                   @ %@@@@@@@@@@@@@@   ,  @@@@@@@@@@@@@@@@@@@@@@@@@@@           
//                  @@ @@@@@@@@@@@@@   .             *@@@@@@@@@  @@@@@@#          
//                 @@@ @@@@@@@@@@@@%   *******@@@&///     &@@@@@@@@@@@@@          
//                 @**  @@@@@@@@@@@   ******@@@@@@,          @@@@@@@@@@           
//                 #*** @@@@@@@@@@@   *****@@@@@                  @@@@*           
//                ***   @@@@@@@@@@@  ,****@@@,                                    
//                 *      @@@@@@@@@@.  *****@@                                    
//                          @@@@@@@@@#   ***%@                                    
//                           ,@@@@@@@@@    ***@,  /                               
//                              @@@@@@@@@(    ***   //////*.     */               
//                                 //@@@@@@%/      *    ///////                   
//                                 @    //////////                                
//                                   @@**                                         
//                                       @*****                                   
//                                            

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.I2C;

/**
 * This is a wrapper for the REV 2m Distance Sensor, allowing multiple ones to be
 * used on a multiplexer board (this was tested using the Adafruit TCA9548A).
 * 
 * This helper class was inspired by Team 4776. <b>Go S.C.O.T.S. Bots!</b>
 */

public class MultiplexedDistanceSensor {
    // Change this if your multiplexer has a different address. This is the
    // TCA9548A's default address.
    private final int kMultiplexerAddress = 0x70;

    // The multiplexer I2C is static because it needs to be used for ALL of the
    // multiplexer sensors, and so by making it static all sensors can access it.
    private static I2C multiplexer;

    // The actual sensor. All of the methods call this sensor to get the data.
    private Rev2mDistanceSensor sensor;

    // What port on the multiplexer the distance sensor is plugged into.
    private final int port;

    public enum Unit {
        kInches, kMillimeters
    };

    public enum RangeProfile {
        kDefault, kHighAccuracy, kLongRange, kHighSpeed
    }

    /**
     * Create a multiplexed distance sensor.
     * 
     * @param i2cPort - What port the multiplexer is plugged into.
     * @param port    - What port the distance sensor is plugged into the multiplexer
     *                <br>
     *                (commonly labeled SC3 and SD3 on the PCB, where 3 is the
     *                port)</br>
     */
    public MultiplexedDistanceSensor(I2C.Port i2cPort, int port) {
        if (multiplexer == null) {
            multiplexer = new I2C(i2cPort, kMultiplexerAddress);
        }
        this.port = port;
        setChannel();
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
    }

    /**
     * Helper method. This just sets the multiplexer to the correct port before
     * using the distance sensor.
     */
    private void setChannel() {
        multiplexer.write(kMultiplexerAddress, 1 << port);
    }

    /*-----------------------------------------------------------------------*/
    /* Below are all of the methods used for the distance sensor. */
    /* All this does is set the channel, then run the command on the sensor. */
    /*-----------------------------------------------------------------------*/

    public boolean isEnabled() {
        setChannel();
        return sensor.isEnabled();
    }

    public void setAutomaticMode(boolean enabling) {
        setChannel();
        sensor.setAutomaticMode(enabling);
    }

    public boolean isRangeValid() {
        setChannel();
        return sensor.isRangeValid();
    }

    public double getRange() {
        setChannel();
        return sensor.getRange();
    }

    public double getRange(com.revrobotics.Rev2mDistanceSensor.Unit units) {
        setChannel();
        return sensor.getRange(units);
    }

    public double getTimestamp() {
        setChannel();
        return sensor.getTimestamp();
    }

    public void setEnabled(boolean enable) {
        setChannel();
        sensor.setEnabled(enable);
    }

    public boolean setRangeProfile(com.revrobotics.Rev2mDistanceSensor.RangeProfile profile) {
        setChannel();
        return sensor.setRangeProfile(profile);
    }

    public void setMeasurementPeriod(double period) {
        setChannel();
        sensor.setMeasurementPeriod(period);
    }

    public double getMeasurementPeriod() {
        setChannel();
        return sensor.getMeasurementPeriod();
    }

    public void setDistanceUnits(com.revrobotics.Rev2mDistanceSensor.Unit units) {
        setChannel();
        sensor.setDistanceUnits(units);
    }

    public com.revrobotics.Rev2mDistanceSensor.Unit getDistanceUnits() {
        setChannel();
        return sensor.getDistanceUnits();
    }

    public double GetRange() {
        setChannel();
        return sensor.GetRange();
    }

}