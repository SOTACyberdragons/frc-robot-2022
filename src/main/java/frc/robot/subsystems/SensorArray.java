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
//                                             *                                  

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SensorArray extends SubsystemBase {

    // Create the sensor objects
    TimeOfFlight m_leftSensor = new TimeOfFlight(20);
    TimeOfFlight m_rightSensor = new TimeOfFlight(21);

    MedianFilter leftFilter = new MedianFilter(10);
    MedianFilter rightFilter = new MedianFilter(10);

    double leftMedian;
    double rightMedian;

    /** Creates a new SensorArray. */
    public SensorArray() {
        m_leftSensor.setRangingMode(RangingMode.Short, 0.1);
        m_rightSensor.setRangingMode(RangingMode.Short, 0.1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        leftMedian = leftFilter.calculate(m_leftSensor.getRange());
        rightMedian = rightFilter.calculate(m_rightSensor.getRange());
    }

    public double getDistanceOffset() {
        return (Constants.kShooterSweetSpot - ((leftMedian + rightMedian) / 2));
    }

    public double getAngularOffset() {
        double sensorOffset;
        
        sensorOffset = leftMedian - rightMedian;
        return (Math.atan(sensorOffset / Constants.kSensorSpread)) * 100;
    }

}
