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

package frc.robot;

public class RobotMap {
	// drivetrain
	public static final int RIGHT_MASTER = 2;
	public static final int RIGHT_SLAVE = 3;
	public static final int LEFT_MASTER = 1;
	public static final int LEFT_SLAVE = 0;

	// spinner
	public static final int SPINNER_MOTOR = 7;

	// shooter
	public static final int LEFT_SHOOTER_MOTOR = 4;
	public static final int RIGHT_SHOOTER_MOTOR = 5;

	// robotintake
	public static final int INTAKE_MOTOR = 6;
	public static final int FEEDER_MOTOR = 7;

	// hopper
	public static final int HOPPER_MOTOR = 6;

	// PCM
	public static final int DOUBLE_SOLENOID_ZERO = 0;
	public static final int DOUBLE_SOLENOID_ONE = 7;

	// DIO
	public static final int FEEDER_BREAKBEAM = 1;

	// Pigeon IMU
	public static final int PIGEON_IMU = 14;

	// camera resolution
	public static final int IMG_WIDTH = 320;
	public static final int IMG_HEIGHT = 240;
}