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

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SpinUpShooter extends CommandBase {
    // Motor characterization
    private static double kS = 0.56092;
    private static double kV = 0.10938;
    private static double kA = 0.0060046;

    // Motor PID values
    private static double kP = 0.00052791;
    private static double kI = 0;
    private static double kD = 0;

    // Additional motor control setting for the PID controller
    private double velocityTolerance = 2;
    private static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    // Initialize the PID Controller
    public PIDController m_pidController = new PIDController(kP, kI, kD);

    // Expose input variables
    private double m_shooterTargetRPS = 0;

    /** Creates a new SpinShooter. */
    public SpinUpShooter(String profile) {
        String my_profile = profile;
        if (my_profile == "High") {
            this.m_shooterTargetRPS = Constants.kShooterRPSHigh;
        } else if (my_profile == "Low") {
            this.m_shooterTargetRPS = Constants.kShooterRPSLow;
        } else {
            // Default to the high shooter profile
            this.m_shooterTargetRPS = Constants.kShooterRPSHigh;
        }
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidController.setSetpoint(m_shooterTargetRPS);
        m_pidController.setTolerance(velocityTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pidOutput =
                m_pidController.calculate(RobotContainer.m_shooter.getRPS(), m_shooterTargetRPS)
                        + feedForward.calculate(m_shooterTargetRPS);

        // IMPORTANT! Always divide pidOutput by 12 for Falcon 500s
        RobotContainer.m_shooter.setPower(pidOutput / 12);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
