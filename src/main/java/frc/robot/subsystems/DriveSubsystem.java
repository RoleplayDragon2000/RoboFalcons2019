/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //Instantiate Moter Controllers
  public PWMVictorSPX L1_leftMaster = new PWMVictorSPX(RobotMap.L1_leftMasterPort);
  public PWMVictorSPX L2_leftSlave = new PWMVictorSPX(RobotMap.L1_leftMasterPort);
  public PWMVictorSPX R1_rightMaster = new PWMVictorSPX(RobotMap.L1_leftMasterPort);
  public PWMVictorSPX R2_rightSlave = new PWMVictorSPX(RobotMap.L1_leftMasterPort);

  //Group Speed Controllers
  SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(L1_leftMaster, L2_leftSlave);
  SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(R1_rightMaster, R2_rightSlave);

  //Instantiate Differential Drive System, Assign Motor Controllers
  public DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  //Add Drive() method
  public void manuelDrive(double move, double turn){
    if (move > .5) move = .5;
    if (turn > .5) turn = .5;
    if (Math.abs(move) < 0.10) move = 0;
    if (Math.abs(turn) < 0.10) move = 0;
      drive.arcadeDrive(move, turn);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveManuallyCommand());
  }
}