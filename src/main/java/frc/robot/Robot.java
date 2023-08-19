// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.EnumKeySerializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  TalonFX elevator = new TalonFX(1);
  XboxController controller;
  double kp = 0.000012;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    controller = new XboxController(0);

    elevator.setSelectedSensorPosition(0);
    elevator.setInverted(true);
    elevator.set(ControlMode.PercentOutput, (4500 - elevator.getSelectedSensorPosition()) * kp);
  }

  /** This function is called periodically during operator control. */
  // to go up is positive pwoer, and positive encoder
  @Override
  public void teleopPeriodic() {
    //inverted motor and power
    if(Math.abs(controller.getLeftY()) > 0.1){
      elevator.set(ControlMode.PercentOutput, controller.getLeftY()*0.5);
      SmartDashboard.putNumber("power", controller.getLeftY()*0.5);
    }
    boolean elevatorbool = false;
  
    if(controller.getAButtonPressed()){
      elevatorbool = !elevatorbool;
    }
    
    if(elevatorbool){
      elevator.set(ControlMode.PercentOutput, (138000 - elevator.getSelectedSensorPosition())* kp);
      SmartDashboard.putNumber("encoder power", (138000 - elevator.getSelectedSensorPosition()) * kp);
    } else if (!elevatorbool){
      elevator.set(ControlMode.PercentOutput, 0);

    }
    SmartDashboard.putBoolean("elevatorbool", elevatorbool);
    SmartDashboard.putNumber("encoder", elevator.getSelectedSensorPosition());
    SmartDashboard.updateValues();
    // -138000
    // -4500
    //top
    // 
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
