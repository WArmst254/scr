// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024;



import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


import com.ctre.phoenix6.signals.NeutralModeValue;

import com.team4522.lib.util.RunOnce;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc2024.Constants.RobotMode;

import frc2024.controlboard.Controlboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Timer timeSinceDisabled = new Timer();
  private Timer autoTimer = new Timer();
  private RunOnce autoConfigurator = new RunOnce();

  public Robot() {}

  @Override
  public void robotInit() {

    switch (Constants.MODE) {
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        Logger.start();
        break;
      case SIM, DEV:
        break;
      default:
        break;
    } 

    robotContainer = new RobotContainer();
    timeSinceDisabled.reset();
    timeSinceDisabled.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    autoConfigurator.runOnceWhen(
      () -> {
        RobotContainer.getSwerve().configureAutoBuilder();
        RobotContainer.configAuto();
        System.out.println("Ready To Enable");
      },
      DriverStation.getAlliance().isPresent());
  }

  @Override
  public void disabledInit() {
    timeSinceDisabled.reset();
    timeSinceDisabled.start();
    RobotContainer.stopAll();
    Controlboard.driverController_Command.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void disabledPeriodic() {
    if(((int) timeSinceDisabled.get()) == 5 && Constants.MODE == RobotMode.DEV){
      RobotContainer.getSwerve().setNeutralModes(NeutralModeValue.Coast, NeutralModeValue.Coast);
    }
  }

  @Override
  public void disabledExit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.stopAll();
    timeSinceDisabled.stop();
    
    if(Constants.MODE == RobotMode.DEV){
      RobotContainer.getSwerve().setNeutralModes(NeutralModeValue.Brake, NeutralModeValue.Brake);
    }
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand().andThen(() -> RobotContainer.stopAll());

    if (autonomousCommand != null) {
      autoTimer.reset();
      autoTimer.start();
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    if(autonomousCommand.isFinished()){
      autoTimer.stop();
      System.out.println("[Auto] Time taken: " + autoTimer.get());
    }
  }

  @Override
  public void autonomousExit() {
    RobotContainer.stopAll();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    RobotContainer.getSwerve().stopOdometryThread();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
