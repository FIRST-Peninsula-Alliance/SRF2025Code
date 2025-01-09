// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MasterArmSubsystem;

public class ScoreIndexedSpeakerCmd extends Command {
  private MasterArmSubsystem m_noteConductor;
  private boolean            m_readyToScore;
  private long               m_startTime;

  /** Creates a new ScoreIndexedSpeakerCmd. */
  public ScoreIndexedSpeakerCmd(MasterArmSubsystem noteConductor) {
    m_noteConductor = noteConductor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_noteConductor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_noteConductor.prepForIndexedSpeakerScore();
    m_readyToScore = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_readyToScore) {
      // Allow an extra 300 ms to get up to speed
      // This emulates a typical reaction time for a human player to respond to
      // the teleop "ready to shoot" rumble on the gameController.
      if ((System.currentTimeMillis() - m_startTime) > 300) {
        m_noteConductor.scoreNote();
        m_readyToScore = false;
      }
    } else if (m_noteConductor.isReadyToScoreSpeaker()) {
      m_readyToScore = true;
      m_startTime = System.currentTimeMillis();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_noteConductor.isIdle();
  }
}
