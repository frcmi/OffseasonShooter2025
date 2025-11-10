// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5937.frc2025.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team5937.lib.subsystem.VirtualSubsystem;
import team5937.lib.subsystem.angular.AngularIO;
import team5937.lib.subsystem.angular.AngularSubsystem;
import team5937.lib.subsystem.linear.LinearIO;
import team5937.lib.subsystem.linear.LinearSubsystem;
import team5937.frc2025.constants.Intake.RollerConstants;
import team5937.frc2025.constants.RobotConstants;
import team5937.frc2025.constants.Intake.PivotConstants;
import lombok.Getter;

public class Intake extends VirtualSubsystem {
  private final AngularSubsystem rollers;
  private final AngularSubsystem pivot;

  @Getter private IntakeState targetState = IntakeState.kStowed;
  @Getter private IntakeState measuredState;

  /** Creates a new Intake. */
  public Intake() {
        this(
              new AngularSubsystem(
                      new AngularIO() {}, RollerConstants.kSubsystemConfigReal),
              new AngularSubsystem(
                      new AngularIO() {}, PivotConstants.kSubsystemConfigReal));
  }

  public Intake(AngularSubsystem rollers, AngularSubsystem pivot) {
    this.rollers = rollers;
    this.pivot = pivot;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> getTargetState().getPivot()));
    rollers.setDefaultCommand(rollers.openLoop(() -> getTargetState().getRollers()));

    measuredState = new IntakeState(pivot.getAngle(), targetState.getRollers());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setPivot(pivot.getAngle());
    measuredState.setRollers(targetState.getRollers());

    Logger.recordOutput("Intake/TargetState", targetState);
    Logger.recordOutput("Intake/MeasuredState", measuredState);
  }

  public Command waitUntilAtGoal() {
    return sequence(waitSeconds(RobotConstants.kDt), waitUntil(pivot.atAngle()));
  }

  public Command set(IntakeState state) {
    return set(() -> state);
}

  public Command set(Supplier<IntakeState> state) {
    return parallel(
      Commands.runOnce(() -> this.targetState = state.get()),
      pivot.angle(() -> state.get().getPivot()),
      rollers.openLoop(() -> state.get().getRollers())
    );
  }
}
