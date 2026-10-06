// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;


public class MAXSwerveModule {

    // Drive Motors - NEO
    private final SparkMax m_drivingSpark;

    // Turn Motors - NEO
    private final SparkMax m_turningSpark;

    // Note: Since both the driving and turning motors are Spark Maxes, they need encoders.
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEO turn motors with SPARK MAXes, NEO drive motors w/ Spark Maxes, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {

        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
        m_drivingEncoder = m_drivingSpark.getEncoder();

        // Configuring the NEO turn motors.
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle. This is applied only to the turning motor.
        m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double motorRps = m_drivingEncoder.getVelocity();
        double wheelRps = motorRps / ModuleConstants.kKrakenDriveGearRatio;
        double mps = wheelRps * ModuleConstants.kWheelCircumferenceMeters;

        return new SwerveModuleState(mps, getTurningAngle());
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double motorRot = m_drivingEncoder.getPosition();
        return new SwerveModulePosition(motorRotToMeters(motorRot), getTurningAngle());
    }

    public double getNeoVelocity() {
        return m_drivingEncoder.getVelocity() * (ModuleConstants.kWheelDiameterMeters / 2);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState corrected = new SwerveModuleState(desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        corrected.optimize(Rotation2d.fromRadians(m_turningEncoder.getPosition()));

        // Set the motor's internal controller to drive at the goal velocity speed.
        m_drivingSpark.set(corrected.speedMetersPerSecond);

        // Use position control to command the turning motor to go to the desired angle.
        m_turningClosedLoopController.setSetpoint(corrected.angle.getRadians(), ControlType.kPosition);

        // Store the desired state for debugging purposes.
        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    /**  */
    private double motorRotToMeters(double motorRot) {
        double wheelRot = motorRot / ModuleConstants.kKrakenDriveGearRatio;
        return wheelRot * ModuleConstants.kWheelCircumferenceMeters;
    }

    private Rotation2d getTurningAngle() {
        return Rotation2d.fromRadians(m_turningEncoder.getPosition())
                .minus(Rotation2d.fromRadians(m_chassisAngularOffset));
    }
}
