// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.vision2;

// import java.util.Arrays;
// import java.util.List;
// import java.util.Optional;
// import java.util.function.BooleanSupplier;
// import java.util.stream.Collectors;
// import java.util.stream.Stream;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.vision.VisionIOLimelightBase;

// public class MultiBaseLimelight extends SubsystemBase {

//   private Alliance m_alliance;
//   private final BooleanSupplier m_isPoseUpdateAllowed;
//   private final List<LimelightBase> m_limelights;

//   /** Creates a new MultiBaseLimelight. */
//   public MultiBaseLimelight(BooleanSupplier isPoseUpdateAllowed, String... limelights) {
//     m_alliance = Alliance.Invalid;
//     m_isPoseUpdateAllowed = isPoseUpdateAllowed;
//     m_limelights = Stream.of(limelights)
//       .map(limelight -> new LimelightBase(limelight))
//       .collect(Collectors.toUnmodifiableList());
//   }

//   @Override
//   public void periodic() {
//     if (m_alliance == Alliance.Invalid) {
//       m_alliance = DriverStation.getAlliance();
//       return;
//     }
//     if (!m_isPoseUpdateAllowed.getAsBoolean()) {
//       return;
//     }
//     final Optional<LimelightBase> llsWithMultipleTags = m_limelights.stream()
//         .filter(limelight -> limelight.getVisibleTagCount() > 1)
//         .findFirst();
//     if (llsWithMultipleTags.isEmpty()) {
//       return;
//     }
//     // llsWithMultipleTags.get().
//   }
// }
