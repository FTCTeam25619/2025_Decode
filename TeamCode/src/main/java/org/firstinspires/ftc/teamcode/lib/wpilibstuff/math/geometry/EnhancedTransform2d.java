// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.geometry;

//import static edu.wpi.first.units.Units.Meters;

// import edu.wpi.first.math.geometry.proto.Transform2dProto;
// import edu.wpi.first.math.geometry.struct.Transform2dStruct;
//import edu.wpi.first.units.Distance;
//import edu.wpi.first.units.Measure;
// import edu.wpi.first.util.protobuf.ProtobufSerializable;
// import edu.wpi.first.util.struct.StructSerializable;
import com.seattlesolvers.solverslib.geometry.Transform2d;

/** Represents a transformation for a Pose2d in the pose's frame. */
public class EnhancedTransform2d extends Transform2d
// implements ProtobufSerializable, StructSerializable
{
  /**
   * A preallocated Transform2d representing no transformation.
   *
   * <p>This exists to avoid allocations for common transformations.
   */
  public static final EnhancedTransform2d kZero = new EnhancedTransform2d();

}
