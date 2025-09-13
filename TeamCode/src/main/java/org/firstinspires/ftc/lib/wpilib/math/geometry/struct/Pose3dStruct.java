// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.geometry.struct;

import java.nio.ByteBuffer;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.psilynx.psikit.core.wpi.Struct;

public class Pose3dStruct implements Struct<Pose3d> {
  @Override
  public Class<Pose3d> getTypeClass() {
    return Pose3d.class;
  }

  @Override
  public String getTypeName() {
    return "Pose3d";
  }

  @Override
  public int getSize() {
    return Translation3d.struct.getSize() + Rotation3d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Translation3d translation;Rotation3d rotation";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Translation3d.struct, Rotation3d.struct};
  }

  @Override
  public Pose3d unpack(ByteBuffer bb) {
    Translation3d translation = Translation3d.struct.unpack(bb);
    Rotation3d rotation = Rotation3d.struct.unpack(bb);
    return new Pose3d(translation, rotation);
  }

  @Override
  public void pack(ByteBuffer bb, Pose3d value) {
    Translation3d.struct.pack(bb, value.getTranslation());
    Rotation3d.struct.pack(bb, value.getRotation());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
