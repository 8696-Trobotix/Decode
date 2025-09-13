// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.geometry.struct;

import java.nio.ByteBuffer;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.psilynx.psikit.core.wpi.Struct;

public class Translation3dStruct implements Struct<Translation3d> {
  @Override
  public Class<Translation3d> getTypeClass() {
    return Translation3d.class;
  }

  @Override
  public String getTypeName() {
    return "Translation3d";
  }

  @Override
  public int getSize() {
    return kSizeDouble * 3;
  }

  @Override
  public String getSchema() {
    return "double x;double y;double z";
  }

  @Override
  public Translation3d unpack(ByteBuffer bb) {
    double x = bb.getDouble();
    double y = bb.getDouble();
    double z = bb.getDouble();
    return new Translation3d(x, y, z);
  }

  @Override
  public void pack(ByteBuffer bb, Translation3d value) {
    bb.putDouble(value.getX());
    bb.putDouble(value.getY());
    bb.putDouble(value.getZ());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
