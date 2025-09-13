// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.geometry.struct;

import static org.psilynx.psikit.core.wpi.Struct.kSizeDouble;

import java.nio.ByteBuffer;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Quaternion;
import org.psilynx.psikit.core.wpi.Struct;

public class QuaternionStruct implements Struct<Quaternion> {
  @Override
  public Class<Quaternion> getTypeClass() {
    return Quaternion.class;
  }

  @Override
  public String getTypeName() {
    return "Quaternion";
  }

  @Override
  public int getSize() {
    return kSizeDouble * 4;
  }

  @Override
  public String getSchema() {
    return "double w;double x;double y;double z";
  }

  @Override
  public Quaternion unpack(ByteBuffer bb) {
    double w = bb.getDouble();
    double x = bb.getDouble();
    double y = bb.getDouble();
    double z = bb.getDouble();
    return new Quaternion(w, x, y, z);
  }

  @Override
  public void pack(ByteBuffer bb, Quaternion value) {
    bb.putDouble(value.getW());
    bb.putDouble(value.getX());
    bb.putDouble(value.getY());
    bb.putDouble(value.getZ());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
