// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.geometry.struct;

import java.nio.ByteBuffer;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Quaternion;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.psilynx.psikit.core.wpi.Struct;

public class Rotation3dStruct implements Struct<Rotation3d> {
  @Override
  public Class<Rotation3d> getTypeClass() {
    return Rotation3d.class;
  }

  @Override
  public String getTypeName() {
    return "Rotation3d";
  }

  @Override
  public int getSize() {
    return Quaternion.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Quaternion q";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Quaternion.struct};
  }

  @Override
  public Rotation3d unpack(ByteBuffer bb) {
    Quaternion q = Quaternion.struct.unpack(bb);
    return new Rotation3d(q);
  }

  @Override
  public void pack(ByteBuffer bb, Rotation3d value) {
    Quaternion.struct.pack(bb, value.getQuaternion());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
