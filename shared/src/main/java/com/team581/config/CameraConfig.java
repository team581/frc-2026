package com.team581.config;

public record CameraConfig(
    LimelightModel model,
    boolean useMt2,
    boolean useMt1RotationCloseUp,
    double forward,
    double right,
    double up,
    double pitch,
    double yaw,
    double roll) {}
