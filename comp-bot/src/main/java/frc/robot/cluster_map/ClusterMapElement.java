package frc.robot.cluster_map;

import edu.wpi.first.math.geometry.Translation2d;

public record ClusterMapElement(
    double expiresAt, Translation2d clusterTranslation, double health) {}
