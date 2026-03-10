package com.team581.math;

import com.team581.util.AprilTags;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;
import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.RegionBSPTree2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.numbers.core.Precision;
import org.hipparchus.geometry.euclidean.twod.hull.MonotoneChain;

public class ObstructionCalculator {
  private static final Precision.DoubleEquivalence PRECISION =
      Precision.doubleEquivalenceOfEpsilon(1e-6);

  private static final MonotoneChain MONOTONE_CHAIN = new MonotoneChain();

  @SafeVarargs
  public static ObstructionCalculator fromTranslations(List<Translation2d>... obstructionCorners) {
    return new ObstructionCalculator(
        Arrays.stream(obstructionCorners)
            .map(corners -> corners.stream().map(ObstructionCalculator::vector2d).toList())
            .toList());
  }

  private static final Vector2D vector2d(Translation2d translation) {
    return Vector2D.of(translation.getX(), translation.getY());
  }

  private final RegionBSPTree2D region;

  /**
   * @param obstructionCorners Corners of polygons representing obstructions on the field.
   */
  public ObstructionCalculator(List<List<Vector2D>> obstructionCorners) {
    region = RegionBSPTree2D.empty();

    for (List<Vector2D> corners : obstructionCorners) {
      List<org.hipparchus.geometry.euclidean.twod.Vector2D> math3Points =
          corners.stream()
              .map(p -> new org.hipparchus.geometry.euclidean.twod.Vector2D(p.getX(), p.getY()))
              .toList();

      List<Vector2D> hull =
          Arrays.stream(MONOTONE_CHAIN.generate(math3Points).getVertices())
              .map(v -> Vector2D.of(v.getX(), v.getY()))
              .toList();

      region.add(ConvexArea.convexPolygonFromVertices(hull, PRECISION));
    }
  }

  public boolean contains(Translation2d point) {
    return contains(vector2d(point));
  }

  public boolean contains(Translation2d start, Translation2d end) {
    return contains(vector2d(start), vector2d(end));
  }

  public boolean contains(Vector2D point) {
    return region.contains(point);
  }

  public boolean contains(Vector2D start, Vector2D end) {
    return region.contains(start)
        || region.contains(end)
        || region.linecastFirst(Lines.segmentFromPoints(start, end, PRECISION)) != null;
  }

  public String toSvg() {
    return toSvg(AprilTags.FIELD_LAYOUT.getFieldLength(), AprilTags.FIELD_LAYOUT.getFieldWidth());
  }

  public String toSvg(double fieldLengthMeters, double fieldWidthMeters) {
    double scale = 100.0;
    double svgWidth = fieldLengthMeters * scale;
    double svgHeight = fieldWidthMeters * scale;

    StringBuilder sb = new StringBuilder();
    sb.append(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %s %s\">\n"
            .formatted((int) svgWidth, (int) svgHeight, svgWidth, svgHeight));

    sb.append(
        "<rect width=\"%s\" height=\"%s\" fill=\"#e0e0e0\"/>\n".formatted(svgWidth, svgHeight));

    for (ConvexArea convex : region.toConvex()) {
      List<Vector2D> vertices = convex.getVertices();
      sb.append("<polygon points=\"");
      for (int i = 0; i < vertices.size(); i++) {
        if (i > 0) {
          sb.append(' ');
        }
        // Flip Y so (0,0) is bottom-left
        sb.append(vertices.get(i).getX() * scale);
        sb.append(',');
        sb.append(svgHeight - vertices.get(i).getY() * scale);
      }
      sb.append("\" fill=\"red\" stroke=\"darkred\" stroke-width=\"1\"/>\n");
    }

    sb.append("</svg>\n");
    return sb.toString();
  }
}
