package com.team581.math;

import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;
import org.hipparchus.analysis.polynomials.PolynomialFunction;
import org.hipparchus.fitting.PolynomialCurveFitter;
import org.hipparchus.fitting.WeightedObservedPoint;

public class PolynomialRegression {
  /**
   * Perform a quadratic regression on the points in the interpolation table and returns a {@link
   * PolynomialRegression} that is fit to the points.
   */
  public static PolynomialRegression quadratic(
      String prefix, InterpolatingDoubleTreeMap interpolationTable) {
    return quadratic(prefix, TunableInterpolatingDoubleTreeMap.getEntries(interpolationTable));
  }

  /**
   * Perform a quadratic regression on the given points and returns a {@link PolynomialRegression}
   * that is fit to the points.
   */
  public static PolynomialRegression quadratic(String prefix, Map<Double, Double> points) {
    var fitter = PolynomialCurveFitter.create(2);

    var coefficients =
        fitter.fit(
            points.entrySet().stream()
                .map(entry -> new WeightedObservedPoint(1, entry.getKey(), entry.getValue()))
                .toList());

    return new PolynomialRegression(prefix, coefficients);
  }

  /**
   * Creates a {@link PolynomialRegression} for a quadratic polynomial with the given coefficients.
   *
   * @param prefix The prefix to use for logging.
   * @param a The leading coefficient.
   * @param b The slope coefficient.
   * @param c The y-intercept.
   * @return A {@link PolynomialRegression} of the form {@code ax^2 + bx + c}.
   */
  public static PolynomialRegression quadratic(String prefix, double a, double b, double c) {
    return new PolynomialRegression(prefix, new double[] {a, b, c});
  }

  // This is ripped from the original library but rounds to 2 decimal places
  /**
   * Creates a string representing a coefficient, removing ".0" endings.
   *
   * @param coeff Coefficient.
   * @return a string representation of {@code coeff}.
   */
  private static String toString(double coeff) {
    final String c = Double.toString(MathHelpers.roundTo(coeff, 2));
    if (c.endsWith(".0")) {
      return c.substring(0, c.length() - 2);
    } else {
      return c;
    }
  }

  private final PolynomialFunction function;

  private final double[] coefficients;

  public PolynomialRegression(String prefix, double[] coefficients) {
    this.function = new PolynomialFunction(coefficients);
    this.coefficients = coefficients;

    DogLog.log(prefix + "/Formula", toString());
    DogLog.log(prefix + "/Coefficients", coefficients);
  }

  public double calculate(double x) {
    return function.value(x);
  }

  // This is ripped from the original library but reverses the order of the coefficients and uses
  // our toString method
  @Override
  public String toString() {
    StringBuilder s = new StringBuilder();

    for (int i = coefficients.length - 1; i >= 1; --i) {
      if (coefficients[i] != 0) {
        if (!s.isEmpty()) {
          if (coefficients[i] < 0) {
            s.append(" - ");
          } else {
            s.append(" + ");
          }
        } else {
          if (coefficients[i] < 0) {
            s.append("-");
          }
        }

        double absAi = Math.abs(coefficients[i]);
        if ((absAi - 1) != 0) {
          s.append(toString(absAi));
          s.append(' ');
        }

        s.append("x");
        if (i > 1) {
          s.append('^');
          s.append(i);
        }
      }
    }

    if (coefficients[0] == 0.0) {
      if (s.isEmpty()) {
        return "0";
      }
    } else {
      if (!s.isEmpty()) {
        if (coefficients[0] < 0) {
          s.append(" - ");
          s.append(toString(Math.abs(coefficients[0])));
        } else {
          s.append(" + ");
          s.append(toString(coefficients[0]));
        }
      } else {
        s.append(toString(coefficients[0]));
      }
    }

    return s.toString();
  }
}
