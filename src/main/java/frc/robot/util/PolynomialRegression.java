package frc.robot.util;

import java.util.List;

/**
 * Class for managing polynomial regressions. Each instance has its own parameters and coefficients.
 */
public class PolynomialRegression extends GradientDescentOptimized {
	/**
	 * Compute f(x) from the coefficients and parameters. n degree polynomial, but without any a_0 coefficient due to the fact that we want f(0)=0.
	 * @param x
	 * @return f(x)
	 */
	public double function(double x) {
		double ret = 0;
		for (int i = 0; i < getParameters().length; i++) {
			ret += getParameters()[i] * Math.pow(x, i+1);
		}
		return ret;
	}
	/**
	 * degree - Largest power in the polynomial.
	 * bufferSize - Number of parameters in the path to store to combat randomness errors (20 is about good).
	 * @param degree
	 * @param bufferSize
	 */
	public PolynomialRegression(int degree, int bufferSize) {
		super(degree , bufferSize);
	}
	/**
	 * Finds 1-r^2 for the polynomial regression
	 * Errorparams is a list of parameters for the error function.
	 * @param errorparams
	 * @return error
	 */
	@SuppressWarnings("unchecked")
	@Override public double error(Object... errorparams) {
		List<Double> x = (List<Double>) errorparams[0];
		List<Double> y = (List<Double>) errorparams[1];
		if (x.size() != y.size()) throw new IllegalArgumentException("Unequal length");

		double error = 0;
		double tError = 0;
		double yBar = 0;

		for (int i = 0; i < x.size(); i++) {
			yBar += y.get(i);
			error += Math.pow(function(x.get(i)) - y.get(i), 2);
		}

		yBar /= y.size();

		for (int i = 0; i < x.size(); i++) tError += Math.pow(y.get(i) - yBar, 2);

		return error / tError;
	}
}
