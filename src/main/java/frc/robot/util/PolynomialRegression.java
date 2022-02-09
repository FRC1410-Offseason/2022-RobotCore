package frc.robot.util;

import java.util.Random;

public class PolynomialRegression {
	public double[] parameters;
    private int buffersize;

	public double f(double x) {
		double ret = 0;
		for (int i = 0; i < parameters.length; i++) {
			ret += parameters[i] * Math.pow(x, i+1);
		}
		return ret;
	}

	public PolynomialRegression(int degree, int buffersize) {
		parameters = new double[degree];
		this.buffersize = buffersize;
	}

	public double error(double[] x, double[] y) {
		if (x.length != y.length) throw new IllegalArgumentException("Unequal length");
		double error = 0;
		double terror = 0;
		double ybar = 0;
		for (int i = 0; i < x.length; i++) {
			ybar += y[i];
			error += Math.pow(f(x[i] )- y[i], 2);
		}
		ybar /= y.length;
		for (int i = 0; i < x.length; i++) terror += Math.pow(y[i] - ybar, 2);
		return error / terror;
	}

	private Random random = new Random();
	private double[][] lastParameters = null;
	private double[] lastErrors = null;
	private int lastParameterIndex = 0;

	public void gradStep(double alpha, double[] x, double[] y, double stepsize, double noise) {
		if (lastParameters == null) lastParameters = new double[buffersize][parameters.length];
		if (lastErrors==null) lastErrors = new double[buffersize];
		double[] deltas = new double[parameters.length];
		double currError = error(x,y);
		for (int i=0; i < parameters.length; i++) {lastParameters[lastParameterIndex % buffersize][i] = parameters[i];}
		lastErrors[lastParameterIndex % buffersize] = currError;
		lastParameterIndex++;
		for (int i=0; i < parameters.length; i++) {
			parameters[i] += stepsize;
			double newError = error(x,y);
			deltas[i] = (newError - currError) / stepsize;
			parameters[i] -= stepsize;
		}
		for (int i=0; i < parameters.length; i++) {
			double r = random.nextDouble();
			r *= noise * alpha * Math.signum(deltas[i]);
			parameters[i] -= (alpha * deltas[i] + r);
		}
	}
    
	public void setLowestInBuffer() {
		int num = Math.min(lastParameterIndex,buffersize);
		double lowestError = -1;
		for (int i=0; i < num; i++) {
			double error = lastErrors[i];
			if (error < lowestError || lowestError < 0) {
				lowestError = error;
				for (int j = 0; j < parameters.length; j++) {parameters[j]=lastParameters[i][j];}
			}
		}
	}
}
