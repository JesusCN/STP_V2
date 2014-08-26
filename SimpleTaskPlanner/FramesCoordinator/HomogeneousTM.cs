using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.Mathematics;

namespace SimpleTaskPlanner
{
	abstract class HomogeneousTM
	{
		Matrix hMat;

		/// <summary>
		/// Create Homogeneous Transformation Matrix based on Denavit-Hartenberg parameters.  z0 is axis of actuation of joint 1. x1 is perpendicular to z0. x1 intersects z0.
		/// </summary>
		/// <param name="a">Distance between axes z0 and z1, along the x1 axis.</param>
		/// <param name="alpha">Angle between axes z0 and z1, measured in a plane normal to x1. Positive from z0 to z1.</param>
		/// <param name="d">Perpendicular distance fromthe origin o0 to the intersection of x1 and z0, measured along z0.</param>
		/// <param name="theta">Angle between x0 and x1, measured in a plane normal to z0. Positive from x0 to x1.</param>
		public HomogeneousTM(double a, double alpha, double d, double theta)
		{
			hMat = Matrix4.Zero;

			this.hMat[0, 0] = MathUtil.Cos(theta);
			this.hMat[1, 0] = MathUtil.Sin(theta);
			this.hMat[2, 0] = 0;
			this.hMat[3, 0] = 0;

			this.hMat[0, 1] = -(1) * MathUtil.Sin(theta) * MathUtil.Cos(alpha);
			this.hMat[1, 1] = MathUtil.Cos(theta) * MathUtil.Cos(alpha);
			this.hMat[2, 1] = MathUtil.Sin(alpha);
			this.hMat[3, 1] = 0;

			this.hMat[0, 2] = MathUtil.Sin(theta) * MathUtil.Sin(alpha);
			this.hMat[1, 2] = -(1) * MathUtil.Cos(theta) * MathUtil.Sin(alpha);
			this.hMat[2, 2] = MathUtil.Cos(alpha); ;
			this.hMat[3, 2] = 0;

			this.hMat[0, 3] = a * MathUtil.Cos(theta);
			this.hMat[1, 3] = a * MathUtil.Sin(theta);
			this.hMat[2, 3] = d;
			this.hMat[3, 3] = 1;
		}

		/// <summary>
		/// Create Homogeneous Transformation Matrix with a rotation (roll,pitch,yaw) followed by a translation.
		/// </summary>
		/// <param name="rotationMatrix"></param>
		/// <param name="translationMatrix"></param>
		public HomogeneousTM(Matrix rotationMatrix, Matrix translationMatrix)
		{
			if ((rotationMatrix.Rows != 3) || (rotationMatrix.Columns != 3))
				throw new ArgumentException("rotationMatrix must be a 3x3 matrix");

			if ((translationMatrix.Rows != 3) || (translationMatrix.Columns != 1))
				throw new ArgumentException("translationMatrix must be a 3x1 matrix");

			hMat = Matrix4.Zero;

			// Filling Rotation Matrix
			for (int row = 0; row < rotationMatrix.Rows; row++)
				for (int col = 0; col < rotationMatrix.Columns; col++)
					hMat[row, col] = rotationMatrix[row, col];

			// Filling Translation Matrix
			for (int row = 0; row < translationMatrix.Rows; row++)
				hMat[row, 3] = translationMatrix[row, 0];

			// Filling Homogenous Row
			hMat[3, 0] = 0;
			hMat[3, 1] = 0;
			hMat[3, 2] = 0;
			hMat[3, 3] = 1;
		}

		/// <summary>
		/// Clone an Homogeneous Transformation Matrix.
		/// </summary>
		/// <param name="hm"></param>
		public HomogeneousTM(HomogeneousTM hm)
		{
			hMat = new Matrix(hm.hMat);
		}

		public HomogeneousTM Inverse
		{
			get
			{
				return InvertHomogenousMatrix(this);
			}
		}

		public Vector3 Transform(Vector3 vec)
		{
			if (vec == null)
				throw new ArgumentNullException();

			Matrix hVec = new Matrix(4, 1);
			hVec[0, 0] = vec.X;
			hVec[1, 0] = vec.Y;
			hVec[2, 0] = vec.Z;
			hVec[3, 0] = 1;

			Matrix resMat = hMat * hVec;
			Vector3 resVec = new Vector3();
			resVec.X = resMat[0, 0];
			resVec.Y = resMat[1, 0];
			resVec.Z = resMat[2, 0];

			return resVec;
		}

		private HomogeneousTM InvertHomogenousMatrix(HomogeneousTM hm)
		{
			Matrix rotMatrix = new Matrix(3, 3);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					rotMatrix[i, j] = hm.hMat[i, j];
				}
			}

			Matrix traMatrix = new Matrix(3, 1);
			for (int i = 0; i < 3; i++)
			{
				traMatrix[i, 0] = hm.hMat[i, 3];
			}

			Matrix newRotMat = rotMatrix.Transpose;
			Matrix newTraMat = (-1)*rotMatrix.Transpose*traMatrix;

			return new HomogeneousTM(newRotMat, newTraMat );
		}

		#region Static Methods

		public static Matrix TranslationMatrix(double xTranslation, double yTranslation, double zTranslation)
		{
			Matrix transMatrix = new Matrix(3, 1);

			transMatrix[0, 0] = xTranslation;
			transMatrix[1, 0] = yTranslation;
			transMatrix[2, 0] = zTranslation;

			return transMatrix;
		}

		public static Matrix RollZMatrix(double rollAngle)
		{
			Matrix rollM = new Matrix(3, 3);

			rollM[0, 0] = MathUtil.Cos(rollAngle);
			rollM[0, 1] = -MathUtil.Sin(rollAngle);
			rollM[0, 2] = 0;

			rollM[1, 0] = MathUtil.Sin(rollAngle);
			rollM[1, 1] = MathUtil.Cos(rollAngle);
			rollM[1, 2] = 0;

			rollM[2, 0] = 0;
			rollM[2, 1] = 0;
			rollM[2, 2] = 1;

			return rollM;
		}

		public static Matrix PitchYMatrix(double pitchAngle)
		{
			Matrix pitchM = new Matrix(3, 3);

			pitchM[0, 0] = MathUtil.Cos(pitchAngle);
			pitchM[0, 1] = 0;
			pitchM[0, 2] = MathUtil.Sin(pitchAngle);

			pitchM[1, 0] = 0;
			pitchM[1, 1] = 1;
			pitchM[1, 2] = 0;

			pitchM[2, 0] = -MathUtil.Sin(pitchAngle);
			pitchM[2, 1] = 0;
			pitchM[2, 2] = MathUtil.Cos(pitchAngle);

			return pitchM;
		}

		public static Matrix YawXMatrix(double yawAngle)
		{
			Matrix yawM = new Matrix(3, 3);

			yawM[0, 0] = 1;
			yawM[0, 1] = 0;
			yawM[0, 2] = 0;

			yawM[1, 0] = 0;
			yawM[1, 1] = MathUtil.Cos(yawAngle);
			yawM[1, 2] = -MathUtil.Sin(yawAngle);

			yawM[2, 0] = 0;
			yawM[2, 1] = MathUtil.Sin(yawAngle);
			yawM[2, 2] = MathUtil.Cos(yawAngle);

			return yawM;
		}

		public static Matrix RotationMatrix(double rollAngle, double pitchAngle, double yawAngle)
		{
			Matrix rotMatrix = new Matrix(3, 3);
			rotMatrix = RollZMatrix(rollAngle) * PitchYMatrix(pitchAngle) * YawXMatrix(yawAngle);

			return rotMatrix;
		}

		public static HomogeneousTM operator *(HomogeneousTM hm1, HomogeneousTM hm2)
		{
			HomogeneousTM newHM = new HomogeneousTM(0, 0, 0, 0);
			newHM.hMat = hm1.hMat * hm1.hMat;
			
			return newHM;
		}

		#endregion
	}
}