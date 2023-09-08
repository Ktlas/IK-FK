#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

	// Converts degrees to radians.
	template<typename real>
	inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

	template<typename real>
	Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
	{
		Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
		Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
		Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

		switch (order)
		{
		case RotateOrder::XYZ:
			return RZ * RY * RX;
		case RotateOrder::YZX:
			return RX * RZ * RY;
		case RotateOrder::ZXY:
			return RY * RX * RZ;
		case RotateOrder::XZY:
			return RY * RZ * RX;
		case RotateOrder::YXZ:
			return RZ * RX * RY;
		case RotateOrder::ZYX:
			return RX * RY * RZ;
		}
		assert(0);
	}

	// Performs forward kinematics, using the provided "fk" class.
	// This is the function whose Jacobian matrix will be computed using adolc.
	// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
	//   IKJointIDs is an array of integers of length "numIKJoints"
	// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
	// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
	template<typename real>
	void forwardKinematicsFunction(
		int numIKJoints, const int* IKJointIDs, const FK& fk,
		const std::vector<real>& eulerAngles, std::vector<real>& handlePositions)
	{
		// Students should implement this.
		// The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
		// The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
		// Then, implement the same algorithm into this function. To do so,
		// you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
		// Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
		// It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
		// so that code is only written once. We considered this; but it is actually not easily doable.
		// If you find a good approach, feel free to document it in the README file, for extra credit.

		int jointCount = fk.getNumJoints();
		vector<Vec3<real>> localTranslations(jointCount);
		vector<Vec3<real>> globalTranslations(jointCount);
		vector<Mat3<real>> localTransforms(jointCount);
		vector<Mat3<real>> globalTransforms(jointCount);

		// local
		for (int i = 0; i < jointCount; i++)
		{
			Mat3<real> rotationMatrix;
			Mat3<real> orientRotationMatrix;

			Vec3<real> tempAngle;

			tempAngle[0] = eulerAngles[i * 3];
			tempAngle[1] = eulerAngles[i * 3 + 1];
			tempAngle[2] = eulerAngles[i * 3 + 2];
			// rotation
			rotationMatrix = Euler2Rotation(tempAngle.data(), fk.getJointRotateOrder(i));
			// jointOrientation
			tempAngle[0] = fk.getJointOrient(i).data()[0];
			tempAngle[1] = fk.getJointOrient(i).data()[1];
			tempAngle[2] = fk.getJointOrient(i).data()[2];
			orientRotationMatrix = Euler2Rotation(tempAngle.data(), getDefaultRotateOrder());
			// localTransform
			localTransforms[i] = orientRotationMatrix * rotationMatrix;
			// localTranslation
			localTranslations[i][0] = fk.getJointRestTranslation(i)[0];
			localTranslations[i][1] = fk.getJointRestTranslation(i)[1];
			localTranslations[i][2] = fk.getJointRestTranslation(i)[2];
		}

		// global
		for (int i = 0; i < jointCount; i++)
		{
			int jointIndex = fk.getJointUpdateOrder(i);
			int parentIndex = fk.getJointParent(jointIndex);

			if (parentIndex == -1)
			{
				globalTransforms[jointIndex] = localTransforms[jointIndex];
				globalTranslations[jointIndex] = localTranslations[jointIndex];
			}
			else
			{
				multiplyAffineTransform4ds(
					globalTransforms[parentIndex], 
					globalTranslations[parentIndex], 
					localTransforms[jointIndex], 
					localTranslations[jointIndex], 
					globalTransforms[jointIndex],
					globalTranslations[jointIndex]
				);
			}
		}

		// update handles
		for (int i = 0; i < numIKJoints; i++)
		{
			int jointIndex = IKJointIDs[i];
			handlePositions[3 * i] = globalTranslations[jointIndex][0];
			handlePositions[3 * i + 1] = globalTranslations[jointIndex][1];
			handlePositions[3 * i + 2] = globalTranslations[jointIndex][2];
		}
	}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int* IKJointIDs, FK* inputFK, int adolc_tagID)
{
	this->numIKJoints = numIKJoints;
	this->IKJointIDs = IKJointIDs;
	this->fk = inputFK;
	this->adolc_tagID = adolc_tagID;

	FKInputDim = fk->getNumJoints() * 3;
	FKOutputDim = numIKJoints * 3;

	train_adolc();
}

void IK::train_adolc()
{
	// Start adolc tracing
	trace_on(adolc_tagID);
	
	// Define input
	vector<adouble> x(FKInputDim);
	for (int i = 0; i < FKInputDim; i++)
	{
		x[i] <<= 0.0;
	}

	// Define output
	vector<adouble> y(FKOutputDim);

	// Define function
	forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, x, y);
	
	vector<double> output(FKOutputDim);
	for (int i = 0; i < FKOutputDim; i++)
	{
		y[i] >>= output[i];
	}

	// End adolc tracing
	trace_off();
}

void IK::doIK(const Vec3d* targetHandlePositions, Vec3d* jointEulerAngles)
{
	// You may find the following helpful:
	int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

	int isTikhonov = 1;

	// Students should implement this.
	// Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
	// Specifically, use ::function, and ::jacobian .
	// See ADOLCExample.cpp .
	//
	// Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
	// Note that at entry, "jointEulerAngles" contains the input Euler angles. 
	// Upon exit, jointEulerAngles should contain the new Euler angles.

	// First calculate jacobian
	// now, you can call ::function(adolc_tagID, ...) as many times as you like to ask ADOL-C to evaluate f for different x:
	//int lFKOutputDim = FKOutputDim;
	//int lFKInputDim = FKInputDim;
	double* handlePos = new double[FKOutputDim];
	for (int i = 0; i < FKOutputDim; i++)
	{
		handlePos[i] = 0.0;
	}
	::function(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), handlePos);

	// You can call ::jacobian(adolc_tagID, ...) as many times as you like to ask ADOL-C to evalute the jacobian matrix of f on different x:
	double* jacobianMatrix = new double[FKOutputDim * FKInputDim]; // We store the matrix in row-major order.
	double** jacobianMatrixEachRow = new double*[FKOutputDim]; // pointer array where each pointer points to one row of the jacobian matrix
	for (int i = 0; i < FKOutputDim; i++)
	{
		jacobianMatrixEachRow[i] = &jacobianMatrix[i * FKInputDim];
	}
	::jacobian(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), jacobianMatrixEachRow); // each row is the gradient of one output component of the function


	// Variables
	Eigen::VectorXd deltaTheta(FKInputDim);

	if (isTikhonov == 1) {
		// Tikhonov
		// (J.T * J + lambda * I) * deltaTheta = J.T * deltaP
		// delta P = desiredPos - currentPos
		// Create hyper parameters
		double lambda = 0.001;
		// J
		// Copy Jacobian Matrix to Eigen lib Matrix
		Eigen::MatrixXd J(FKOutputDim, FKInputDim);
		for (int i = 0; i < FKOutputDim; i++)
		{
			for (int j = 0; j < FKInputDim; j++)
			{
				J(i, j) = jacobianMatrix[i * FKInputDim + j];
			}
		}

		// Identity
		Eigen::MatrixXd I(FKInputDim, FKInputDim);
		I.setIdentity();

		// delta P
		Eigen::VectorXd deltaP(FKOutputDim);
		for (int i = 0; i < FKOutputDim; i++)
		{
			deltaP[i] = targetHandlePositions->data()[i] - handlePos[i];
		}

		// Adol-C Solver Arguments
		// A = (J.T * J + lambda * I)
		Eigen::MatrixXd A(FKInputDim, FKInputDim);
		A = J.transpose() * J + lambda * I;
		// b = J.T * deltaP
		Eigen::VectorXd b(FKOutputDim);
		b = J.transpose() * deltaP;
		// Solve
		deltaTheta = A.ldlt().solve(b);
	}
	else
	{
		// Pseudo Inverse
		// deltaTheta = J.dagger() * deltaP
		// J
		// Copy Jacobian Matrix to Eigen lib Matrix
		Eigen::MatrixXd J(FKOutputDim, FKInputDim);
		for (int i = 0; i < FKOutputDim; i++)
		{
			for (int j = 0; j < FKInputDim; j++)
			{
				J(i, j) = jacobianMatrix[i * FKInputDim + j];
			}
		}

		// delta P
		Eigen::VectorXd deltaP(FKOutputDim);
		for (int i = 0; i < FKOutputDim; i++)
		{
			deltaP[i] = targetHandlePositions->data()[i] - handlePos[i];
		}

		// J Dagger
		Eigen::MatrixXd JDagger(FKInputDim, FKOutputDim);
		JDagger = J.transpose() * (J * J.transpose()).inverse();
		
		// Solve for deltaTheta
		deltaTheta = JDagger * deltaP;
	}
	

	// Update joints
	for (int i = 0; i < numJoints; i++)
	{
		jointEulerAngles[i][0] += deltaTheta(i * 3);
		jointEulerAngles[i][1] += deltaTheta(i * 3 + 1);
		jointEulerAngles[i][2] += deltaTheta(i * 3 + 2);
	}

	delete handlePos;
	delete jacobianMatrix;
	delete jacobianMatrixEachRow;
}

