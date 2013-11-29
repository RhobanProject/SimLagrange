#ifndef LEPH_LWPLS_LWPLS_HPP
#define LEPH_LWPLS_LWPLS_HPP

#include <vector>
#include <stdexcept>
#include "Eigen/Dense"
#include "Eigen/StdVector"

namespace Leph {
namespace LWPLS {

/**
 * LWPLS
 *
 * Implementation of Incremental Locally
 * Weighted Partial Least Square regression
 *
 * Notations of \ref{Incremental_Online_Learning_Hight_Dimensions}
 *
 * Template parameters :
 * inputDim is the prediction (x) space dimention (K)
 * scalar is value type (float or double)
 */
template <int inputDim, class scalar = double>
class LWPLS
{
    public:

        /**
         * Typedef for predictor vector
         */
        typedef Eigen::Matrix<scalar, inputDim, 1> InputVector;

        /**
         * Typedef for custom Eigen allocator dealing with 
         * alignement issues
         */
        typedef Eigen::aligned_allocator<InputVector> InputVectorAllocator;

        /**
         * Initialization
         * The learning rate between 0 and 1 is given
         */
        LWPLS(scalar learningRate) :
            _learningRate(learningRate),
            _sumWeight(0.0),
            _inputMean(InputVector::Zero()),
            _outputMean(0.0),
            _projectionDim(0),
            _projectionPredictionError(),
            _projectionVector(),
            _outputRegression(),
            _inputRegression(),
            _latentVariance(),
            _outputCovariance(),
            _inputCovariance()
        {
            //Sanity checks
            if (_learningRate < 0.0 || _learningRate > 1.0) {
                throw std::logic_error("LWPLS invalid learning rate");
            }
            //Initialize first projection
            addProjection();
        }

        /**
         * Add a learning point to LWPLS regression and update
         * internal model
         * inputVector is the prediction (x) vector
         * outputScalar is the learned scalar (y)
         * weight is the weight coefficient of the point
         */
        inline void learning(const InputVector& inputVector, 
            scalar outputScalar, scalar weight)
        {
            scalar oldSumWeight = _sumWeight;
            //Update sum of weights and means (table 3 - 2a)
            _sumWeight = _learningRate*_sumWeight + weight;
            _inputMean = (_learningRate*oldSumWeight*_inputMean 
                + weight*inputVector)/_sumWeight;
            _outputMean = (_learningRate*oldSumWeight*_outputMean 
                + weight*outputScalar)/_sumWeight;

            //Latent coordinate of learning point for each projection (z_r)
            std::vector<scalar> latentCoord(_projectionDim);
            //Input residuals for each projection (x_{res,r})
            std::vector<InputVector, InputVectorAllocator> 
                inputRes(_projectionDim);

            //Compute prediction error and latent coordinates (table 3 - 2b)
            inputRes[0] = inputVector - _inputMean;
            scalar outputRes = _outputMean;
            for (unsigned int r=0;r<_projectionDim;r++) {
                latentCoord[r] = inputRes[r].transpose()
                    *_projectionVector[r].normalized();
                outputRes = outputRes + latentCoord[r]*_outputRegression[r];
                _projectionPredictionError[r] = 
                    _learningRate*_projectionPredictionError[r] 
                    + weight*(outputScalar-outputRes)*(outputScalar-outputRes);
                if (r != _projectionDim-1) {
                    inputRes[r+1] = inputRes[r] - latentCoord[r]*_inputRegression[r];
                }
            }

            //Update regression coefficients and 
            //variance/covariance variables (table 3 - 2c)
            outputRes = outputScalar - _outputMean;
            for (unsigned int r=0;r<_projectionDim;r++) {
                _latentVariance[r] = _learningRate*_latentVariance[r]
                    + weight*latentCoord[r]*latentCoord[r];
                _outputCovariance[r] = _learningRate*_outputCovariance[r]
                    + weight*latentCoord[r]*outputRes;
                _inputCovariance[r] = _learningRate*_inputCovariance[r] 
                    + weight*latentCoord[r]*inputRes[r];
                _outputRegression[r] = _outputCovariance[r]/_latentVariance[r];
                _inputRegression[r] = _inputCovariance[r]/_latentVariance[r];
                _projectionVector[r] = _learningRate*_projectionVector[r] 
                    + weight*outputRes*inputRes[r];
                outputRes = outputRes - latentCoord[r]*_outputRegression[r];
            }
        }

        /**
         * Use LWPLS model to predicts a new output scalar
         * value for the given input vector (x)
         */
        inline scalar prediction(const InputVector& inputVector)
        {
            InputVector inputRes = inputVector - _inputMean;
            scalar output = _outputMean;

            //(Table 3 - 3)
            for (unsigned int r=0;r<_projectionDim;r++) {
                scalar latentCoord = inputRes.transpose()
                    *_projectionVector[r].normalized();
                output = output + latentCoord*_outputRegression[r];
                inputRes = inputRes - latentCoord*_inputRegression[r];
            }

            return output;
        }

        /**
         * Increase by one the number of projections (R) and
         * initialize values
         */
        inline void addProjection()
        {
            if (_projectionDim >= inputDim) {
                return;
            }

            _projectionDim++;
            _projectionPredictionError.push_back(0.0);
            _projectionVector.push_back(InputVector::Zero());
            _outputRegression.push_back(0.0);
            _inputRegression.push_back(InputVector::Zero());
            _latentVariance.push_back(0.0);
            _outputCovariance.push_back(0.0);
            _inputCovariance.push_back(InputVector::Zero());
        }

        /**
         * Custom Eigen macro dealing with alignement issues due to
         * _inputMean and in case this object is dynamicaly allocated
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

        /**
         * Incremental leanring rate (\lambda)
         * (Between 0 and 1)
         */
        scalar _learningRate;

        /**
         * The sum of learning points weight (W)
         */
        scalar _sumWeight;

        /**
         * The weighted mean of input vector and scalar output of
         * learning points (x_0, \beta_0)
         */
        InputVector _inputMean;
        scalar _outputMean;

        /**
         * The number of latent variables or number
         * of projections (R)
         */
        unsigned int _projectionDim;

        /**
         * Prediction error 
         * for each projection (MSE_r)
         */
        std::vector<scalar> _projectionPredictionError;

        /**
         * Projection vector (latent variables) 
         * for each projection (u_r)
         */
        std::vector<InputVector, InputVectorAllocator> _projectionVector;

        /**
         * Output regression coefficient 
         * for earch projection (\beta_r)
         */
        std::vector<scalar> _outputRegression;

        /**
         * Input regression vector 
         * for each projection (p_r)
         */
        std::vector<InputVector, InputVectorAllocator> _inputRegression;

        /**
         * Latent coordinate variance 
         * for each projection (a_{zz,r})
         */
        std::vector<scalar> _latentVariance;

        /**
         * Latent coordinate and output scalar covariance 
         * for each projection (a_{zres,r})
         */
        std::vector<scalar> _outputCovariance;

        /**
         * Latent coordinate and input vector covariance 
         * for each projection (a_{xz,r})
         */
        std::vector<InputVector, InputVectorAllocator> _inputCovariance;
};

}
}

#endif

