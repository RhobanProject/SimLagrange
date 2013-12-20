#ifndef LEPH_LWPR_RECEPTIVEFIELD_HPP
#define LEPH_LWPR_RECEPTIVEFIELD_HPP

#include <cmath>
#include <stdexcept>
#include <vector>
#include "Eigen/Dense"
#include "LWPLS/src/LWPLS.hpp"

namespace Leph {
namespace LWPR {

/**
 * ReceptiveField
 *
 * Implementation of Receptive Field for
 * LWPR algorithm
 * \ref{Incremental_Online_Learning_Hight_Dimensions}
 */
template <unsigned int inputDim, class scalar = double>
class ReceptiveField
{
    public:
        
        /**
         * Typedef for predictor vector
         */
        typedef Eigen::Matrix<scalar, inputDim, 1> InputVector;

        /**
         * Typedef for predictor distance matrix
         */
        typedef Eigen::Matrix<scalar, inputDim, inputDim> InputMatrix;

        /**
         * Initialization
         * The forgetting rate between 0 and 1
         * The penalty coefficient for shape size update
         * The speed of gradient descent for shape update
         * The center of ReceptiveField in input space
         * The initial value of ReceptiveField shape matrix
         */
        ReceptiveField(scalar forgettingRate, scalar penaltyShape, 
            scalar gradientLearningRate,
            const InputVector& center, 
            const InputMatrix& initShape) :
            _lwpls(forgettingRate),
            _center(center),
            _shape(initShape),
            _shapeDecomposition(InputMatrix::Zero()),
            _forgettingRate(forgettingRate),
            _penaltyShape(penaltyShape),
            _gradientLearningRate(gradientLearningRate),
            _projectionH(),
            _projectionG(),
            _projectionE()
        {
            //Cholesky decomposition of initial shape matrix
            _shapeDecomposition = _shape.llt().matrixU();
            //Initialization of statistic traces 
            //(same length as number of PLS projection)
            _projectionH.push_back(0.0);
            _projectionG.push_back(0.0);
            _projectionE = 0.0;
        }

        /**
         * Return the activation weight (w) of the given point
         */
        inline scalar activationWeight(const InputVector& input) const
        {
            InputVector pt = input - _center;
            return exp(-0.5*(scalar)(pt.transpose()*_shape*pt));
        }

        /**
         * Call LWPLS regression to evaluate an output scalar 
         * prediction (y) for the given input vector (x)
         */
        inline scalar prediction(const InputVector& input) const
        {
            return _lwpls.prediction(input);
        }

        /**
         * Update PLS regression. Gradient descent for shape update and
         * check for adding new PLS projection
         */
        inline void learning(const InputVector& inputVector, 
            scalar outputScalar, scalar weight)
        {
            //No learning in case of null weight
            if (weight < 1e-9f) {
                return;
            }

            //Update PLS regression
            _lwpls.learning(inputVector, outputScalar, weight);

            //Update receptive field shape (Table 4)
            scalar sumWeight = _lwpls.getSumWeight();
            scalar dJ1 = computeDJ1(weight);
            InputMatrix dShape = InputMatrix::Zero();
            //Iteration through upper triangular shape decomposition
            for (int j=0;j<dShape.cols();j++) {
                for (int i=0;i<=j;i++) {
                    dShape(i, j) = 
                        dJ1
                        *computeDWeight(inputVector, weight, i, j)
                        + weight/sumWeight
                        *computeDJ2(i, j);
                }
            }
            std::cout << "weight: " << weight << " Shape: " << _shape << " decomposition: " << _shapeDecomposition << " dShape: " << dShape << std::endl;//TODO

            //Update shape decomposition
            //Eval due to Eigen internal optimization
            //(see aliasing)
            _shapeDecomposition = (_shapeDecomposition 
                - _gradientLearningRate*dShape).eval();
            //Update shape matrix
            _shape = _shapeDecomposition.transpose()*_shapeDecomposition;
            
            std::cout << "=> Shape: " << _shape << " decomposition: " << _shapeDecomposition << std::endl;//TODO

            //Check for new projection
            //TODO
        }

        /**
         * Access to internal PLS regression for information getters
         */
        inline const LWPLS::LWPLS<inputDim, scalar>& getLWPLS() const
        {
            return _lwpls;
        }

        /**
         * Custom Eigen macro dealing with alignement issues due to
         * attribute Matrix and in case this object is dynamicaly allocated
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

        /**
         * The Locally Partial Least Square regresion
         * container
         */
        LWPLS::LWPLS<inputDim, scalar> _lwpls;

        /**
         * The center of the ReceptiveField (c)
         * in input space
         */
        InputVector _center;

        /**
         * The definite positive shape and size matrix
         * (D) in input space
         */
        InputMatrix _shape;

        /**
         * The Cholesky decomposition (M) of shape matrix (D)
         * The matrix is upper triangular
         */
        InputMatrix _shapeDecomposition;

        /**
         * The forgetting factor (\lambda) controling PLS 
         * regression speed of convergence
         */
        scalar _forgettingRate;
        
        /**
         * The penalty parameter (\gamma) preventing the
         * Receptive Field shape to shrink to much
         */
        scalar _penaltyShape;

        /**
         * Gradient descent learning rate (\alpha).
         * Control the speed of convergence of shape
         * update
         */
        scalar _gradientLearningRate;

        /**
         * Statistic incremental values used to compute cost
         * function derivative (Table 4)
         * The variables (a_H), (a_G) and (a_E) are vector
         * of same length as the number of PLS projections.
         * (Their have to be update as projection number increase)
         */
        std::vector<scalar> _projectionH;
        std::vector<scalar> _projectionG;
        scalar _projectionE;

        /**
         * Return the coefficient (i,j) of the derivative 
         * of (D) with respect to the coefficient (k,l) of (M)
         */
        inline scalar computeDShapeCoef(int i, int j, 
            int k, int l) const
        {
            scalar value = 0.0;
            if (i == l) {
                value += _shapeDecomposition(k, j);
            }
            if (j == l) {
                value += _shapeDecomposition(k, i);
            }

            return value;
        }

        /**
         * Construct and return the derivative of (D) with respect
         * the coefficient (k,l) of (M)
         */
        inline InputMatrix computeDShape(int k, int l) const
        {
            InputMatrix dShape;
            for (int j=0;j<dShape.cols();j++) {
                for (int i=0;i<dShape.rows();i++) {
                    dShape(i, j) = computeDShapeCoef(i, j, k, l);
                }
            }

            return dShape;
        }

        /**
         * Compute the derivative of given weight with respect to
         * the coefficient (k,l) of (M)
         */
        inline scalar computeDWeight(const InputVector& input, 
            scalar weight, int k, int l) const
        {
            InputVector pt = input - _center;
            return -0.5*weight
                *(scalar)(pt.transpose()*computeDShape(k, l)*pt);
        }

        /**
         * Compute the derivative of (J_2) with respect to
         * the coefficient (k,l) of (M)
         */
        inline scalar computeDJ2(int k, int l) const
        {
            scalar sum = 0.0;
            for (int j=0;j<_shape.cols();j++) {
                for (int i=0;i<_shape.rows();i++) {
                    sum += _shape(i, j) * computeDShapeCoef(i, j, k, l);
                }
            }

            return 2.0*_penaltyShape*sum/inputDim;
        }

        /**
         * Compute the derivative of (J_1) with respect to
         * given weight accordingly with the reference article (Table 4)
         */
        inline scalar computeDJ1(scalar weight)
        {
            //Create useful variables
            unsigned int projDim = _lwpls.getProjectionDim();
            //(z)
            std::vector<scalar> z(projDim);
            //(z^2)
            std::vector<scalar> zSquared(projDim);
            //(q)
            std::vector<scalar> q(projDim);
            //(q^2)
            std::vector<scalar> qSquared(projDim);
            //(h)
            scalar h = 0.0;
            for (size_t r=0;r<projDim;r++) {
                z[r] = _lwpls.getLastLatentCoord(r);
                zSquared[r] = z[r]*z[r];
                if (_lwpls.getLatentVariance(r) < 1e-9f) {
                    q[r] = 0.0;
                } else {
                    q[r] = z[r]/_lwpls.getLatentVariance(r);
                }
                qSquared[r] = q[r]*q[r];
                h += z[r]*q[r];
            }
            h *= weight;
            //When h is near 1.0, its mean that the
            //matrix inversion process by Sherman–Morrison 
            //formula can not be done. Then, h is set to zero 
            //so the cross validation error take into account
            //the current point. Seems not to happen offen.
            if (fabs(1.0-h) < 1e-9f) {
                h = 0.0;
            }
            //(e_cv)
            scalar e_cv = _lwpls.getLastPredictionError();
            //(e)
            scalar e = _lwpls.getLastFittingError();
            //(W)
            scalar W = _lwpls.getSumWeight();

            //Update of statictic traces
            for (size_t r=0;r<projDim;r++) {
                _projectionH[r] = _forgettingRate*_projectionH[r]
                    + weight*e_cv*z[r]/(1.0-h);
                _projectionG[r] = _forgettingRate*_projectionG[r]
                    + weight*weight*e_cv*e_cv*zSquared[r]/(1.0-h);
            }
            _projectionE = _forgettingRate*_projectionE
                + weight*e_cv*e_cv;

            //Compute scalar product
            //(q'.a_H)
            scalar prodQH = 0.0;
            //(q^2'.a_G)
            scalar prodQSquaredG = 0.0;
            for (size_t r=0;r<projDim;r++) 
            {
                prodQH += q[r]*_projectionH[r];
                prodQSquaredG += qSquared[r]*_projectionG[r];
            }

            /* //TODO
            for (size_t r=0;r<projDim;r++) {
                std::cout << "??? " << weight << " " << e_cv << " " << z[r] << " " << h << " --- " << (1.0-h) << " " << e_cv*z[r]/(1.0-h) << std::endl;
                std::cout << "!!! " << weight << " " << z[r] << " " << q[r] << "   " << _lwpls.getLatentVariance(r) << std::endl;
                std::cout << r << " >>> " << z[r] << " " << q[r] << " " << _projectionH[r] << " " << _projectionG[r] << " " << _projectionE << std::endl;
            }
            std::cout <<"##>"<<projDim<<" "<<e_cv<<" "<<e<<" "<<W<<" "<<prodQH<<" "<<prodQSquaredG<<" "<<h<< std::endl;
            */
            //Return the J1 derivative
            scalar dJ1 = 
                e_cv*e_cv/W 
                - 2.0*e*prodQH/W 
                - 2.0*prodQSquaredG/W 
                - _projectionE/(W*W);
            //std::cout << "||>" << dJ1 << std::endl; //TODO
            return dJ1;
        }
};

}
}

#endif

