#ifndef LEPH_LWPR_LWPR_HPP
#define LEPH_LWPR_LWPR_HPP

#include <vector>
#include "Eigen/StdVector"
#include "LWPR/src/ReceptiveField.hpp"

namespace Leph {
namespace LWPR {

/**
 * LWPR
 *
 * Implementation of Locally Weighted
 * Projection Regression
 *
 * Notations of \ref{Incremental_Online_Learning_Hight_Dimensions}
 *
 * Template parameters :
 * inputDim is the prediction (x) space dimention (K)
 * scalar is value type (float or double)
 */
template <unsigned int inputDim, class scalar = double>
class LWPR
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
         * Typedef for custom Eigen allocator dealing with 
         * alignement issues
         */
        typedef Eigen::aligned_allocator<ReceptiveField<inputDim, scalar> > 
            InputVectorAllocator;

        /**
         * Typedef for internal Receptive Field
         */
        typedef ReceptiveField<inputDim, scalar> RF;

        /**
         * Initialization
         * The forgetting rate between 0 and 1
         * The threshold of activation weigth receptive field creation
         * The penalty coefficient for shape size update
         * The speed of gradient descent for shape update
         * The initial value of ReceptiveField shape matrix
         */
        LWPR(scalar forgettingRate, scalar weigthGen, 
            scalar penaltyShape, scalar gradientLearningRate,
            const InputMatrix& shapeInit) :
            _forgettingRate(forgettingRate),
            _shapeInit(shapeInit),
            _weightGen(weigthGen),
            _penaltyShape(penaltyShape),
            _gradientLearningRate(gradientLearningRate),
            _receptiveFields()
        {
        }

        /**
         * Return the prediction scalar (y) for the given
         * input vector point
         */
        inline scalar prediction(const InputVector& input)
        {
            //Compute a weighted sum of all Receptive Fields
            //single prediction
            scalar sumWeight = 0.0;
            scalar output = 0.0;
            for (size_t i=0;i<_receptiveFields.size();i++) {
                scalar weight = _receptiveFields[i].activationWeight(input);
                sumWeight += weight;
                output += weight*_receptiveFields[i].prediction(input);
            }

            return output/sumWeight;
        }

        /**
         * Learning of all internal receptive field with given
         * predictor input vector and scalar output result
         */
        inline void learning(const InputVector& input, 
            scalar output)
        {
            std::cout << "========== LEARN " << input.transpose() << " " << output << std::endl; //TODO
            //Forward learnign to all receptive fields
            scalar maxWeight = 0.0;
            for (size_t i=0;i<_receptiveFields.size();i++) {
                scalar weight = _receptiveFields[i].activationWeight(input);
                std::cout << "=== Learn " << i << " with " << weight << std::endl; //TODO
                _receptiveFields[i].learning(input, output, weight);
                if (weight > maxWeight) {
                    maxWeight = weight;
                }
            }
            //Create new receptive field if no one have been 
            //activated more than a threshold
            if (maxWeight < _weightGen) {
                _receptiveFields.push_back(ReceptiveField<inputDim, scalar>(
                    _forgettingRate, _penaltyShape, 
                    _gradientLearningRate, input, _shapeInit));
            }
        }

        /**
         * Custom Eigen macro dealing with alignement issues due to
         * attribute Matrix and in case this object is dynamicaly allocated
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Return the number of internal Receptive Fields
         */
        inline size_t getReceptiveFieldCount() const
        {
            return _receptiveFields.size();
        }

        /**
         * Access to internal Receptive Field
         */
        inline const RF& getReceptiveField(size_t n) const
        {
            return _receptiveFields.at(n);
        }

    private:

        /**
         * The forgetting factor (\lambda) controling PLS 
         * regression speed of convergence
         */
        const scalar _forgettingRate;
        
        /**
         * Initial value for shape Receptive Field matrix 
         * on creation (D_def)
         */
        const InputMatrix _shapeInit;

        /**
         * Activation weight threshold (w_gen) for
         * creation of new receptive field
         */
        const scalar _weightGen;

        /**
         * The penalty parameter (\gamma) preventing the
         * Receptive Field shape to shrink to much
         */
        const scalar _penaltyShape;

        /**
         * Gradient descent learning rate (\alpha).
         * Control the speed of convergence of shape
         * update
         */
        const scalar _gradientLearningRate;

        /**
         * Receptive Fields container
         */
        std::vector<RF, InputVectorAllocator> 
            _receptiveFields;
};

}
}

#endif

