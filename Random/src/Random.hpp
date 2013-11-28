#ifndef LEPH_RANDOM_RANDOM_HPP
#define LEPH_RANDOM_RANDOM_HPP

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace Leph {
namespace Random {

/**
 * Random
 *
 * Utility functions for random
 * number generation
 */
class Random
{
    public:

        /**
         * Set up and initialize random seed
         */
        static inline void seed(unsigned int seed)
        {
            srand(seed);
        }

        /**
         * Return random long number uniformly distributed
         * between min and max
         */
        static inline long rangeLong(long min, long max)
        {
            return min + rand()%(max - min + 1);
        }

        /**
         * Return random double number uniformly distributed
         * between min and max
         */
        static inline double rangeDouble(double min, double max)
        {
            double r = (double)rand()/(double)RAND_MAX;
            return min + r*(max - min);
        }

        /**
         * Return a random double number from a gaussian 
         * distribution of given mean and standard deviation
         */
        static inline double gaussian(double mean, double std)
        {
            double x1, x2;
            double w;
            do {
                x1 = Random::rangeDouble(-1.0, 1.0);
                x2 = Random::rangeDouble(-1.0, 1.0);
                w = x1*x1 + x2*x2;
            } while (w >= 1.0);
            w = sqrt((-2.0*log(w))/w);
            double y = x1 * w;

            return std*y + mean;
        }
};

}
}

#endif

