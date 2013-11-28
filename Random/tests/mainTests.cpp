#include <iostream>
#include <cassert>
#include "Random/src/Random.hpp"

using namespace std;
using namespace Leph::Random;

int main()
{
    for (long i=0;i<10000;i++) {
        long r = Random::rangeLong(10, 20);
        assert(r >= 10 && r <= 20);
    }
    for (long i=0;i<10000;i++) {
        double r = Random::rangeDouble(10.0, 20.0);
        assert(r >= 10.0 && r <= 20.0);
    }
    
    double mean = 0;
    double squaredMean = 0;
    for (long i=0;i<10000;i++) {
        double r = Random::gaussian(10.0, 20.0);
        mean += r;
        squaredMean += r*r;
    }
    mean /= (double)10000;
    squaredMean /= (double)10000;
    double std = sqrt(squaredMean - mean*mean);
    assert(mean > 9.5 && mean < 10.5);
    assert(std > 19.5 && std < 20.5);

    return 0;
}

