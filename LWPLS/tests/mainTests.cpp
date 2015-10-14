#include <iostream>
#include <cassert>
#include <cmath>
#include "LWPLS/src/LWPLS.hpp"
#include "Random/src/Random.hpp"
#include "Plot/src/Plot.hpp"

using namespace std;
using namespace Leph::LWPLS;

int main()
{
    double lambda = 0.95;
    LWPLS<2> lwpls(lambda);
    lwpls.addProjection();

    double a = 1.5;
    double b = -0.75;
    double c = 0.5;
    double sigma = 0.4;

    for (int k=0;k<500;k++) {
        double x = Leph::Random::Random::rangeDouble(-3.0, 3.0);
        double y = Leph::Random::Random::rangeDouble(-3.0, 3.0);
        double z = a*x + b*y + c;
        double zz = z + Leph::Random::Random::gaussian(0.0, sigma);
        LWPLS<2>::InputVector m(x, y);
        lwpls.learning(m, zz, 1.0);
        Leph::Plot::Plot::add("Target", x, y, z);
        Leph::Plot::Plot::add("Noise", x, y, zz);
    }

    double error = sqrt(lwpls.getProjectionPredictionError(1)*(1.0-lambda));
    assert(error < 1.5*sigma && error > 0.5*sigma);

    double errorMean = 0;
    double errorStd = 0;
    for (int k=0;k<500;k++) {
        double x = Leph::Random::Random::rangeDouble(-5.0, 5.0);
        double y = Leph::Random::Random::rangeDouble(-5.0, 5.0);
        LWPLS<2>::InputVector m(x, y);
        double z = a*x + b*y + c;
        double zz = lwpls.prediction(m);
        cout << x << " " << y << " --> " << abs(zz-z) << endl;
        errorMean += abs(zz-z);
        errorStd += abs(zz-z)*abs(zz-z);
        //assert(abs(zz-z) < sigma);
        Leph::Plot::Plot::add("Result", x, y, zz);
    }
    errorMean /= (double)500;
    errorStd /= (double)500;
    errorStd = sqrt(errorStd - errorMean*errorMean);
    cout << "==> " << errorMean << " " << errorStd << endl;
    Leph::Plot::Plot::plot();
    lwpls.print();

    return 0;
}

