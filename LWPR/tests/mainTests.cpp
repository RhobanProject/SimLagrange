#include <iostream>
#include <cassert>
#include <cmath>
#include "LWPR/src/LWPR.hpp"
#include "Random/src/Random.hpp"
#include "Plot/src/Plot.hpp"

using namespace std;
using namespace Leph::LWPR;

#define INPUTDIM 1

void displayActivationWeight(const LWPR<INPUTDIM>& lwpr, double offset)
{
    for (double x=-4.0;x<=4.0;x+=0.05) {
        LWPR<INPUTDIM>::InputVector m; m(0) = x;
        for (size_t i=0;i<lwpr.getReceptiveFieldCount();i++) {
            double w = lwpr.getReceptiveField(i)
                .activationWeight(m);
            Leph::Plot::Plot::add(i, "Weight", x, w-offset);
            Leph::Plot::Plot::option(i, "Weight", "with lines");
        }
    }
}

int main()
{
    LWPR<INPUTDIM> lwpr(0.99, 0.2, 1e-6f, 0.1, 100*LWPR<INPUTDIM>::InputMatrix::Identity());
    lwpr.learning(LWPR<INPUTDIM>::InputVector::Zero(), 0.0);

    for (int k=0;k<150;k++) {
        double x = Leph::Random::Random::rangeDouble(-3.0, 3.0);
        double y = sin(2.0*x)+3.0*exp(-16.0*x*x)+sin(3.0*x);
        double yy = y + Leph::Random::Random::gaussian(0.0, 0.2);
        Leph::Plot::Plot::add("Noise", x, yy);
        LWPR<INPUTDIM>::InputVector m; m(0) = x;
        lwpr.learning(m, yy);
        if (k%10 == 0) {
            displayActivationWeight(lwpr, 1.0*k/10);
        }
    }

    for (double x=-4.0;x<=4.0;x+=0.05) {
        double y = sin(2.0*x)+3.0*exp(-16.0*x*x)+sin(3.0*x);
        LWPR<INPUTDIM>::InputVector m; m(0) = x;
        double yy = lwpr.prediction(m);
        //cout << "PRINT " << x << " " << yy << endl;
        Leph::Plot::Plot::add("Result", x, yy);
        Leph::Plot::Plot::add("Target", x, y);
    }
    Leph::Plot::Plot::option("Target", "with lines");
    Leph::Plot::Plot::option("Result", "with lines");

    Leph::Plot::Plot::plot();

    return 0;
}

