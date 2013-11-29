#include <iostream>
#include <cassert>
#include "Plot/src/Plot.hpp"

using namespace std;
using namespace Leph::Plot;

int main()
{
    Plot::option("set grid;");
    Plot::add("test1", 1.0);
    Plot::add("test1", 2.0);
    Plot::add("test1", 3.0);
    Plot::add("test2", 0.0, 2.0);
    Plot::add("test2", 1.0, 2.0);
    Plot::add("test2", 2.0, 2.0);
    Plot::add("test2", 3.0, 3.0);
    Plot::option("test2", "with lines");
    Plot::add("test3", 2.0, 2.0, 4.0);
    Plot::add("test3", 3.0, 3.0, 4.0);
    Plot::add("test3", 4.0, 4.0, 3.0);
    Plot::add("test3", 5.0, 5.0, 2.0);
    Plot::option("test3", "with lines");
    Plot::plot();

    return 0;
}

