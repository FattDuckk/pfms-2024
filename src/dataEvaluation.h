#ifndef GRADIENT_H
#define GRADIENT_H
#include <cmath>

/**
 * @brief Class to evaluate the gradient of the path
 * I made this so it could be used for mission an utest.
 * I didn't but I really should have planned ahead.
 */

class DataEvaluator {
public:
    /**
     * @brief Function to evaluate the gradient of the path
     */
    double evaluateGradient(double height1, double height2, double distance);
};

#endif