#include "dataEvaluation.h"




double DataEvaluator::evaluateGradient(double height1, double height2, double distance) {
    return (fabs(height2 - height1) / distance);
}