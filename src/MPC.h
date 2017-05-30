#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// Evaluate a polynomial.
// @params coeffes: Eigen Vector of polynomial coeffecients.
// @params x: Double corresponding to x-value at which to evaluate polynomial f(x).
// @returns Double corresponding to polynomial value evaluated at x.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// @parmas xvals: Eigen Vector of independant variable.
// @params yvals: Eigen Vector of dependant variable.
// @params order: Order of polynomial to be fitted.
// @returns Eigen Vector of polynomial coeffecients.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

// Transform points form world coordinate frame to vehicle coordinate frame.
// @params ptsx: Vector of world frame x-coordinates
// @params ptsy: Vector of world frame y-coordinates
// @params px: Vehicle world frame x-coordinate
// @params py: Vehicle world frame y-coordinate
// @params psi: Vehicle world frame pose
// @params new_ptsx: Pointer to vector that will hold world x-coordinates in vehicle coordinate frame.
// @params new_ptsy: Pointer to vector that will hold world y-coordinates in vehicle coordinate frame.
void TransformToVehicleCoordinates(vector<double> ptsx, vector<double> ptsy, double px, double py,
                                   double psi, vector<double> *new_ptsx, vector<double> *new_ptsy);

class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
