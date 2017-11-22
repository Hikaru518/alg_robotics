/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>

// Add Eigen Library
#include <Eigen/Dense>
#include <Eigen/LU>

// Add other library
#include <vector>
#include <math.h>

// Setting parameters
const int n = 3;
std::vector<double> l(n+1);
std::vector<double> m(n+1);
const double g = 9.81;

// setting limit;
const double velocity_limit = 10;

// define type
typedef std::pair<double, double> Point2D;
typedef std::vector<Point2D> Rect;

namespace ob = ompl::base;
namespace oc = ompl::control;
using Eigen::MatrixXd;

//lineIntersection
bool lineIntersection(Point2D ours0, Point2D ours1, Point2D theirs0, Point2D theirs1)
{
    return true;
}

void getPhi(std::vector<double> phi, const oc::ODESolver::StateType& q){
    phi[0] = q[0];
    for (int i = 1;i<n;i++){
        phi[i] = q[i] + phi[i-1];
    }
}

void getXY(const std::vector<double> phi, std::vector<double> x, std::vector<double> y){
    x[n] = 0; y[n]=0;
    for (int i = n-1; i >= 0; i--)
    {
        x[i] = x[i+1] + l[i]*cos(phi[i]);
        y[i] = y[i+1] + l[i]*sin(phi[i]);
    }
}

// i,j,phi,JL
MatrixXd getPsi_ij(const std::vector<MatrixXd> JL, const std::vector<double> phi, int i, int j){
    MatrixXd tM(2,n);
    MatrixXd tPsi(n,n);
    tM = getJL_dqj(j,i,phi);
    tPsi = tM.transpose()*JL[i]+JL[i].transpose()*tM;
    return tPsi;
}

MatrixXd getJL_dqj(int j, int i, const std::vector<double> phi){
    MatrixXd JL_dqj(2,n);
    JL_dqj.setZero(2,n);
    double r;
    for (int a = 0; a < n; ++a)
    {
        r = max(a,j);
        if (a<=i and a<=j)
        {
            JL_dqj(0,a) = -x[r] + x[i] - 0.5*l[i]*cos(phi[i]);    
            JL_dqj(1,a) = -y[r] + y[i] - 0.5*l[i]*sin(phi[i]);
        }
        else
        {
            JL_dqj(0,a) = 0;
            JL_dqj(1,a) = 0;
        }
    }
    return JL_dqj;
}

MatrixXd expandtM(MatrixXd tM, MatrixXd Psi, MatrixXd q_dot, int j){
    MatrixXd mat(1,n);
    MatrixXd result = tM;
    mat = q_dot.transpose()*Psi;
    for (int i = 0; i < n; ++i)
    {
        result(j,i) = mat(0,i);
    }
    return result;
}

void get_h_H(MatrixXd H, MatrixXd h, vector<double> phi, const oc::ODESolver::StateType& q){
    ///int N = q.size() - 1;
    std::vector<MatrixXd> JL;
    std::vector<MatrixXd> JA;
    MatrixXd tmpJL(2,n);

    // Calculate JL
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
                if(j<i){
                    tmpJL(0,j) = -y[j] + y[i] - 0.5*l[i]*sin(phi[i]);
                    tmpJL(1,j) = x[j]  - x[i] + 0.5*l[i]*cos(phi[i]);
                }
                if (j == i)
                {
                    tmpJL(0,j) = -0.5*l[i]*sin(phi[i]);
                    tmpJL(1,j) = 0.5*l[i]*cos(phi[i]);
                }
                if (j>i)
                {
                    tmpJL(0,j) = 0;
                    tmpJL(1,j) = 0;
                }
        }
        JL.push_back(tmpJL);
    }

    // Calculate JA*I*JA
    MatrixXd tmpJA(n,n);
    for (int i = 0; i < n; ++i)
    {
        tmpJA.setZero(n,n);
        setIdentity(tmpJA,i);
        tmpJA = m[i]*l[i]*l[i]*(1/12.0)*tmpJA;
        JA.push_back(tmpJA);
    }

    // Calculate H
    H.setZero(n,n);
    for (int i = 0; i < n; ++i)
    {
        H = H + m[i]*JL[i].transpose()*JL[i] + JA[i];        
    }

    // Calculate h
    // get q_dot
    MatrixXd q_dot(n,1);
    for (int i = 0; i < n; ++i)
    {
        q_dot(i,1) = q[n+i];
    }

    MatrixXd sum2(n,1);
    MatrixXd sum1(n,1);
    sum2.setZero(n,1);
    sum1.setZero(n,1);
    MatrixXd tmpMatPsi(n,n);
    MatrixXd tM(n,n);
    for (int i = 0; i < n; ++i)
    {
        tM.setZero(n,n);
        for (int j = 0; j < n-1; ++j)
        {
            tmpMatPsi = getPsi_ij(i,j,phi,JL);
            sum2 = sum2 + tmpMatPsi * q_dot(j,0);
            tM = expandtM(tM, tmpMatPsi, q_dot, j);
        }
        sum1 = sum1 + m[i]*((sum2*q_dot)-0.5*tM*q_dot);
    }
    h = sum1;
}

void setIdentity(MatrixXd m,int n){
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            m(j,i) = 1;
        }
    }
}

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void robotODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double s1 = q[0];
    const double s2 = q[1];
    const double s3 = q[2];
    const double s_1 = q[3];
    const double s_2 = q[4];
    const double s_3 = q[5];
    
    vector<double> phi(n);
    getPhi(phi,q);

    MatrixXd H(n,n);
    MatrixXd h(n,1);
    MatrixXd G(n,1);
    MatrixXd t(n,1); // Torque;
    MatrixXd q_d2(n,1); //q_dot_dot

    //for (int i = 0; i < n; ++i)
    //{
    //    q_d2(i,0) = q[n+i]; 
    //}
    
    double sum = 0;
    G(n-1,0) = g*m[n-1]*(0.5*l[n-1]*cos(phi[n-1]));
    for (int i = n-2;i>=0;i--){
        sum = 0;
        for(int k = i; k<n;k++)
        {
            sum = sum + m[k]*l[i]*cos(phi[i]);
        }
        G(i,0) = G(i+1) + g*(m[i]*(0.5*l[i])*cos(phi[i]) + sum);
    }

    get_h_H(H, h, phi, q);
    
    q_d2 = H.transpose()*(t - h - G);

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = s_1;
    qdot[1] = s_2;
    qdot[2] = s_3;

    qdot[3] = q_d2[0];
    qdot[4] = q_d2[1];
    qdot[5] = q_d2[2];
}

// This is a callback method invoked after numerical integration.
void robotPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

bool checkSelf(std::vector<Point2D> pts){
    int N = pts.size();
    for (int i = 0; i < N-1; ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            lineIntersection(pts[i],pts[i+1],pts[j],pts[j+1])
        }
    }
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    /// cast the abstract state type to the type we expect
    const ompl::base::CompoundState* cstate;
    cstate = state->as<ob::CompoundState>();

    //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const auto *rot = cstate->as<ob::SO2StateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const auto *vel = cstate->as<ob::RealVectorStateSpace::StateType>(1);

    /// check validity of state defined by rotat & rot
    for (int i = 0; i < n; ++i)
    {
        if (vel[i] > velocity_limit)
        {
            return false;
        }
        else if(vel[i] < - velocity_limit){
            return false;
        }
        else{
            return true;
        }
    }

    /// collision checking
    /// store nodes
    std::vector<Point2D> pts;
    for (int i = 0; i < n; ++i)
    {
        pts.push_back(std::make_pair(x[i],y[i]));
    }
    if( checkSelf(pts) == false){
        return false;
    }
    else if(/*checkObstacles*/ 1 ){
        return true;
    }
    else{
        return true;
    }

}

/// @cond IGNORE
class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, n)
    {
    }
};
/// @endcond

void planWithSimpleSetup()
{
    /// construct the state space we are planning in
    //auto space(std::make_shared<ob::SE2StateSpace>());
    ompl::base::StateSpacePtr space;
    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
    ompl::base::StateSpacePtr r(new ompl::base::RealVectorStateSpace(n));

    ompl::base::RealVectorBounds velocity_limit(n);
    velocity_limit.setLow(-velocity_limit);
    velocity_limit.setHigh(velocity_limit);
    r->as<ompl::base::RealVectorStateSpace>()->setBounds(velocity_limit);

    // set state space
    space = so2;
    for (int i = 0; i < n-1; ++i)
    {
        ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
        space = space + so2;
    }
    space = space + r;

    ////    space = r2 + so2 +  r;


    /// set the bounds for the R^n part of SE(2)
    ob::RealVectorBounds bounds(n);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(n);
    cbounds.setLow(-10);
    cbounds.setHigh(10);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });

    // Setting the propagation routine for this space:
    // KinematicCarModel does NOT use ODESolver
    //ss.setStatePropagator(std::make_shared<KinematicCarModel>(ss.getSpaceInformation()));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &robotODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &robotPostIntegration));

    /// create a start state
    ob::ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;
    start[3] = 0.0;
    start[4] = 0.0;
    start[5] = 0.0;
    
    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<> goal(space);
    goal[0] = 3.14159;
    goal[1] = 0.0;
    goal[2] = 0.0;
    goal[3] = 0.0;
    goal[4] = 0.0;
    goal[5] = 0.0;

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);
    
    // planner 
    ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    /// we want to have a reasonable value for the propagation step size
    ss.setup();

    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(20.0);

    std::ofstream fout("path.txt");
    path.printAsMatrix(fout);
    fout.close();

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen

        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // Input length, calculate mass
    l[0] = 3;
    l[1] = 2;
    l[2] = 1;
    l[3] = 0;
    m = l;


    planWithSimpleSetup();

    return 0;
}