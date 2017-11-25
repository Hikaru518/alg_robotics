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
#include <ompl/control/planners/rrt/RRT.h>

// Add Eigen Library
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Dense>

// Add other library
#include <vector>
#include <math.h>

#include "isLineIntersect.h"

// Setting parameters
int n;
int envType;
std::vector<double> l;
std::vector<double> m;
std::vector<double> x(100),y(100);
double time_limit;
const double g = 9.81;

// setting and torque limit;
double velocity_limit = 10;
double t_limit = 100;

// define type
typedef std::pair<double, double> Point2D;
typedef std::vector<Point2D> Rect;

namespace ob = ompl::base;
namespace oc = ompl::control;
using Eigen::MatrixXd;
using namespace std::placeholders;

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
    x[n-1] = 0; y[n-1]=0;
    for (int i = n-2; i >= 0; i--)
    {
        x[i] = x[i+1] + l[i]*cos(phi[i]);
        y[i] = y[i+1] + l[i]*sin(phi[i]);
    }
}

MatrixXd getJL_dqj(int j, int i, const std::vector<double> phi){
    MatrixXd JL_dqj(2,n);
    JL_dqj.setZero(2,n);
    double r;
    for (int a = 0; a < n; ++a)
    {
        r = (a>j?a:j);
        //r = (a >j? a:j);
        if (a<=i and j<=i)
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

MatrixXd getPsi_ij(std::vector<MatrixXd> JL, const std::vector<double> phi, int i, int j){
    MatrixXd tM(2,n);
    MatrixXd tPsi(n,n);
    tM = getJL_dqj(j,i,phi);
    tPsi = tM.transpose()*JL[i]+JL[i].transpose()*tM;
    return tPsi;
}

MatrixXd expandtM(MatrixXd tM, MatrixXd Psi, MatrixXd q_dot, int j,const std::vector<double> phi){
    MatrixXd mat(1,n);
    MatrixXd result = tM;
    mat = q_dot.transpose()*Psi;
    for (int i = 0; i < n; ++i)
    {
        result(j,i) = mat(0,i);
    }
    return result;
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

void get_h_H(MatrixXd& H, MatrixXd& h, std::vector<double> phi, const oc::ODESolver::StateType& q){
    ///int N = q.size() - 1;
    std::vector<MatrixXd> JL;
    std::vector<MatrixXd> JA;
    MatrixXd tmpJL(2,n);
    //std::cout << l[n-1] << std::endl;
    /*for(int i = 0;i < n; i++)
    {
        std::cout << "phi[" << i << "] = " << phi[i] << " ";
    }
    std::cout << endl;*/

    //std::cout << "define two vector MatrixXd" <<std::endl;
    
    // Calculate JL
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
                if(j<i){
                    tmpJL(0,j) = -y[j] + y[i] - 0.5*l[i]*sin(phi[i]);
                    tmpJL(1,j) = x[j]  - x[i] + 0.5*l[i]*cos(phi[i]);
                    //std::cout << "j<i" <<std::endl;
                }
                if (j == i)
                {
                    tmpJL(0,j) = -0.5*l[i]*sin(phi[i]);
                    tmpJL(1,j) = 0.5*l[i]*cos(phi[i]);
                    //std::cout << "j=i" <<std::endl;
                }
                if (j>i)
                {
                    tmpJL(0,j) = 0;
                    tmpJL(1,j) = 0;
                    //std::cout << "j>i" <<std::endl;
                }
                //std::cout << "finished: (" << i << "," << j<<")"<<std::endl; 
        }
        JL.push_back(tmpJL);
        //std::cout << "pushed_back (" <<i<<")"<<std::endl;
    }
    //std::cout << "Calculate JL"<< std::endl;

    // Calculate JA*I*JA
    MatrixXd tmpJA(n,n);
    for (int i = 0; i < n; ++i)
    {
        tmpJA.setZero(n,n);
        setIdentity(tmpJA,i);
        tmpJA = m[i]*l[i]*l[i]*(1/12.0)*tmpJA;
        JA.push_back(tmpJA);
    }
    //std::cout << "Calculate JA*I*JA"<<std::endl;

    // Calculate H
    H.setZero(n,n);
    for (int i = 0; i < n; ++i)
    {
        H = H + m[i]*JL[i].transpose()*JL[i] + JA[i];        
    }
    //std::cout << "Calculate H"<<std::endl;

    // Calculate h
    // get q_dot
    MatrixXd q_dot(n,1);
    for (int i = 0; i < n; ++i)
    {
        q_dot(i,0) = q[n+i];
    }
    //std::cout << "Calculate h---part1"<<std::endl;

    MatrixXd sum2(n,1);
    MatrixXd sum1(n,1);
    sum2.setZero(n,n);
    sum1.setZero(n,1);
    MatrixXd tmpMatPsi(n,n);
    MatrixXd tM(n,n);

    for (int i = 0; i < n; ++i)
    {
        tM.setZero(n,n);
        for (int j = 0; j < n-1; ++j)
        {
            tmpMatPsi = getPsi_ij(JL,phi,i,j);
	    //std::cout << tmpMatPsi << std::endl;
            //std::cout << "q_dot" << std::endl;
            //std::cout << q_dot(j,0) << std::endl;
            // Note !! Blow is the problematic line
            sum2 = sum2 + tmpMatPsi * q_dot(j,0);

            //std::cout << "Calculate h ---- part2"<<std::endl;

            tM = expandtM(tM, tmpMatPsi, q_dot, j,phi);
        }
        sum1 = sum1 + m[i]*((sum2*q_dot)-0.5*tM*q_dot);
    }
    h = sum1;
    //std::cout << "Calculate h ---- part3"<<std::endl;
}

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void robotODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    //std::cout <<"robotODE" << std::endl;
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> s,s_dot;
    
    for (int i = 0; i < n; i++)
    {
        s.push_back(q[i]);
        s_dot.push_back(q[i+n]);
    }
    //std::cout<<"getState"<<std::endl;
    /*
    const double s1 = q[0];
    const double s2 = q[1];
    
    const double s3 = q[2];
    const double s_1 = q[3];
    const double s_2 = q[4];
    const double s_3 = q[5];
    //const double s_1 = q[1];    
    **/

    std::vector<double> phi(n);
    getPhi(phi,q);

    MatrixXd H(n,n);
    MatrixXd h(n,1);
    MatrixXd G(n,1);
    MatrixXd t(n,1); // Torque;
    MatrixXd q_d2(n,1); //q_dot_dot

    for(int i = 0;i<n;i++)
    {
        t(i,0) = u[i];
    }

    //std::cout << "defineMatrix"<<std::endl;
    
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
    //std::cout<<"calculate G"<<std::endl;

    get_h_H(H, h, phi, q);
    
    //std::cout <<"calculate h and H"<<std::endl;

    q_d2 = H.transpose()*(t - h - G);

    // Zero out qdot
    qdot.resize (q.size (), 0);

    for (int i = 0; i<n; i++)
    {
        qdot[i] = s_dot[i];
        qdot[n+i] = q_d2(i,0);
    }
    //qdot[0] = q[1];
    //qdot[1] = -g*cos(q[0]) + u[0];
    

    /*qdot[0] = s_1;
    qdot[1] = s_2;
    qdot[2] = s_3;
    //qdot[1] = q_d2(0,0);
    qdot[3] = q_d2(0,0);
    qdot[4] = q_d2(1,0);
    qdot[5] = q_d2(2,0);*/

    //std::cout <<"ODE____end" << std::endl;
}

// This is a callback method invoked after numerical integration.
void robotPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    //SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(0));
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0));
}

bool checkSelf(std::vector<Point2D> pts){
    int N = pts.size();
    bool result;
    for (int i = 0; i < N-1; ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            result = isLineIntersect(pts[i],pts[i+1],pts[j],pts[j+1]);
            if(result == false){
                return false;
            }
        }
    }
    return true;
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state )
{


    //std::cout << "isStateValid"<<std::endl;
    //    ob::ScopedState<ob::SE2StateSpace>
    /// cast the abstract state type to the type we expect
    const ompl::base::CompoundState* cstate;
    cstate = state->as<ob::CompoundState>();

    //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const auto *rot = cstate->as<ob::RealVectorStateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const auto *vel = cstate->as<ob::RealVectorStateSpace::StateType>(1);

    /// check validity of state defined by rotat & rot
    for (int i = 0; i < n; ++i)
    {
        if (vel->values[i] > velocity_limit)
        {
            return false;
        }
        else if(vel->values[i] < - velocity_limit){
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
    
    if(envType == 1){

	    //std::vector<std::vector<Rect>> obstacles;
	    std::vector<Rect> obstacles;
	    std::vector<Point2D> rect3,r;
	    rect3.push_back(std::make_pair(-0.5,-1.5));
	    rect3.push_back(std::make_pair(-0.5,-1.0));
	    rect3.push_back(std::make_pair(0.0,-1.0));
	    rect3.push_back(std::make_pair(0.0,-1.5));
	    obstacles.push_back(rect3);
	    //obstacles.push_back(narrow_obstacles);

	    for(int k = 0;k<obstacles.size();k++)
	    {
		for(int i = 0;i <obstacles[k].size();i++)
		{
		    for(int j = 0;j<pts.size();j++)
		    {
		        bool intersection = isLineIntersect(pts[j],pts[j+1],r[i],r[(i+1) % r.size()]);
		        if(intersection){
		            return false;
		        }
		    }
		}
	    }

    }
    return true;

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

void planWithSimpleSetup(std::vector<double> v1, std::vector<double> v2)
{
    /// construct the state space we are planning in
    //auto space(std::make_shared<ob::SE2StateSpace>());
    ompl::base::StateSpacePtr space;
    //ompl::base::StateSpacePtr r2(new ompl::base::SO2StateSpace());
    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(n));
    ompl::base::StateSpacePtr r(new ompl::base::RealVectorStateSpace(n));
    //ompl::base::StateSpacePtr r3(new ompl::base::SO2StateSpace());

    ompl::base::RealVectorBounds Bounds(n);
    Bounds.setLow(-velocity_limit);
    Bounds.setHigh(velocity_limit);
    r->as<ompl::base::RealVectorStateSpace>()->setBounds(Bounds);

    Bounds.setLow(-3.1415926);
    Bounds.setHigh(3.1415926);
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(Bounds);
    
    //std::cout <<"lalala" << std::endl;

    // set state space
    
    space = r2 + r;
    //std::cout <<"lalala" << std::endl;

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(n);
    cbounds.setLow(-t_limit);
    cbounds.setHigh(t_limit);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });


    //std::cout <<"lalala" << std::endl;

    // Setting the propagation routine for this space:
    // KinematicCarModel does NOT use ODESolver
    //ss.setStatePropagator(std::make_shared<KinematicCarModel>(ss.getSpaceInformation()));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &robotODE));
    //ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &robotPostIntegration));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    ss.getSpaceInformation()->setPropagationStepSize(0.05);

    //std::cout <<"lalala" << std::endl;

    /// create a start state
    ob::ScopedState<> start(space);
    for (int i = 0;i<2*n;i++){
        start[i] = v1[i];
    }
    
    //std::cout <<"start space" << std::endl;

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<> goal(space);
    for (int i = 0;i<2*n;i++){
        goal[i] = v2[i];
    }

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    // planner 
    ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    /// we want to have a reasonable value for the propagation step size
    ss.setup();

    //std::cout <<"setup" << std::endl;

    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(time_limit);

    
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ompl::control::PathControl& path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);
        /// print the path to screen
        std::ofstream fout("path.txt");
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{


    n = 3;
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    std::cout << "Please input n:";
    std::cin >> n;

    // Input length, calculate mass
    std::cout << "Please input length:" << std::endl;
    
    double tmp;
    for(int i = 0;i < n; i++)
    {
        std::cout << "link length[" << i+1 << "] : ";
        std::cin >>  tmp;
        l.push_back(tmp);
    }

    for(int i = 0;i<n;i++)
    {
        m.push_back(l[i]);
    }

    //Input start state
    std::vector<double> v1(2*n);
    std::cout << "Please input start state: (please input between -pi~pi)" << std::endl;
    for(int i = 0;i<n;i++){
        std::cout << "q[" << i+1 << "] = ";
        std::cin >> v1[i];
    }
    for(int i = n;i<2*n;i++){
        std::cout << "q_dot[" << i-n+1 << "] = ";
        std::cin >> v1[i];
    }

    //Input goal state;
    std::vector<double> v2(2*n);
    std::cout << "Please input goal state:" << std::endl;
    for(int i = 0;i<n;i++){
        std::cout << "q[" << i+1 << "] = ";
        std::cin >> v2[i];
    }
    for(int i = n;i<2*n;i++){
        std::cout << "q_dot[" << i-n+1 << "] = ";
        std::cin >> v2[i];
    }

    std::cout << "Please input velocity limit:" <<std::endl;
    std::cin >> velocity_limit;

    std::cout << "Please input torque limit:" << std::endl;
    std::cin >> t_limit;

    std::cout << "Please choose environment:" <<std::endl;
    std::cout << "0): a free environment with no obstacles." << std::endl;
    std::cout << "1): a environment with a square obstacle located in left lower plane." << std::endl;
    std::cout << "Type 0 or 1:";
    std::cin >> envType;

    std::cout << "Please input the time limit of RRT planner:" ;
    std::cin >> time_limit;

    planWithSimpleSetup(v1, v2);

    return 0;
}
