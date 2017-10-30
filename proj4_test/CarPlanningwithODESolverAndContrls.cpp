
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <cmath>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Definition of the ODE for the pendulum.
void PendulumODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
   
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[0];
    const double omega = q[1];

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = omega;
    qdot[1] = -9.81*cos(theta) + u[0];
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    /// cast the abstract state type to the type we expect
    const ob::CompundState* cstate;
    cstate = state->as<ob::CompundState>();

    /// extract the first component of the state and cast it to what we expect
    //const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::SO2StateSpace::StateType *rot = cstate->as<ob::SO2StateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    //const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType * vel = cstate->as<ob::RealVectorStateSpace::StateType>(1);

    /// check validity of state defined by pos & rot
    if (vel->values[0] > 10)
	return false;
    else if(vel->values[0] < -10)
        return false;
    else
        return true;
}

/// @cond IGNORE
class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 1)
    {
    }
};
/// @endcond

void planWithSimpleSetup()
{
    /// construct the state space we are planning in
    auto space(std::make_shared<ob::StateSpace>());
    auto so2(std::make_shared<ob::SO2StateSpace>());
    auto r(std::make_shared<ob::RealVectorStateSpace>());

    ///set the bounds of control space
    ob::RealVectorBounds velocity_limit(1);
    velocity_limit.setLow(-10);///10 is the velocity limit of the pendulum
    velocity_limit.setHigh(10);
    r->as<ob::RealVectorStateSpace>()->setBounds(velocity_limit);

    space = so2 + r;

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-3);
    cbounds.setHigh(3);
   
    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(isStateValid);

    // Use the ODESolver to propagate the system. 
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &PendulumODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    /// create a start state
    ob::ScopedState<> start(space);
    start[0]=-3.1415926/2;
    start[1]=0.0;

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<> goal(space);
    goal[0]=3.1415926/2;
    goal[1]=0.0;
   
    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.10);

    ///RRT
    ss.getSpaceInformation()->setPropagationStepSize(0.01);
    ob::PlannerPtr Planner(new oc::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    /// we want to have a reasonable value for the propagation step size
    ss.setup();

    /// attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen
	oc::PathControl& path = ss.getSolutionPath();
	path.printAsMatrix(std::cout);

	// print path to file
	std::ofstream fout("path.txt");
	fout << "Pendulum" << std::endl;
	fout.close();
        //ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();

    return 0;
}
