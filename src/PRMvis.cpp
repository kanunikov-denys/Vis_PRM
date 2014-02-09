/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Denys Kanunikov */

#include "ompl/geometric/planners/prm/PRMvis.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#include "GoalVisitor.hpp"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

namespace ompl
{
    namespace magic
    {

        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 5;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;
    }
}


ompl::geometric::PRMvis::PRMvis(const base::SpaceInformationPtr &si) : PRM(si, false)
{
    setName("PRMvis");
    params_.remove("max_nearest_neighbors");
}

void ompl::geometric::PRMvis::growRoadmap(double growTime)
{
    growRoadmap(base::timedPlannerTerminationCondition(growTime));
}

void ompl::geometric::PRMvis::growRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    base::State *workState = si_->allocState();
    growRoadmap (ptc, workState);
    si_->freeState(workState);
}

void ompl::geometric::PRMvis::addGuard(const base::State *workState){
    OMPL_INFORM("%s: Add guard node", getName().c_str());
    si_->printState(workState, std::cout);
    std::cout << std::endl;

    std::list<base::State*> newGuard;
    newGuard.push_back(si_->cloneState(workState));
    guards.push_back(newGuard);
//    checkedStates.insert(si_->cloneState(workState));
}

void ompl::geometric::PRMvis::mergeVisComponents(
        std::list<base::State*>* into, std::list<base::State*>* it_Gi){
    OMPL_INFORM("%s: mergeVisComponents", getName().c_str());
    for (std::list<base::State*>::const_iterator it = it_Gi->begin();
         it != it_Gi->end(); it++){
        into->push_back(*it);
    }
    it_Gi->clear();
}

bool ompl::geometric::PRMvis::check_guard_or_connector(const base::State *workState){
    OMPL_INFORM("%s: check guards or connector for ", getName().c_str());
    si_->printState(workState, std::cout);
    std::cout << std::endl;

    bool found;
    bool isGuard = false;
    bool isConnector = false;
    base::State* g_vis = NULL;
    std::list<base::State*> *G_vis = NULL;
    for (std::list<std::list<base::State*> >::iterator it_Gi = guards.begin();
         it_Gi != guards.end(); it_Gi++ ){
        found = false;

        for (std::list<base::State*>::iterator it_g = it_Gi->begin();
             (it_g != it_Gi->end() && (!found)); it_g++){
            std::cout << "taken state: \n";
                si_->printState(*it_g, std::cout);
            if (si_->checkMotion((*it_g), workState)){
                si_->printState(workState, std::cout);
                std::cout << " visible by ";
                si_->printState(*it_g, std::cout);
                std::cout << std::endl;
                found = true;
                if (!g_vis) {
                    g_vis = (*it_g);
                    G_vis = &(*it_Gi);
                }
                else /*connection node*/{
                    OMPL_INFORM("%s: Add Connector", getName().c_str());
                    connectors.push_back(si_->cloneState(workState));
//                    checkedStates.push_back(si_->cloneState(workState));
                    isConnector = true;
                    /*connection is made later*/
                    mergeVisComponents(G_vis, &(*it_Gi));
//                    OMPL_INFORM("%s: test1", getName().c_str());
////                    guards.remove(*it_Gi);
//                    OMPL_INFORM("%s: test2", getName().c_str());
                }
            }
        }
//        OMPL_INFORM("%s: test20", getName().c_str());
    }
    if (!G_vis) /*guard_node*/{
        addGuard(workState);
        isGuard = true;
    }
    return (isGuard || isConnector);
}

bool ompl::geometric::PRMvis::isEqualState(const base::State* A, const base::State* B){
    std::cout << "Comparing states: \n";
    si_->printState(A, std::cout);
    si_->printState(B, std::cout);
    std::cout << std::endl;
    return si_->equalStates(A, B);
}

bool ompl::geometric::PRMvis::isChecked(base::State* _state){
    foreach (base::State* i, checkedStates) {
        if (isEqualState(_state, i)) return true;
    }
    return false;
}

/*поиск следующей валидной вершины*/
void ompl::geometric::PRMvis::growRoadmap(const base::PlannerTerminationCondition &ptc,
                                       base::State *workState)
{
    OMPL_INFORM("%s: growRoadMap", getName().c_str());
    /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
    while (ptc == false)
    {
        // search for a valid state
        bool found = false;
        bool checked;
        bool valid = false;
        while (!found && ptc == false)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(workState);
                valid = si_->isValid(workState);
                checked = false;
                std::cout << "Checking for a state: ";
                si_->printState(workState, std::cout);
                std::cout << "found " << found << " " << valid << std::endl;
//                if (found && (checkedStates.find(workState) == checkedStates.end())){
                if (found && (!isChecked(workState))){
                    checked = false;
                    checkedStates.push_back(si_->cloneState(workState));
                }
                else{checked = true;
                    std::cout << "Already checked\n";
                    found = false;
                }
                //checker for guard and connection
                if (!checked) found = check_guard_or_connector(workState);
                if (!found && !checked) std::cout << "Not sufficiant of guard connection problem\n";
                attempts++;
            } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
        }
        // add it as a milestone
        if (found)
            addMilestone(si_->cloneState(workState));
    }
}

//добавление вершины в граф
ompl::geometric::PRMvis::Vertex ompl::geometric::PRMvis::addMilestone(base::State *state)
{
    OMPL_INFORM("%s: addMilestone", getName().c_str());
    si_->printState(state, std::cout);
    boost::mutex::scoped_lock _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    nn_->add(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex>& neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(m, n))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[m], stateProperty_[n]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
                const unsigned int id = maxEdgeID_++;
                const Graph::edge_property_type properties(weight, id);
                boost::add_edge(m, n, properties, g_);
                uniteComponents(n, m);
            }
        }

    return m;
}

void ompl::geometric::PRMvis::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("%s: construct roadmap", getName().c_str());
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State*> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    while (ptc() == false)
    {
        // maintain a 2:1 ratio for growing/expansion of roadmap
        // call growRoadmap() twice as long for every call of expandRoadmap()
//        if (grow)
            growRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(2.0 * magic::ROADMAP_BUILD_TIME)), xstates[0]);
//        else
//            expandRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(magic::ROADMAP_BUILD_TIME)), xstates);
        grow = !grow;
    }

    si_->freeStates(xstates);
}



ompl::base::PlannerStatus ompl::geometric::PRMvis::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("%s: solver started\n", getName().c_str());
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart()) {
        startM_.push_back(addMilestone(si_->cloneState(st)));
        addGuard(si_->cloneState(st));
        checkedStates.push_back(si_->cloneState(st));
//        si_->printState(st, std::cout);
    }

    if (startM_.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st){
            goalM_.push_back(addMilestone(si_->cloneState(st)));
            checkedStates.push_back(si_->cloneState(st));
            addGuard(si_->cloneState(st));
        }

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nrStartStates);

    // Reset addedSolution_ member and create solution checking thread
    addedSolution_ = false;
    base::PathPtr sol;
    boost::thread slnThread(boost::bind(&PRMvis::checkForSolution, this, ptc, boost::ref(sol)));
    OMPL_INFORM("%s: thread created\n", getName().c_str());
    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerTerminationCondition ptcOrSolutionFound =
        base::plannerOrTerminationCondition(ptc, base::PlannerTerminationCondition(boost::bind(&PRMvis::addedNewSolution, this)));

    constructRoadmap(ptcOrSolutionFound);

    // Ensure slnThread is ceased before exiting solve
    slnThread.join();
    assert(false);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    if (sol)
    {
        base::PlannerSolution psol(sol);
        // if the solution was optimized, we mark it as such
        if (addedNewSolution())
            psol.optimized_ = true;
        pdef_->addSolutionPath (psol);
    }

    // Return true if any solution was found.
    return sol ? (addedNewSolution() ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::APPROXIMATE_SOLUTION) : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::PRMvis::checkForSolution(const base::PlannerTerminationCondition &ptc,
                                            base::PathPtr &solution)
{
    base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    while (!ptc && !addedSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        // Check for a solution
        addedSolution_ = haveSolution(startM_, goalM_, solution);
        // Sleep for 1ms
        if (!addedSolution_)
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}

bool ompl::geometric::PRMvis::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    base::Cost sol_cost(0.0);
    bool sol_cost_set = false;
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                base::PathPtr p = constructSolution(start, goal);
                if (p)
                {
                    // Check if optimization objective is satisfied
                    base::Cost pathCost = p->cost(opt_);
                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        return true;
                    }
                    else if (!sol_cost_set || opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                        sol_cost_set = true;
                    }
                }
            }
        }
    }

    return false;
}

bool ompl::geometric::PRMvis::addedNewSolution(void) const
{
    return addedSolution_;
}
///*single thread*/
//ompl::base::PlannerStatus ompl::geometric::PRMvis::solve(const ompl::base::PlannerTerminationCondition& ptc)
//{
//    OMPL_INFORM("%s: solver started\n", getName().c_str());
//    checkValidity();
//    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

//    if (!goal)
//    {
//        OMPL_ERROR("Goal undefined or unknown type of goal");
//        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
//    }

//    // Add the valid start states as milestones
//    while (const base::State *st = pis_.nextStart()) {
//        startM_.push_back(addMilestone(si_->cloneState(st)));
//        addGuard(si_->cloneState(st));
//    }

//    if (startM_.size() == 0)
//    {
//        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
//        return base::PlannerStatus::INVALID_START;
//    }

//    if (!goal->couldSample())
//    {
//        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
//        return base::PlannerStatus::INVALID_GOAL;
//    }

//    // Ensure there is at least one valid goal state
//    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
//    {
//        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
//        if (st){
//            goalM_.push_back(addMilestone(si_->cloneState(st)));
//            addGuard(si_->cloneState(st));
//        }

//        if (goalM_.empty())
//        {
//            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
//            return base::PlannerStatus::INVALID_GOAL;
//        }
//    }

//    unsigned int nrStartStates = boost::num_vertices(g_);
//    OMPL_INFORM("Starting with %u states", nrStartStates);
//    if (!sampler_)
//        sampler_ = si_->allocValidStateSampler();
//    if (!simpleSampler_)
//        simpleSampler_ = si_->allocStateSampler();

//    std::vector<base::State*> xstates(MAX_RANDOM_BOUNCE_STEPS);
//    si_->allocStates(xstates);
//    bool grow = true;

//    // Reset addedSolution_ member
//    addedSolution_ = false;
//    base::PathPtr sln;
//    sln.reset();

//    double roadmap_build_time = 0.05;
//    while (ptc == false && !addedSolution_)
//    {
//        // Check for any new goal states
//        if (goal->maxSampleCount() > goalM_.size())
//        {
//            const base::State *st = pis_.nextGoal();
//            if (st)
//                goalM_.push_back(addMilestone(si_->cloneState(st)));
//        }

//        // maintain a 2:1 ratio for growing/expansion of roadmap
//        // call growRoadmap() twice as long for every call of expandRoadmap()
//        if (grow)
//            ompl::geometric::PRM::growRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(2.0*roadmap_build_time)), xstates[0]);
//        else
//            ompl::geometric::PRM::expandRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(roadmap_build_time)), xstates);
//        grow = !grow;

//        // Check for a solution
//        addedSolution_ = haveSolution (startM_, goalM_, sln);
//    }

//    OMPL_INFORM("Created %u states", boost::num_vertices(g_) - nrStartStates);

//    if (sln)
//    {
//        if(addedSolution_)
//            pdef_->addSolutionPath (sln);
//        else
//            // the solution is exact, but not as short as we'd like it to be
//            pdef_->addSolutionPath (sln, true, 0.0);
//    }

//    si_->freeStates(xstates);

//    // Return true if any solution was found.
//    return sln ? (addedSolution_ ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::APPROXIMATE_SOLUTION) : base::PlannerStatus::TIMEOUT;
//}
