/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

#ifndef PRMVIS_H
#define PRMVIS_H

/*@Author: Denys Kanunikov */

#include "ompl/geometric/planners/prm/PRM.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor PRMvis
           Run PRM visibility algorithm
           @par Short description
           @par External documentation
            Visibility-based probabilistic roadmaps for motion planning
            T. SIMÉON, J.-P. LAUMOND and C. NISSOUX
            LAAS-CNRS, 7 avenue du Colonel-Roche, 31077 Toulouse, France
        */

        /** \brief Visibility PRM planner */
        class PRMvis : public PRM
        {
        public:

            /** \brief Constructor */
            PRMvis(const base::SpaceInformationPtr &si);

            /** \brief Function that can solve the motion planning
                problem. Grows a roadmap using
                constructRoadmap(). This function can be called
                multiple times on the same problem, without calling
                clear() in between. This allows the planner to
                continue work for more time on an unsolved problem,
                for example. Start and goal states from the currently
                specified ProblemDefinition are cached. This means
                that between calls to solve(), input states are only
                added, not removed. When using PRM as a multi-query
                planner, the input states should be however cleared,
                without clearing the roadmap itself. This can be done
                using the clearQuery() function. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void growRoadmap(double);
            virtual void growRoadmap(const base::PlannerTerminationCondition &ptc);
            virtual void growRoadmap(const base::PlannerTerminationCondition &ptc,
                                                   base::State *workState);
            virtual ompl::geometric::PRMvis::Vertex addMilestone(base::State *state);

            virtual bool check_guard_or_connector(base::State* workState);
            void constructRoadmap(const base::PlannerTerminationCondition &ptc);
        protected:
            void addGuard(base::State *workState);
            void mergeVisComponents(std::list<base::State*>*, std::list<base::State*>*);
            /** Thread that checks for solution */
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution);

            /** \brief Returns the value of the addedSolution_ member. */
            bool addedNewSolution(void) const;
        private:
            std::list<std::list<base::State*> > guards;
            std::set<base::State*> checkedStates;
            std::list<base::State*> connectors;
        };
    }
}

#endif // PRMVIS_H
