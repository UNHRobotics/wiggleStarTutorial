#ifndef OMPL_CONTROL_PLANNERS_WIGGLESTAR_WIGGLESTAR_
#define OMPL_CONTROL_PLANNERS_WIGGLESTAR_WIGGLESTAR_

#include "ompl/control/planners/PlannerIncludes.h"



namespace ompl
{
    namespace control
    {
        class WiggleStar : public base::Planner
        {
        public:
            // just using the parent constructor
            WiggleStar( const base::SpaceInformationPtr &si )
                : base::Planner( si, "WiggleStar" ) {};

            // Pretty much the only function we have to implement
            virtual base::PlannerStatus solve( const base::PlannerTerminationCondition &ptc );
        };
    }
}

#endif
