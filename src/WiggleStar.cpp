#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/wiggleStar/WiggleStar.h"
#include <limits>


namespace ompl {

    // Pretty much the only function we have to implement
    base::PlannerStatus control::WiggleStar::solve( const base::PlannerTerminationCondition &ptc )
    {
        // Checks whether the problem definition has a start and a goal
        checkValidity();

        // get start and goal states
        base::Goal *goal = pdef_->getGoal().get();
        const base::State *start = pis_.nextStart();

        // So there's base::SpaceInfo and control::SpaceInfo
        // SpaceInfo has some control-related functions that we want so
        // we'll make one and use that
        control::SpaceInformation *siC_ = si_.get();

        // Getting things ready for sampling and propagation
        control::ControlSamplerPtr controlSampler = siC_->allocControlSampler();
        control::StatePropagatorPtr propagator = siC_->getStatePropagator();
        unsigned int duration = siC_->getMaxControlDuration();

        // The things we'll be using to keep track of our path
        base::State *state;
        base::State *next;
        Control *control;
        std::vector<base::State*> states;
        std::vector<Control*> controls;

        // Get things ready to start planning (finally!)
        siC_->copyState( state, start );
        states.push_back( state );
        bool solved = false;
        double distance = std::numeric_limits<double>::max();

        // while we stil have time and haven't finished
        while( !ptc && !solved )
        {
            // allocate and sample a new control
            control = siC_->allocControl();
            controlSampler->sample( control, state );

            // Apply that control
            next = siC_->allocState();
            propagator->propagate( state, control, duration, next );

            // Make sure it's valid (i.e. doesn't collide)
            if ( !siC_->checkMotion( state, next ) ) {
                siC_->freeControl( control );
                siC_->freeState( next );
                continue;
            }

            // add the control and next state to our path
            controls.push_back( control );
            states.push_back( next );
            state = next;

            // Check to see whether we solved it!
            solved = goal->isSatisfied( state, &distance );
        }

        PathControl *path = new PathControl( si_ );
        for (unsigned int i = 0; i < controls.size()-1; ++i ) {
            path->append( states[i], controls[i], duration );
        }
        //last state doesn't have a control associated with it
        path->append( states.back() );
        pdef_->addSolutionPath( base::PathPtr(path), !solved, distance, getName() );

        if (solved) {
          return base::PlannerStatus::EXACT_SOLUTION;
        } else {
          return base::PlannerStatus::APPROXIMATE_SOLUTION;
        }
    }

}
