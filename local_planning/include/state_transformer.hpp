#ifndef STATE_TRANSFORMER_HPP_
#define STATE_TRANSFORMER_HPP_

#include "lane.hpp"
#include "frenet_state.hpp"

class StateTransformer {
 public:
    StateTransformer() {}
  
    StateTransformer(const Lane& lane) { 
        lane_ = lane; 
    }

    ErrorType GetStateFromFrenetState(const FrenetState& fs, State* s) const;

    /**
     *  ~ note that this function may introduce approx 1cm error in vec position
     *  ~ due to the finite sampling strategy. the time consumed is about 0.03ms.
     */
    ErrorType GetFrenetStateFromState(const State& s, FrenetState* fs) const;

    ErrorType GetFrenetStateVectorFromStates(const vec_E<State> state_vec, vec_E<FrenetState>* fs_vec) const;

    ErrorType GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec, vec_E<State>* state_vec) const;

    ErrorType GetFrenetPointFromPoint(const Vec2f& s, Vec2f* fs) const;

    ErrorType GetFrenetPointVectorFromPoints(const vec_E<Vec2f>& s, vec_E<Vec2f>* fs) const;

    bool IsValid() const { 
        return lane_.IsValid(); 
    }

    void print() {}

 private:
    Lane lane_;
};

#endif  // STATE_TRANSFORMER_HPP_