#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

/**
 * The StopAction makes the robot stop with the option to coast
 */
class StopAction : public Action
{
   public:
    // We consider the robot to be stopped when the magnitude of its velocity is less than
    // 5cm / s When robots are stationary, noise in the camera data can cause its velocity
    // to be non-zero, which is why we use a non-zero value here.
    static constexpr double ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT = 0.05;

    /**
     * Creates a new StopAction
     *
     * @param stopped_speed_threshold How slow the robot must be moving before the action
     * is considered done
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit StopAction(
        double stopped_speed_threshold = ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT,
        bool loop_forever              = false);

    /**
     * Returns the next Intent this StopAction wants to run, given the parameters.
     *
     * @param robot the robot that should stop
     * @param coast Whether or not to coast to a stop. If this is false, the robot will
     * try actively brake to come to a stop
     *
     * @return A unique pointer to the Intent the StopAction wants to run.
     *
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot, bool coast);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    // Whether or not the robot should coast to a stop
    bool coast;
    // The maximum speed the robot may be moving at to be considered stopped
    double stopped_speed_threshold;
    bool loop_forever;
};
