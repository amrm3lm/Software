/**
 * Tests for the Reciever Tactic
 */
#include "ai/hl/stp/tactic/receiver_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/kick_intent.h"
#include "ai/intent/move_intent.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "test/test_util/test_util.h"

using namespace Passing;

TEST(ReceiverTacticTest, robot_not_at_receive_position_pass_not_started)
{
    Robot receiver = Robot(13, Point(1, -3), Vector(), Angle::zero(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Pass pass({1, 1}, {0.5, 0}, 2.29, Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1, 1}, {0, 0}, Timestamp::fromSeconds(0));

    Field field = ::Test::TestUtil::createSSLDivBField();
    ReceiverTactic tactic(field, friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    Angle shot_dir = (field.enemyGoal() - receiver.position()).orientation();

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_DOUBLE_EQ(0.5, move_intent.getDestination().x());
    EXPECT_DOUBLE_EQ(0.0, move_intent.getDestination().y());
    EXPECT_EQ((pass.receiverOrientation() + shot_dir) / 2, move_intent.getFinalAngle());
    EXPECT_FALSE(move_intent.isDribblerEnabled());
    EXPECT_EQ(move_intent.getAutoKickType(), NONE);
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_not_started)
{
    Pass pass({1, 1}, {0.5, 0}, 2.29, Timestamp::fromSeconds(5));

    Robot receiver = Robot(13, Point(0.5, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1, 1}, {0, 0}, Timestamp::fromSeconds(0));

    Field field = ::Test::TestUtil::createSSLDivBField();
    ReceiverTactic tactic(field, friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since we're already setup to receive the pass, we should just be trying to move
    // to our current position. We should continue to yield new Move Intents even though
    // we're at the target position
    for (int i = 0; i < 5; i++)
    {
        tactic.updateParams(friendly_team, enemy_team, pass, ball);

        Angle shot_dir = (field.enemyGoal() - receiver.position()).orientation();

        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
        EXPECT_EQ(13, move_intent.getRobotId());
        EXPECT_DOUBLE_EQ(0.5, move_intent.getDestination().x());
        EXPECT_DOUBLE_EQ(0.0, move_intent.getDestination().y());
        EXPECT_EQ((pass.receiverOrientation() + shot_dir) / 2,
                  move_intent.getFinalAngle());
        EXPECT_FALSE(move_intent.isDribblerEnabled());
        EXPECT_EQ(move_intent.getAutoKickType(), NONE);
    }
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_started_goal_open_angle_feasible)
{
    // Test case where we can feasibly one-touch the ball into the net

    Pass pass({1, 1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1, -3}, {-1, 3}, Timestamp::fromSeconds(5));

    Field field = ::Test::TestUtil::createSSLDivBField();
    ReceiverTactic tactic(field, friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // We should be trying to move into a position to properly deflect the ball into
    // the net with a kick
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_LT(move_intent.getDestination().x(), -0.001);
    EXPECT_GT(move_intent.getDestination().x(), -0.2);

    EXPECT_GT(move_intent.getDestination().y(), 0.001);
    EXPECT_LT(move_intent.getDestination().y(), 0.1);

    EXPECT_LT(move_intent.getFinalAngle().toDegrees(), -1);
    EXPECT_GT(move_intent.getFinalAngle().toDegrees(), -90);

    EXPECT_FALSE(move_intent.isDribblerEnabled());
    EXPECT_EQ(move_intent.getAutoKickType(), AUTOKICK);
}

TEST(ReceiverTacticTest,
     robot_at_receive_position_pass_started_goal_open_angle_not_feasible)
{
    // Test case where we can't feasibly one-touch the ball into the net because the
    // deflection angle between the pass and shot would be way too large

    Pass pass({-1, -1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({-1, -1}, {1, 1}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team,
                          enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since there's no reasonable way we could one-touch kick the pass into the net,
    // we should be lining up to receive it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0.0, move_intent.getDestination().x(), 0.0001);
    EXPECT_NEAR(0.0, move_intent.getDestination().y(), 0.0001);
    EXPECT_EQ(pass.receiverOrientation(), move_intent.getFinalAngle());

    EXPECT_TRUE(move_intent.isDribblerEnabled());
    EXPECT_EQ(move_intent.getAutoKickType(), NONE);
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_started_goal_blocked)
{
    // Test case where we are facing right towards the goal, but it is blocked

    Pass pass({0.5, 0.5}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {1, -0.2}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(1, {1, -0.1}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(2, {1, 0.0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(3, {1, 0.1}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(4, {1, 0.2}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});

    Ball ball({0.5, 0.5}, {-0.5, -0.5}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team,
                          enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since there's no reasonable way we could one-touch kick the pass into the net,
    // we should be lining up to receive it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0.0, move_intent.getDestination().x(), 0.0001);
    EXPECT_NEAR(0.0, move_intent.getDestination().y(), 0.0001);
    EXPECT_EQ(pass.receiverOrientation(), move_intent.getFinalAngle());

    EXPECT_TRUE(move_intent.isDribblerEnabled());
    EXPECT_EQ(move_intent.getAutoKickType(), NONE);
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_received)
{
    // Test where we have received the pass and the ball is now positioned in our dribbler

    Pass pass({-1, 1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    // Ball is travelling towards the robot
    Ball ball({-0.5, 0.5}, {-1, 1}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team,
                          enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // We should yield one intent here
    EXPECT_TRUE(tactic.getNextIntent());

    // Position the ball just in front of the robot dribbler
    Point ball_pos = receiver.position() +
                     Vector(receiver.orientation().cos(), receiver.orientation().sin())
                         .norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);
    ball = Ball(ball_pos, {-1, 1}, Timestamp::fromSeconds(5));
    tactic.updateParams(friendly_team, enemy_team, pass, ball);

    // Since we've received the ball, we shouldn't yield anything
    EXPECT_FALSE(tactic.getNextIntent());
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_one_touch_kicked)
{
    // Test where we have just one-touch kicked the pass towards the goal

    Pass pass({-1, 1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    // The ball is travelling away from the origin towards the enemy net
    Ball ball({1, 0}, {4, 0}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team,
                          enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since we've kicked the ball, we shouldn't yield anything
    EXPECT_EQ(std::unique_ptr<Intent>{}, tactic.getNextIntent());
}

class OneTimeShotDirectionTest
    : public ::testing::TestWithParam<std::tuple<Point, Point, double, double>>
{
};
TEST_P(OneTimeShotDirectionTest, test_shot_towards_enemy_net)
{
    Point robot_location     = std::get<0>(GetParam());
    Point ball_location      = std::get<1>(GetParam());
    double min_angle_degrees = std::get<2>(GetParam());
    double max_angle_degrees = std::get<3>(GetParam());

    // Create a ball traveling from the specified position towards the robot
    Ball ball(ball_location, robot_location - ball_location, Timestamp::fromSeconds(0));

    // Create a shot towards the enemy net
    Ray shot(robot_location, Vector(4.5, 0) - robot_location);

    Angle robot_angle = ReceiverTactic::getOneTimeShotDirection(shot, ball);

    EXPECT_GT(robot_angle.toDegrees(), min_angle_degrees);
    EXPECT_LT(robot_angle.toDegrees(), max_angle_degrees);
}
// Since the exact direction for one time shots is highly variable and depends a lot on
// physical tests, we check the exact angles, but we can at least test that they're in
// the right range
INSTANTIATE_TEST_CASE_P(
    All, OneTimeShotDirectionTest,
    ::testing::Values(
        // Robot at the origin, ball coming at it from different directions
        std::make_tuple<Point, Point, double, double>({0, 0}, {1, 1}, 1, 20),
        std::make_tuple<Point, Point, double, double>({0, 0}, {3, 1}, 1, 20),
        std::make_tuple<Point, Point, double, double>({0, 0}, {3, -1}, -20, -1),
        std::make_tuple<Point, Point, double, double>({0, 0}, {1, -1}, -20, -1),
        std::make_tuple<Point, Point, double, double>({0, 0}, {0, 1}, 1, 40),
        std::make_tuple<Point, Point, double, double>({0, 0}, {0, -1}, -40, -1),
        // Corner kicks, robot is roughly in the opposite corner of the goal crease to
        // where the corner kick is coming from
        std::make_tuple<Point, Point, double, double>({3, 1}, {4.5, -3}, -45, -5),
        std::make_tuple<Point, Point, double, double>({3, -1}, {4.5, 3}, 5, 45),
        // Corner kicks, robot is roughly in the same corner of the goal crease to
        // where the corner kick is coming from
        std::make_tuple<Point, Point, double, double>({3, -1}, {4.5, -3}, 0, 45),
        std::make_tuple<Point, Point, double, double>({3, 1}, {4.5, 3}, -45, 0),
        // Corner kick, robot is close to the goal and directly in front of it
        std::make_tuple<Point, Point, double, double>({4, 0}, {4.5, -3}, -45, -1)));

class OneTimeShotPositionTest
    : public ::testing::TestWithParam<std::tuple<double, double, double, double, double>>
{
};
/**
 * This test was added because during field testing we observed that in rare cases, the
 * receiver robot would move away from the ball's trajectory rather than towards it, and
 * therefore completely miss the pass. The behaviour did not seem to be caused by bad
 * vision data, and seemed more like a logic issue, so this test was added to try catch
 * any strange edge cases in the logic that would cause the robot to move to the wrong
 * position
 */
TEST_P(OneTimeShotPositionTest, test_receiver_moves_to_correct_one_time_shot_position)
{
    Point robot_position(std::get<0>(GetParam()), std::get<1>(GetParam()));
    Point ball_position(std::get<2>(GetParam()), std::get<3>(GetParam()));
    // We just choose a moderate speed for these tests. Varying the speed won't change
    // the results since it's treated as an "ideal ball trajectory" anyway
    double ball_speed                 = 4;
    Angle ball_velocity_vector_offset = Angle::ofDegrees(std::get<4>(GetParam()));
    // We apply angular "noise" to the ball velocity vector to simulate imperfect passes
    Vector ball_velocity_vector = Vector::createFromAngle(
        (robot_position - ball_position).orientation() + ball_velocity_vector_offset);

    // Create a ball traveling from the specified position towards the robot
    Ball ball(ball_position, ball_velocity_vector.norm(ball_speed),
              Timestamp::fromSeconds(0));

    // Create a robot at the robot location with no velocity. The initial orientation
    // does not matter.
    Robot robot(0, robot_position, Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // Create a best shot towards the center of the enemy goal
    Point best_shot_target = Point(4.5, 0);

    auto [ideal_position, ideal_orientation] =
        ReceiverTactic::getOneTimeShotPositionAndOrientation(robot, ball,
                                                             best_shot_target);

    // The position where the ball should make contact with the receiver robot
    Point ball_contact_position =
        ideal_position +
        Point::createFromAngle(ideal_orientation)
            .norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);

    // We check that the position the receiver tries to move to will cause the
    // ball contact point to intersect with the ball's trajectory, meaning that we are
    // in the correct position to make contact with the ball
    Line ball_path = Line(ball.position(), ball.position() + ball.velocity());
    double dist_to_ball_passthrough = dist(ball_path, ball_contact_position);

    EXPECT_TRUE(dist_to_ball_passthrough < 0.001);
}

INSTANTIATE_TEST_CASE_P(
    All, OneTimeShotPositionTest,
    ::testing::Combine(testing::Values(-0.2),           // Robot x coordinate
                       testing::Values(0.0),            // Robot y coordinate
                       testing::Range(-3.0, 3.0, 0.5),  // Ball x coordinate
                       testing::Range(-4.5, 0.0, 0.5),  // Ball y coordinate
                       testing::Range(-5.0, 5.0, 2.5)  // Angle deviation from ideal pass,
                                                       // in degrees
                       ));
