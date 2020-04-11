#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_without_goalie)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                             Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              0.0),
        0.05));
}

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_with_goalie_left_side)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                             Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_goalie =
        Robot(1, world.field().friendlyGoal(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    // The robot's position should be one full robot diameter to the left,
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              2 * ROBOT_MAX_RADIUS_METERS),
        0.05));
}

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_with_goalie_right_side)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                             Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_goalie =
        Robot(1, world.field().friendlyGoal(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::RIGHT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    // The robot's position should be one full robot diameter to the right,
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              -2 * ROBOT_MAX_RADIUS_METERS),
        0.05));
}

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_path_crossing_right_defense_area_segment)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setBallPosition(world, world.field().friendlyCornerNeg(), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 2.09)},
                                             Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_goalie =
            Robot(1, world.field().friendlyGoal(), Vector(0, 0), Angle::zero(),
                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic =
            CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                                 world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);


    // The robot's position should be one robot radius to the Right of the goalie
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    std::cout<<"destination: "<<move_action->getDestination()<<std::endl;
    std::cout<<"expected destination: "<<Point(world.field().friendlyDefenseArea().negXNegYCorner().x() +
                                               ROBOT_MAX_RADIUS_METERS,
                                               world.field().friendlyDefenseArea().negXNegYCorner().y() - 1.5 * ROBOT_MAX_RADIUS_METERS)<<std::endl;

    EXPECT_TRUE(move_action->getDestination().isClose(
            Point(world.field().friendlyDefenseArea().negXNegYCorner().x() +
            ROBOT_MAX_RADIUS_METERS,
                  world.field().friendlyDefenseArea().negXNegYCorner().y() - 1.5 * ROBOT_MAX_RADIUS_METERS),
            0.05));
}


TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_path_crossing_left_defense_area_segment)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setBallPosition(world, world.field().friendlyCornerPos(), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 2.09)},
                                                     Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_goalie =
            Robot(1, world.field().friendlyGoal(), Vector(0, 0), Angle::zero(),
                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic =
            CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                                 world.enemyTeam(), CreaseDefenderTactic::RIGHT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);


    // The robot's position should be one robot radius to the Right of the goalie
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    std::cout<<"destination: "<<move_action->getDestination()<<std::endl;
    std::cout<<"expected destination: "<<Point(world.field().friendlyDefenseArea().negXPosYCorner().x() +
                                               ROBOT_MAX_RADIUS_METERS,
                                               world.field().friendlyDefenseArea().negXPosYCorner().y() + 1.5 * ROBOT_MAX_RADIUS_METERS)<<std::endl;

    EXPECT_TRUE(move_action->getDestination().isClose(
            Point(world.field().friendlyDefenseArea().negXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
                  world.field().friendlyDefenseArea().negXPosYCorner().y() + 1.5 * ROBOT_MAX_RADIUS_METERS),
            0.05));
}

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_goalie_out_of_position)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setBallPosition(world, Point(0,0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 2.09)},
                                                     Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_goalie =
            Robot(1, world.field().friendlyCornerPos() , Vector(0, 0), Angle::zero(),
                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic =
            CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                                 world.enemyTeam(), CreaseDefenderTactic::RIGHT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);


    // The robot's position should be one robot radius to the Right of the goalie
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    std::cout<<"destination: "<<move_action->getDestination()<<std::endl;
    std::cout<<"expected destination: "<<Point(world.field().friendlyDefenseArea().negXPosYCorner().x() +
                                               ROBOT_MAX_RADIUS_METERS,
                                               world.field().friendlyDefenseArea().negXPosYCorner().y() + 1.5 * ROBOT_MAX_RADIUS_METERS)<<std::endl;

    EXPECT_TRUE(move_action->getDestination().isClose(
            Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
                  0.0),
            0.05));
}