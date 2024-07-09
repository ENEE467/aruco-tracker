#include <gtest/gtest.h>

#include "options.hpp"

// Round track error calculation tests -------------------------------------------------------------

/**
 * Test 1:
 *   Line follower is outside the round track in the first quadrant
 *   Center of the track is away from the origin (in first quadrant)
 */
TEST(RoundTrackErrorCalculation, LineFollowerOutsideQ1)
{
  options::RoundTrack testTrack {5, 5, 10*2, 7*2};
  cv::Point2d testLineFollowerPosition {10, 15};

  auto errorTest {testTrack.calculatePerpendicularDistance(testLineFollowerPosition)};

  EXPECT_NEAR(3.72, errorTest, 0.01);
}

/**
 * Test 2:
 *   Line follower is outside the round track in third quadrant
 *   Center of the track is away from the origin (in second quadrant)
 */
TEST(RoundTrackErrorCalculation, LineFollowerOutsideQ3)
{
  options::RoundTrack testTrack {-9.9, 9.5, 13*2, 3.9*2};
  cv::Point2d testLineFollowerPosition {-15, -2};

  auto errorTest {testTrack.calculatePerpendicularDistance(testLineFollowerPosition)};

  EXPECT_NEAR(7.85, errorTest, 0.01);
}

/**
 * Test 3:
 *  Line follower is inside the round track in the first quadrant
 *  Center of the track is away from the origin (in first quadrant)
 */
TEST(RoundTrackErrorCalculation, LineFollowerInsideQ1)
{
 options::RoundTrack testTrack {16.2, 10.1, 9.4*2, 4.5*2};
 cv::Point2d testLineFollowerPosition {23.5, 8.8};

 auto errorTest {testTrack.calculatePerpendicularDistance(testLineFollowerPosition)};

 EXPECT_NEAR(1.24, errorTest, 0.01);
}

/**
 * Test 4:
 *   Line follower is inside the round track in the third quadrant
 *   Center of the track is away from the origin (in first quadrant)
 */
TEST(RoundTrackErrorCalculation, LineFollowerInsideQ3)
{
  options::RoundTrack testTrack {-9.9, -9.3, 8.5*2, 6.7*2};
  cv::Point2d testLineFollowerPosition {-16, -8};

  auto errorTest {testTrack.calculatePerpendicularDistance(testLineFollowerPosition)};

  EXPECT_NEAR(2.12, errorTest, 0.01);
}
