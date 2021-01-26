// Bring in package's API
#include <mslquad/px4_base_controller.h>
 // Bring in gtest
#include <gtest/gtest.h>
 
// Declare a test
TEST(TestSuite, testHello){
  EXPECT_EQ("hello", "hello");
  }

// Declare another test
TEST(TestSuite, testSucceed){
  SUCCEED();
  }

TEST(TestSuite, testAssert){
  int n = 1;
  ASSERT_EQ(n, 1);
  }
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  }