#include <gtest/gtest.h>

// Main collecting all defined unit-tests.
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
