#include <gtest/gtest.h>
#include "core/version.hpp"

TEST(Version, IsNotEmpty) {
  EXPECT_STRNE(core::version(), "");
}
