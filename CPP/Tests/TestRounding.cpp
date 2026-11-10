#include <gtest/gtest.h>
#include "clipper2/clipper.core.h"
#include <cfenv>

using namespace Clipper2Lib;

TEST(Clipper2Tests, TestRoundToInt64)
{
  // volatile keeps the compiler from folding the rounding in the default mode
  static volatile double values[] = { 0.0, -0.0, 0.5, -0.5, 1.5, -1.5, 2.5, -2.5, 2.4, -2.6,
    0.49999999999999994, -0.49999999999999994, 4503599627370495.5, -4503599627370495.5,
    9007199254740992.0, -9007199254740994.0, 1e18 + 0.0, max_coord, min_coord };

  EXPECT_EQ(RoundToInt64(values[2]), 0);
  EXPECT_EQ(RoundToInt64(values[3]), 0);
  EXPECT_EQ(RoundToInt64(values[4]), 2);
  EXPECT_EQ(RoundToInt64(values[6]), 2);
  EXPECT_EQ(RoundToInt64(values[7]), -2);
  EXPECT_EQ(RoundToInt64(values[12]), 4503599627370496);
  EXPECT_EQ(RoundToInt64(values[13]), -4503599627370496);
  EXPECT_EQ(RoundToInt64(values[17]), static_cast<int64_t>(max_coord));

  const int modes[] = { FE_TONEAREST,
#ifdef FE_DOWNWARD
    FE_DOWNWARD,
#endif
#ifdef FE_UPWARD
    FE_UPWARD,
#endif
#ifdef FE_TOWARDZERO
    FE_TOWARDZERO,
#endif
  };
  const int saved = std::fegetround();

  for (int mode : modes)
  {
    EXPECT_EQ(std::fesetround(mode), 0);

    for (const volatile double& v : values)
    {
      const double val = v;
      EXPECT_EQ(RoundToInt64(val), static_cast<int64_t>(std::nearbyint(val))) << "mode " << mode << " value " << val;
    }
  }

  std::fesetround(saved);
}
