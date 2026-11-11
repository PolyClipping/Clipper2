#include <gtest/gtest.h>
#include "clipper2/clipper.offset.h"
#include "ClipFileLoad.h"
TEST(Clipper2Tests, TestOffsettingOrientation1) {
  const Clipper2Lib::Paths64 subject = { Clipper2Lib::MakePath({ 0,0, 0,5, 5,5, 5,0 }) };
  Clipper2Lib::Paths64 solution = Clipper2Lib::InflatePaths(subject, 1,
    Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  ASSERT_EQ(solution.size(), 1);
  //when offsetting, output orientation is positive whatever the input's orientation
  EXPECT_FALSE(Clipper2Lib::IsPositive(subject[0]));
  EXPECT_TRUE(Clipper2Lib::IsPositive(solution[0]));
}
TEST(Clipper2Tests, TestOffsettingOrientation2) {
  const Clipper2Lib::Paths64 subject = {
    Clipper2Lib::MakePath({20, 220, 280, 220, 280, 280, 20, 280}),
    Clipper2Lib::MakePath({0, 200, 0, 300, 300, 300, 300, 200})
  };
  Clipper2Lib::ClipperOffset co;
  co.ReverseSolution(true); // could also assign using a parameter in ClipperOffset's constructor
  co.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  Clipper2Lib::Paths64 solution;
  co.Execute(5, solution);
  ASSERT_EQ(solution.size(), 2);
  // When offsetting, output orientation is positive EXCEPT when ReverseSolution == true
  // However, input path ORDER may not match output path order. For example, order will change
  // whenever inner paths (holes) are defined before their container outer paths (as above).
  // And when offsetting multiple outer paths, their order will likely change too. Due to the
  // sweep-line algorithm used, paths with larger Y coordinates will likely be listed first.
  EXPECT_FALSE(Clipper2Lib::IsPositive(solution[0]));
}

TEST(Clipper2Tests, TestOffsettingOrientation3) { // PolyClipping/Clipper2#13
  // a stray negative sliver lower than the outer path must not reverse the group
  const Clipper2Lib::Paths64 subject = {
    Clipper2Lib::MakePath({ 33621,18600, 33889,18931, 33622,18601, 33118,18033 }),
    Clipper2Lib::MakePath({ 33058,3177, 33032,3337, 33017,3570, 33040,17609, 33118,18033, 32723,17588, 32698,3608,
      32710,3376, 32737,3177 }) };
  Clipper2Lib::Paths64 solution = Clipper2Lib::InflatePaths(subject, -8,
    Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  ASSERT_EQ(solution.size(), 1);
  // the sliver is a hole, so the result is the outer path's own inward offset
  EXPECT_EQ(Clipper2Lib::Area(solution), Clipper2Lib::Area(Clipper2Lib::InflatePaths({ subject[1] }, -8,
    Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon)));
}

TEST(Clipper2Tests, TestOffsettingOrientation4) {
  // a group holding only holes takes its orientation from the outer in another group
  Clipper2Lib::Path64 outer = Clipper2Lib::MakePath({ 0,0, 300,0, 300,300, 0,300 });
  Clipper2Lib::Path64 hole = Clipper2Lib::MakePath({ 100,100, 100,200, 200,200, 200,100 });

  for (int mirror = 0; mirror < 2; ++mirror)
  {
    for (bool outer_first : { true, false })
    {
      Clipper2Lib::ClipperOffset co;
      co.AddPath(outer_first ? outer : hole, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
      co.AddPath(outer_first ? hole : outer, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
      Clipper2Lib::Paths64 solution;
      co.Execute(10, solution);
      EXPECT_EQ(Clipper2Lib::Area(solution), 320 * 320 - 80 * 80);
    }

    std::reverse(outer.begin(), outer.end());
    std::reverse(hole.begin(), hole.end());
  }
}

static Clipper2Lib::Path64 MakeSquare(int64_t left, int64_t top, int64_t size, bool clockwise)
{
  Clipper2Lib::Path64 result = Clipper2Lib::MakePath({ left,top, left + size,top, left + size,top + size,
    left,top + size });

  if (clockwise) std::reverse(result.begin(), result.end());

  return result;
}

static std::vector<double> OffsetAreas(const Clipper2Lib::Paths64& groups, double delta)
{
  Clipper2Lib::ClipperOffset co;

  for (const Clipper2Lib::Path64& path : groups)
    co.AddPath(path, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);

  Clipper2Lib::Paths64 solution;
  co.Execute(delta, solution);
  std::vector<double> result;

  for (const Clipper2Lib::Path64& path : solution)
    result.push_back(Clipper2Lib::Area(path));

  std::sort(result.begin(), result.end());
  return result;
}

static std::vector<double> OneGroupAreas(const Clipper2Lib::Paths64& paths, double delta)
{
  std::vector<double> result;

  for (const Clipper2Lib::Path64& path : Clipper2Lib::InflatePaths(paths, delta, Clipper2Lib::JoinType::Round,
    Clipper2Lib::EndType::Polygon))
    result.push_back(Clipper2Lib::Area(path));

  std::sort(result.begin(), result.end());
  return result;
}

TEST(Clipper2Tests, TestOffsettingOrientation5) {
  // a group wound against another is its own outer, unless it lies inside that group's outer
  const Clipper2Lib::Path64 outer = MakeSquare(0, 0, 1000, false);
  const Clipper2Lib::Path64 disjoint = MakeSquare(2000, 0, 100, true);
  const Clipper2Lib::Path64 hole = MakeSquare(400, 400, 200, true);

  for (bool outer_first : { true, false })
  {
    auto groups = [&](const Clipper2Lib::Path64& other) {
      return outer_first ? Clipper2Lib::Paths64{ outer, other } : Clipper2Lib::Paths64{ other, outer };
    };

    EXPECT_EQ(OffsetAreas(groups(disjoint), 10), std::vector<double>({ 14308, 1040308 }));
    EXPECT_EQ(OffsetAreas(groups(disjoint), -10), std::vector<double>({ 6400, 960400 }));
    EXPECT_EQ(OffsetAreas(groups(hole), 10), std::vector<double>({ -32400, 1040308 }));
    EXPECT_EQ(OffsetAreas(groups(hole), -10), std::vector<double>({ -48308, 960400 }));
  }
}

TEST(Clipper2Tests, TestOffsettingOrientation6) {
  // the innermost enclosing path decides, so a group inside another group's hole is an island
  const Clipper2Lib::Paths64 chain = { MakeSquare(0, 0, 1000, false), MakeSquare(200, 200, 500, true),
    MakeSquare(300, 300, 100, false) };
  std::vector<size_t> order = { 0, 1, 2 };

  do
  {
    const Clipper2Lib::Paths64 groups = { chain[order[0]], chain[order[1]], chain[order[2]] };
    EXPECT_EQ(OffsetAreas(groups, 10), std::vector<double>({ -230400, 14308, 1040308 }));
  } while (std::next_permutation(order.begin(), order.end()));
}

TEST(Clipper2Tests, TestOffsettingOrientation7) {
  // separate groups behave like one group when their paths' roles are unambiguous
  const Clipper2Lib::Path64 outer = MakeSquare(0, 0, 1000, false);
  const Clipper2Lib::Path64 ring_hole = MakeSquare(500, 500, 400, true);
  const Clipper2Lib::Path64 touching = MakeSquare(0, 400, 200, true); // first vertex on the outer's edge
  const Clipper2Lib::Path64 inscribed = Clipper2Lib::MakePath({ 0,500, 500,1000, 1000,500, 500,0 });
  const Clipper2Lib::Path64 island = MakeSquare(600, 600, 100, false);
  const Clipper2Lib::Path64 hole = MakeSquare(100, 100, 100, true);

  EXPECT_EQ(OffsetAreas({ touching, outer }, 10), OneGroupAreas({ outer, touching }, 10));
  EXPECT_EQ(OffsetAreas({ inscribed, outer }, 10), OneGroupAreas({ outer, inscribed }, 10));

  // a group of an outer and its hole encloses an island in the hole and a hole in the ring
  Clipper2Lib::ClipperOffset co;
  co.AddPath(island, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  co.AddPath(hole, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  co.AddPaths({ outer, ring_hole }, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  Clipper2Lib::Paths64 solution;
  co.Execute(10, solution);
  std::vector<double> areas;

  for (const Clipper2Lib::Path64& path : solution)
    areas.push_back(Clipper2Lib::Area(path));

  std::sort(areas.begin(), areas.end());
  EXPECT_EQ(areas, OneGroupAreas({ outer, ring_hole, island, hole }, 10));

  // the hole lies in an outer and in another group's hole of the same area, and the outer decides
  const Clipper2Lib::Path64 frame = MakeSquare(-500, -500, 2000, false);
  const Clipper2Lib::Path64 frame_hole = MakeSquare(0, 0, 1000, true);

  for (bool outer_first : { true, false })
  {
    Clipper2Lib::ClipperOffset tie;
    tie.AddPath(hole, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);

    if (outer_first) tie.AddPath(outer, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);

    tie.AddPaths({ frame, frame_hole }, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);

    if (!outer_first) tie.AddPath(outer, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);

    tie.Execute(10, solution);
    areas.clear();

    for (const Clipper2Lib::Path64& path : solution)
      areas.push_back(Clipper2Lib::Area(path));

    std::sort(areas.begin(), areas.end());
    EXPECT_EQ(areas, OneGroupAreas({ frame, frame_hole, outer, hole }, 10));
  }
}

TEST(Clipper2Tests, TestOffsettingOrientation8) {
  // a container with many collinear vertices is searched through an index of its edges
  const Clipper2Lib::Path64 corners = Clipper2Lib::MakePath({ 0,0, 1000,0, 1000,400, 400,400, 400,1000, 0,1000 });
  Clipper2Lib::Path64 l_shape;

  for (size_t i = 0; i < corners.size(); ++i)
  {
    const Clipper2Lib::Point64& a = corners[i];
    const Clipper2Lib::Point64& b = corners[(i + 1) % corners.size()];
    const int64_t steps = (std::abs(b.x - a.x) + std::abs(b.y - a.y)) / 50;

    for (int64_t k = 0; k < steps; ++k)
      l_shape.emplace_back(a.x + (b.x - a.x) * k / steps, a.y + (b.y - a.y) * k / steps);
  }

  ASSERT_GE(l_shape.size(), 64);
  const Clipper2Lib::Path64 in_arm = MakeSquare(100, 100, 100, true);
  const Clipper2Lib::Path64 touching = MakeSquare(0, 500, 100, true); // first vertex on an edge's vertex
  const Clipper2Lib::Path64 in_notch = MakeSquare(600, 600, 100, true); // inside the bounds only
  Clipper2Lib::Path64 notch_outer = in_notch;
  std::reverse(notch_outer.begin(), notch_outer.end());

  for (double delta : { 10.0, -10.0 })
  {
    EXPECT_EQ(OffsetAreas({ in_notch, in_arm, touching, l_shape }, delta),
      OneGroupAreas({ l_shape, in_arm, touching, notch_outer }, delta));
  }
}

TEST(Clipper2Tests, TestOffsettingStrayHoles) {
  // a hole that no outer encloses contributes nothing, not even the loops of its concave joins
  const Clipper2Lib::Path64 outer = MakeSquare(0, 0, 100, false);
  const Clipper2Lib::Path64 stray = MakeSquare(500, 500, 50, true);
  const Clipper2Lib::Path64 cw_outer = MakeSquare(0, 0, 100, true);
  const Clipper2Lib::Path64 ccw_stray = MakeSquare(500, 500, 50, false);

  for (Clipper2Lib::JoinType join_type : { Clipper2Lib::JoinType::Miter, Clipper2Lib::JoinType::Round })
  {
    for (double delta : { 10.0, -10.0 })
    {
      EXPECT_EQ(Clipper2Lib::InflatePaths({ outer, stray }, delta, join_type, Clipper2Lib::EndType::Polygon),
        Clipper2Lib::InflatePaths({ outer }, delta, join_type, Clipper2Lib::EndType::Polygon));
      EXPECT_EQ(Clipper2Lib::InflatePaths({ cw_outer, ccw_stray }, delta, join_type, Clipper2Lib::EndType::Polygon),
        Clipper2Lib::InflatePaths({ cw_outer }, delta, join_type, Clipper2Lib::EndType::Polygon));
    }
  }

  // a hole around another group's island is still stray, since none of its vertices is inside an outer
  const Clipper2Lib::Path64 far_outer = MakeSquare(0, 0, 1000, false);
  const Clipper2Lib::Path64 around = MakeSquare(2000, 0, 300, true);
  const Clipper2Lib::Path64 island = MakeSquare(2100, 100, 50, false);
  Clipper2Lib::ClipperOffset co;
  co.AddPath(island, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  co.AddPaths({ far_outer, around }, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  Clipper2Lib::Paths64 solution;
  co.Execute(10, solution);
  EXPECT_EQ(solution, Clipper2Lib::InflatePaths({ far_outer, island }, 10, Clipper2Lib::JoinType::Round,
    Clipper2Lib::EndType::Polygon));

  // a hole poking out of its outer still cuts it
  const Clipper2Lib::Path64 poking = MakeSquare(80, 40, 40, true);
  EXPECT_LT(Clipper2Lib::Area(Clipper2Lib::InflatePaths({ outer, poking }, 10, Clipper2Lib::JoinType::Miter,
    Clipper2Lib::EndType::Polygon)), 120 * 120);
}

static Clipper2Lib::Paths64 OffsetGroups(const std::vector<Clipper2Lib::Paths64>& groups, double delta)
{
  Clipper2Lib::ClipperOffset co;

  for (const Clipper2Lib::Paths64& group : groups)
    co.AddPaths(group, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);

  Clipper2Lib::Paths64 solution;
  co.Execute(delta, solution);
  return solution;
}

TEST(Clipper2Tests, TestOffsettingStrayHoleDoesNotOrient) {
  // the stray strip crosses the middle square without enclosing any of its vertices, so it must not
  // decide that the small square inside it is an island rather than the middle square's hole
  Clipper2Lib::Path64 far_outer = MakeSquare(5000, 5000, 10000, false);
  Clipper2Lib::Path64 strip = Clipper2Lib::MakePath({ -100,600, 1100,600, 1100,400, -100,400 });
  Clipper2Lib::Path64 middle = MakeSquare(0, 0, 1000, false);
  Clipper2Lib::Path64 hole = MakeSquare(450, 450, 100, true);

  for (int mirror = 0; mirror < 2; ++mirror)
  {
    for (double delta : { 0.0, 10.0, -10.0 })
    {
      EXPECT_EQ(OffsetGroups({ { far_outer, strip }, { middle }, { hole } }, delta),
        OffsetGroups({ { far_outer }, { middle }, { hole } }, delta));
    }

    for (Clipper2Lib::Path64* path : { &far_outer, &strip, &middle, &hole })
      std::reverse(path->begin(), path->end());
  }
}

TEST(Clipper2Tests, TestOffsettingStrayHoleChain) {
  // each strip is stray only once the hole beside the previous strip is known to be a hole, so the
  // whole chain must settle, however long
  for (bool mirror : { false, true })
  {
    std::vector<Clipper2Lib::Paths64> with_strips, without_strips;
    const Clipper2Lib::Path64 far_outer = MakeSquare(-300000000, -50000000, 100000000, mirror);
    with_strips.push_back({ far_outer });
    without_strips.push_back({ far_outer });
    size_t strip_group = 0;
    int64_t size = 10000000;

    for (int64_t i = 1; i <= 6; ++i, size /= 10)
    {
      const int64_t x = i * 300000000;
      Clipper2Lib::Path64 strip = Clipper2Lib::MakePath({ x - size * 6 / 10,-size / 10, x + size * 6 / 10,-size / 10,
        x + size * 6 / 10,size / 10, x - size * 6 / 10,size / 10 });

      if (!mirror) std::reverse(strip.begin(), strip.end());

      const Clipper2Lib::Path64 small = MakeSquare(x - size / 20, -size / 20, size / 10, !mirror);
      const Clipper2Lib::Path64 middle = MakeSquare(x - size / 2, -size / 2, size, mirror);
      // the strip sits in the group of the previous level's small square
      with_strips[strip_group].push_back(strip);
      strip_group = with_strips.size();
      with_strips.push_back({ small });
      with_strips.push_back({ middle });
      without_strips.push_back({ small });
      without_strips.push_back({ middle });
    }

    for (double delta : { 0.0, 10.0, -10.0 })
      EXPECT_EQ(OffsetGroups(with_strips, delta), OffsetGroups(without_strips, delta));
  }
}

TEST(Clipper2Tests, TestOffsettingHoleSizesCompareExactly) {
  // Area() rounds both of these, and a hole compared as larger than its outer was dropped as stray
  const Clipper2Lib::Path64 far_outer = MakeSquare(-300000000, -300000000, 200000000, false);
  const Clipper2Lib::Path64 outer = Clipper2Lib::MakePath({ 7786030,124989629, 13919251,42104456,
    167208900,68570499, 199121929,96243643, 185340413,198615010, 66854563,174207781 });
  Clipper2Lib::Path64 coincident = outer;
  std::reverse(coincident.begin(), coincident.end());

  // a thin outer and a thin hole strictly inside it, whose areas are exactly 36 and 16
  const Clipper2Lib::Path64 thin_outer = Clipper2Lib::MakePath({ -310414360,-251832566, 13677104,72258904,
    337768562,396350368, 13677098,72258898 });
  const Clipper2Lib::Path64 thin_hole = Clipper2Lib::MakePath({ 13677099,72258899, 229738075,288319879,
    13677103,72258903, -202383873,-143802077 });
  const Clipper2Lib::Path64 small_outer = MakeSquare(-1000000, -1000000, 100000, false);

  for (double delta : { 0.0, 10.0, -10.0 })
  {
    EXPECT_EQ(OffsetGroups({ { far_outer, outer, coincident } }, delta),
      OffsetGroups({ { far_outer }, { outer, coincident } }, delta));
    EXPECT_EQ(OffsetGroups({ { small_outer, thin_outer, thin_hole } }, delta),
      OffsetGroups({ { small_outer }, { thin_outer, thin_hole } }, delta));
  }
}

TEST(Clipper2Tests, TestOffsettingZeroDeltaNormalizes) {
  // a zero delta is insignificant like any other below half a unit, so it normalizes the winding too
  const Clipper2Lib::Path64 square = MakeSquare(0, 0, 100, true);
  const Clipper2Lib::Paths64 solution = Clipper2Lib::InflatePaths({ square }, 0, Clipper2Lib::JoinType::Miter,
    Clipper2Lib::EndType::Polygon);
  EXPECT_EQ(solution, Clipper2Lib::InflatePaths({ square }, 0.3, Clipper2Lib::JoinType::Miter,
    Clipper2Lib::EndType::Polygon));
  EXPECT_EQ(Clipper2Lib::Area(solution), 100 * 100);

  const Clipper2Lib::PathsD square_d = { Clipper2Lib::MakePathD({ 0,0, 0,1, 1,1, 1,0 }) };
  const Clipper2Lib::PathsD solution_d = Clipper2Lib::InflatePaths(square_d, 0, Clipper2Lib::JoinType::Miter,
    Clipper2Lib::EndType::Polygon);
  EXPECT_EQ(solution_d, Clipper2Lib::InflatePaths(square_d, 0.003, Clipper2Lib::JoinType::Miter,
    Clipper2Lib::EndType::Polygon));
  EXPECT_DOUBLE_EQ(Clipper2Lib::Area(solution_d), 1);
}
