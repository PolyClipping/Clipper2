#include <gtest/gtest.h>
#include "clipper2/clipper.h"
#include "ClipFileLoad.h"
#include <stdexcept>
inline Clipper2Lib::PathD MakeRandomPath(int width, int height, unsigned vertCnt)
{
  Clipper2Lib::PathD result;
  result.reserve(vertCnt);
  for (unsigned i = 0; i < vertCnt; ++i)
    result.push_back(Clipper2Lib::PointD(double(rand()) / RAND_MAX * width, double(rand()) / RAND_MAX * height));
  return result;
}
template <size_t N>
inline bool IsInList(int num, const int (&intArray)[N])
{
  const int* list = &intArray[0];
  for (int cnt = N; cnt; --cnt)
    if (num == *list++) return true;
  return false;
}
TEST(Clipper2Tests, TestMultiplePolygons)
{
  std::ifstream ifs("Polygons.txt");
  ASSERT_TRUE(ifs.good());
  const int start_num = 1;
  const int end_num = 1000;
  int test_number = start_num;
  while (test_number <= end_num)
  {
    Clipper2Lib::Paths64 subject, subject_open, clip;
    Clipper2Lib::Paths64 solution, solution_open;
    Clipper2Lib::ClipType ct;
    Clipper2Lib::FillRule fr;
    int64_t stored_area, stored_count;
    if (!LoadTestNum(ifs, test_number,
      subject, subject_open, clip, stored_area, stored_count, ct, fr)) break;
    // check Paths64 solutions
    Clipper2Lib::Clipper64 c;
    c.AddSubject(subject);
    c.AddOpenSubject(subject_open);
    c.AddClip(clip);
    c.Execute(ct, fr, solution, solution_open);
    const int64_t measured_area = static_cast<int64_t>(Area(solution));
    const int64_t measured_count = static_cast<int64_t>(solution.size() + solution_open.size());
    // check the polytree variant too
    Clipper2Lib::PolyTree64 solution_polytree;
    Clipper2Lib::Paths64 solution_polytree_open;
    Clipper2Lib::Clipper64 clipper_polytree;
    clipper_polytree.AddSubject(subject);
    clipper_polytree.AddOpenSubject(subject_open);
    clipper_polytree.AddClip(clip);
    clipper_polytree.Execute(ct, fr, solution_polytree, solution_polytree_open);
    const int64_t measured_area_polytree =
      static_cast<int64_t>(solution_polytree.Area());
    const auto solution_polytree_paths = PolyTreeToPaths64(solution_polytree);
    const int64_t measured_count_polytree =
      static_cast<int64_t>(solution_polytree_paths.size());
    // check polygon counts
    if (stored_count <= 0)
      ; // skip count
    else if (IsInList(test_number, { 120, 121, 130, 138,
      140, 148, 163, 165, 166, 167, 168, 172, 173, 175, 178, 180 }))
      EXPECT_NEAR(measured_count, stored_count, 5) << " in test " << test_number;
    else if (IsInList(test_number, { 126 }))
      EXPECT_NEAR(measured_count, stored_count, 3) << " in test " << test_number;
    else if (IsInList(test_number, { 16, 27, 181 }))
      EXPECT_NEAR(measured_count, stored_count, 2) << " in test " << test_number;
    else if (test_number >= 120 && test_number <= 184)
      EXPECT_NEAR(measured_count, stored_count, 2) << " in test " << test_number;
    else if (IsInList(test_number, { 23, 45, 87, 102, 111, 113, 191 }))
      EXPECT_NEAR(measured_count, stored_count, 1) << " in test " << test_number;
    else
      EXPECT_EQ(measured_count, stored_count) << " in test " << test_number;
    // check polygon areas
    if (stored_area <= 0)
      ; // skip area
    else if (IsInList(test_number, { 19, 22, 23, 24 }))
      EXPECT_NEAR(measured_area, stored_area, 0.5 * measured_area) << " in test " << test_number;
    else if (test_number == 193)
      EXPECT_NEAR(measured_area, stored_area, 0.2 * measured_area) << " in test " << test_number;
    else if (test_number == 63)
      EXPECT_NEAR(measured_area, stored_area, 0.1 * measured_area) << " in test " << test_number;
    else if (test_number == 16)
      EXPECT_NEAR(measured_area, stored_area, 0.075 * measured_area) << " in test " << test_number;
    else if (test_number == 26)
      EXPECT_NEAR(measured_area, stored_area, 0.05 * measured_area) << " in test " << test_number;
    else if (IsInList(test_number, { 15, 52, 53, 54, 59, 60, 64, 66, 117, 119, 184 }))
      EXPECT_NEAR(measured_area, stored_area, 0.02 * measured_area) << " in test " << test_number;
    else
      EXPECT_NEAR(measured_area, stored_area, 0.01 * measured_area) << " in test " << test_number;
    EXPECT_EQ(measured_count, measured_count_polytree)
      << " in test " << test_number;
    EXPECT_EQ(measured_area, measured_area_polytree)
      << " in test " << test_number;
    ++test_number;
  }
  //EXPECT_GE(test_number, 188);
  Clipper2Lib::PathsD subjd, clipd, solutiond;
  Clipper2Lib::FillRule frd = Clipper2Lib::FillRule::NonZero;
}

TEST(Clipper2Tests, TestDifferenceClampsIntersectionToSegment)
{
  // the lines of two edges cross just beyond the subject vertex at 6,6, and with
  // CLIPPER2_HI_PRECISION that crossing used to be returned unclamped, leaving a reversed contour
  Clipper2Lib::Paths64 subject = { Clipper2Lib::MakePath({ 6,6, 8,2, 2,12, 3,7, 12,4 }) };
  Clipper2Lib::Paths64 clip = { Clipper2Lib::MakePath({ 10,3, 7,12, 3,1, 2,11, 11,2, 1,8 }) };
  Clipper2Lib::Paths64 solution = Clipper2Lib::Difference(subject, clip, Clipper2Lib::FillRule::EvenOdd);

  EXPECT_TRUE(std::all_of(solution.begin(), solution.end(),
    [](const Clipper2Lib::Path64& path) { return Clipper2Lib::IsPositive(path); }));
}

#if defined(CLIPPER2_HI_PRECISION) && CLIPPER2_HI_PRECISION
TEST(Clipper2Tests, TestHiPrecisionIntersectNearSegmentEnd)
{
  using Clipper2Lib::Point64;
  using Clipper2Lib::PointD;
  Point64 ip;
  PointD ipd;

  // the crossing at 10,10 is inside the first segment, though a rounded crossing can't tell
  const Point64 a(1000000000, 1000000000), b(0, 0), c(10, 10), d(-999999990, -999999989);
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(a, b, c, d, ip));
  EXPECT_EQ(ip, Point64(10, 10));
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(PointD(a), PointD(b), PointD(c), PointD(d), ipd));
  EXPECT_NEAR(ipd.x, 10, 1e-6);
  EXPECT_NEAR(ipd.y, 10, 1e-6);

  // the crossing is beside the first segment's start, where a rounded crossing cancels to that start
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(PointD(1e12, 0.0), PointD(0.0, 1e12), PointD(0.0, 1e-5),
    PointD(1e12, 1e-5), ipd));
  EXPECT_DOUBLE_EQ(ipd.x, 1e12);
  EXPECT_DOUBLE_EQ(ipd.y, 1e-5);
  using PointF = Clipper2Lib::Point<float>;
  PointF ipf;
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(PointF(1e12f, 0.0f), PointF(0.0f, 1e12f), PointF(0.0f, 1e-5f),
    PointF(1e12f, 1e-5f), ipf));
  EXPECT_FLOAT_EQ(ipf.x, 1e12f);
  EXPECT_FLOAT_EQ(ipf.y, 1e-5f);

  // long double ends that differ by less than double's precision are still told apart
  using PointL = Clipper2Lib::Point<long double>;
  const long double tiny = std::ldexp(1.0L, 4 - std::numeric_limits<long double>::digits);
  PointL ipl;
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(PointL(1 + tiny, 0.0L), PointL(1.0L, 1.0L), PointL(1 + tiny / 2, 0.0L),
    PointL(1 + tiny / 2, 1.0L), ipl));
  EXPECT_EQ(ipl, PointL(1 + tiny / 2, 0.5L));

  // the crossing at m is inside the first segment, though products of such short vectors underflow
  const PointD m(std::ldexp(1.0, -530), std::ldexp(1.0, -530));
  const PointD n(2 * m.x, (2 + std::ldexp(1.0, -51)) * m.y);
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(PointD(0.0, 0.0), PointD(1.0, 1.0), m, n, ipd));
  EXPECT_TRUE(ipd.x >= m.x && ipd.x <= n.x && ipd.y >= m.y && ipd.y <= n.y);

  // the lines cross at 0,0, beyond the first segment's second end
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(Point64(500000001, 500000001), Point64(500000000, 500000000),
    Point64(0, 0), Point64(1000000000, 999999999), ip));
  EXPECT_EQ(ip, Point64(500000000, 500000000));

  // segments leaving a shared vertex meet exactly there
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(Point64(889821762, 0), Point64(441613919, -370558076),
    Point64(889821762, 0), Point64(665717839, -185279037), ip));
  EXPECT_EQ(ip, Point64(889821762, 0));

  // near parallel lines at huge coordinates, whose rounded crossing lands far outside both segments
  const Point64 e(882221495591671156, -709114218932650014), f(-1988259314537543047, 880639977513308803);
  const Point64 g(2169548593363292491, -1422072706116861579), h(1389069062445448553, -989820866934225391);
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(e, f, g, h, ip));
  EXPECT_TRUE(ip.x >= f.x && ip.x <= e.x && ip.y >= e.y && ip.y <= f.y);

  // narrower coordinate types are classified in 64 bits
  using Point16 = Clipper2Lib::Point<int16_t>;
  Point16 ip16;
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(Point16(0, 0), Point16(20000, 100), Point16(10000, 100),
    Point16(30000, 150), ip16));
  EXPECT_EQ(ip16, Point16(20000, 100));
  using Point32 = Clipper2Lib::Point<int32_t>;
  Point32 ip32;
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(Point32(800000000, 0), Point32(1000000000, 100),
    Point32(1100000000, 200), Point32(1400000000, 300), ip32));
  EXPECT_EQ(ip32, Point32(1000000000, 100));
}
#endif

TEST(Clipper2Tests, TestIntersectInt32SpanBeyondIntMax)
{
  // the segments' spans overflow int, so their ends must be subtracted in 64 bits
  using Point32 = Clipper2Lib::Point<int32_t>;
  Point32 ip;
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(Point32(-2000000000, -1000000000), Point32(2000000000, 1000000000),
    Point32(-1000000000, 2000000000), Point32(1000000000, -2000000000), ip));
  EXPECT_EQ(ip, Point32(0, 0));
}

TEST(Clipper2Tests, TestHorzSpikes) //#720
{
  Clipper2Lib::Paths64 paths = {
    Clipper2Lib::MakePath({1600,0, 1600,100, 2050,100, 2050,300, 450,300, 450, 0}),
    Clipper2Lib::MakePath({1800,200, 1800,100, 1600,100, 2000,100, 2000,200}) };
  std::cout << paths << std::endl;
  Clipper2Lib::Clipper64 c;
  c.AddSubject(paths);
  c.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, paths);
  EXPECT_GE(paths.size(), 1);
}

TEST(Clipper2Tests, TestCollinearOnMacOs) //#777
{
  Clipper2Lib::Paths64 subject;
  subject.push_back(Clipper2Lib::MakePath({ 0, -453054451,0, -433253797,-455550000, 0 }));
  subject.push_back(Clipper2Lib::MakePath({ 0, -433253797,0, 0,-455550000, 0 }));
  Clipper2Lib::Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(subject);
  Clipper2Lib::Paths64 solution;
  clipper.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, solution);
  ASSERT_EQ(solution.size(), 1);
  EXPECT_EQ(solution[0].size(), 3);
  EXPECT_EQ(IsPositive(subject[0]), IsPositive(solution[0]));
}

#ifdef USINGZ
TEST(Clipper2Tests, TestExecuteAfterZCallbackThrows)
{
  // a clipper must be left clean when an exception escapes Execute
  Clipper2Lib::Paths64 subject = {
    Clipper2Lib::MakePath({ 0,0, 100,0, 100,100, 0,100 }),
    Clipper2Lib::MakePath({ 50,50, 150,50, 150,150, 50,150 }) };
  Clipper2Lib::Paths64 clip = { Clipper2Lib::MakePath({ 20,-10, 80,-10, 80,200, 20,200 }) };

  Clipper2Lib::Clipper64 fresh;
  fresh.AddSubject(subject);
  fresh.AddClip(clip);
  Clipper2Lib::Paths64 expected;
  fresh.Execute(Clipper2Lib::ClipType::Xor, Clipper2Lib::FillRule::NonZero, expected);

  Clipper2Lib::Clipper64 clipper;
  clipper.AddSubject(subject);
  clipper.AddClip(clip);
  int calls = 0;
  clipper.SetZCallback([&calls](const Clipper2Lib::Point64&, const Clipper2Lib::Point64&,
    const Clipper2Lib::Point64&, const Clipper2Lib::Point64&, Clipper2Lib::Point64&)
    {
      if (++calls == 3) throw std::runtime_error("Z callback");
    });
  Clipper2Lib::Paths64 solution;
  EXPECT_THROW(clipper.Execute(Clipper2Lib::ClipType::Xor, Clipper2Lib::FillRule::NonZero, solution),
    std::runtime_error);

  clipper.SetZCallback(nullptr);
  clipper.Execute(Clipper2Lib::ClipType::Xor, Clipper2Lib::FillRule::NonZero, solution);
  EXPECT_EQ(solution, expected);
}
#endif

TEST(Clipper2Tests, TestXorDivergingSharedEdges) // PolyClipping/Clipper2#13
{
  // the paths share edges that diverge at 33889,18931 by less than half a unit for several scanbeams
  Clipper2Lib::Paths64 subject, clip;
  subject.push_back(Clipper2Lib::MakePath({
      97513,3213, 97542,3449, 97548,169199, 97559,169465, 97626,170263, 97663,170547, 97747,170984, 97915,171671,
      97999,171956, 98156,172387, 98420,173010, 98536,173254, 98766,173674, 99129,174259, 99290,174494, 99585,174873,
      100292,175675, 133417,208800, 133586,209021, 133442,209203, 129002,213655, 128782,213824, 128600,213682,
      7772,92854, 7604,92634, 7752,92446, 13507,86686, 13738,86509, 14008,86472, 26760,86465, 27036,86453,
      27811,86385, 28080,86349, 28535,86262, 29186,86101, 29446,86025, 29917,85856, 30530,85596, 30783,85475,
      31229,85228, 31842,84841, 32182,84600, 32810,84072, 33032,83862, 33342,83530, 33958,82797, 34128,82575,
      34408,82159, 34675,81711, 34805,81471, 35000,81054, 35259,80418, 35352,80157, 35483,79709, 35634,79071,
      35688,78786, 35750,78289, 35820,77190, 35816,24593, 35807,24353, 35739,23457, 35684,23032, 35633,22770,
      35468,22076, 35370,21745, 35176,21200, 35042,20881, 34747,20272, 34566,19941, 34037,19135, 33889,18931,
      33621,18600, 32723,17588, 32698,3608, 32710,3376, 32737,3177, 97246,3177 }));
  clip.push_back(Clipper2Lib::MakePath({
      97512,3212, 97542,3449, 97548,169199, 97559,169465, 97626,170263, 97663,170547, 97747,170984, 97915,171671,
      97999,171956, 98156,172387, 98420,173010, 98536,173254, 98766,173674, 99129,174259, 99290,174494, 99585,174873,
      100292,175675, 133417,208800, 133586,209021, 133442,209203, 129002,213655, 128782,213824, 128600,213682,
      7772,92854, 7604,92634, 7752,92446, 13507,86686, 13738,86509, 14008,86472, 26760,86465, 27036,86453,
      27811,86385, 28080,86349, 28535,86262, 29186,86101, 29446,86025, 29917,85856, 30530,85596, 30783,85475,
      31229,85228, 31842,84841, 32182,84600, 32810,84072, 33032,83862, 33342,83530, 33958,82797, 34128,82575,
      34408,82159, 34675,81711, 34805,81471, 35000,81054, 35259,80418, 35352,80157, 35483,79709, 35634,79071,
      35688,78786, 35750,78289, 35820,77190, 35816,24593, 35807,24353, 35739,23457, 35684,23032, 35633,22770,
      35468,22076, 35370,21745, 35176,21200, 35042,20881, 34747,20272, 34566,19941, 34037,19135, 33889,18931,
      33622,18601, 33118,18033, 33040,17609, 33017,3570, 33032,3337, 33058,3177, 97246,3177 }));

  Clipper2Lib::Paths64 solution = Clipper2Lib::Xor(subject, clip, Clipper2Lib::FillRule::EvenOdd);
  Clipper2Lib::Paths64 swapped = Clipper2Lib::Xor(clip, subject, Clipper2Lib::FillRule::EvenOdd);
  ASSERT_FALSE(solution.empty());
  ASSERT_FALSE(swapped.empty());

  for (const Clipper2Lib::Path64& path : solution)
    EXPECT_TRUE(Clipper2Lib::IsPositive(path));

  for (const Clipper2Lib::Path64& path : swapped)
    EXPECT_TRUE(Clipper2Lib::IsPositive(path));

  EXPECT_EQ(Clipper2Lib::Area(solution), Clipper2Lib::Area(swapped));
}

TEST(Clipper2Tests, TestDifferenceRoundsIntersections) // PolyClipping/Clipper2#13
{
  // the exact difference has an area of 3.27; truncating its intersection points
  // reversed the only contour to an area of -3.5
  Clipper2Lib::Paths64 subject = { Clipper2Lib::MakePath({ 0,10, 5,2, 7,1 }) };
  Clipper2Lib::Paths64 clip = { Clipper2Lib::MakePath({ 3,6, 6,1, 11,11 }) };
  Clipper2Lib::Paths64 solution = Clipper2Lib::Difference(subject, clip, Clipper2Lib::FillRule::NonZero);

  ASSERT_EQ(solution.size(), 1);
  EXPECT_TRUE(Clipper2Lib::IsPositive(solution[0]));
  EXPECT_NEAR(Clipper2Lib::Area(solution), 3.27, 1.0);
}

#if !defined(CLIPPER2_HI_PRECISION) || !CLIPPER2_HI_PRECISION
TEST(Clipper2Tests, TestIntersectPointIsNotFused)
{
  // both exact intersections have a coordinate of -100.5, and the tie rounds to even unless
  // the multiply and add in GetLineIntersectPt are fused into one rounding, as Clang does
  // by default on arm64 and GCC does on any target with FMA
  using Clipper2Lib::Point64;
  // reading a volatile stops the compiler folding the calls into constants without fusing
  volatile int64_t zero = 0;
  auto pt = [&zero](int64_t x, int64_t y) { return Point64(x + zero, y + zero); };
  Point64 ip;
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(pt(-207, -383), pt(350, 194), pt(347, 193), pt(-206, -384), ip));
  EXPECT_EQ(ip, Point64(66, -100));
  ASSERT_TRUE(Clipper2Lib::GetLineIntersectPt(pt(-383, -207), pt(194, 350), pt(193, 347), pt(-384, -206), ip));
  EXPECT_EQ(ip, Point64(-100, 66));
}
#endif
