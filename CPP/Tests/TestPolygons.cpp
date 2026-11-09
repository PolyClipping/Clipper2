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
    else if (IsInList(test_number, { 15, 52, 53, 54, 59, 60, 64, 117, 119, 184 }))
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
