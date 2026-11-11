/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  11 October 2025                                                 *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  Path Offset (Inflate/Shrink)                                    *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#include "clipper2/clipper.h"
#include "clipper2/clipper.offset.h"

CLIPPER2_NO_FP_CONTRACT

namespace Clipper2Lib {

const double floating_point_tolerance = 1e-12;

// Clipper2 approximates arcs by using series of relatively short straight
//line segments. And logically, shorter line segments will produce better arc
// approximations. But very short segments can degrade performance, usually
// with little or no discernable improvement in curve quality. Very short
// segments can even detract from curve quality, due to the effects of integer
// rounding. Since there isn't an optimal number of line segments for any given
// arc radius (that perfectly balances curve approximation with performance),
// arc tolerance is user defined. Nevertheless, when the user doesn't define
// an arc tolerance (ie leaves alone the 0 default value), the calculated
// default arc tolerance (offset_radius / 500) generally produces good (smooth)
// arc approximations without producing excessively small segment lengths.
// See also: https://github.com/PolyClipping/Clipper2/wiki/Trigonometry
const double arc_const = 0.002; // <-- 1/500


//------------------------------------------------------------------------------
// Miscellaneous methods
//------------------------------------------------------------------------------

// adds a * b, or subtracts it, to the 128-bit two's complement value hi:lo, wrapping around
static void AddProduct(uint64_t& hi, uint64_t& lo, int64_t a, int64_t b, bool subtract)
{
	const uint64_t ua = static_cast<uint64_t>(a), ub = static_cast<uint64_t>(b);
	const UInt128Struct product = MultiplyUInt64(ua, ub);
	uint64_t product_lo = product.lo;
	// corrects the unsigned product to the signed one, modulo 2^128
	uint64_t product_hi = product.hi - (a < 0 ? ub : 0) - (b < 0 ? ua : 0);

	if (subtract)
	{
		product_lo = ~product_lo + 1;
		product_hi = ~product_hi + (product_lo == 0 ? 1 : 0);
	}

	lo += product_lo;
	hi += product_hi + (lo < product_lo ? 1 : 0);
}

// true when path lies inside a container, judged by the first of its vertices that locate
// doesn't put on the container's edge. A path touching it at every vertex counts as inside.
template <typename Locate>
static bool IsInside(const Path64& path, Locate locate)
{
	for (const Point64& pt : path)
	{
		PointInPolygonResult pip = locate(pt);

		if (pip != PointInPolygonResult::IsOn) return pip == PointInPolygonResult::IsInside;
	}

	return true;
}

// true when any vertex of path is inside a container, or all of them are on its edge
template <typename Locate>
static bool HasVertexInside(const Path64& path, Locate locate)
{
	bool all_on = true;

	for (const Point64& pt : path)
	{
		PointInPolygonResult pip = locate(pt);

		if (pip == PointInPolygonResult::IsInside) return true;

		if (pip == PointInPolygonResult::IsOutside) all_on = false;
	}

	return all_on;
}

inline double Hypot(double x, double y)
{
	// given that this is an internal function, and given the x and y parameters
	// will always be coordinate values (or the difference between coordinate values),
	// x and y should always be within INT64_MIN to INT64_MAX. Consequently, 
	// there should be no risk that the following computation will overflow
	// see https://stackoverflow.com/a/32436148/359538
	return std::sqrt(x * x + y * y);
}

static PointD GetUnitNormal(const Point64& pt1, const Point64& pt2)
{
	if (pt1 == pt2) return PointD(0.0, 0.0);
	double dx = static_cast<double>(pt2.x - pt1.x);
	double dy = static_cast<double>(pt2.y - pt1.y);
	double inverse_hypot = 1.0 / Hypot(dx, dy);
	dx *= inverse_hypot;
	dy *= inverse_hypot;
	return PointD(dy, -dx);
}

inline bool AlmostZero(double value, double epsilon = 0.001)
{
	return std::fabs(value) < epsilon;
}

inline PointD NormalizeVector(const PointD& vec)
{
	double h = Hypot(vec.x, vec.y);
	if (AlmostZero(h)) return PointD(0,0);
	double inverseHypot = 1 / h;
	return PointD(vec.x * inverseHypot, vec.y * inverseHypot);
}

inline PointD GetAvgUnitVector(const PointD& vec1, const PointD& vec2)
{
	return NormalizeVector(PointD(vec1.x + vec2.x, vec1.y + vec2.y));
}

inline bool IsClosedPath(EndType et)
{
	return et == EndType::Polygon || et == EndType::Joined;
}

// true when every vertex lies on one line, so the path encloses no area
static bool IsFlat(const Path64& path)
{
	auto it = std::find_if(path.cbegin(), path.cend(), [&](const Point64& pt) { return pt != path[0]; });

	if (it == path.cend()) return true;

	const Point64& pt1 = *it;
	return std::all_of(it + 1, path.cend(),
		[&](const Point64& pt) { return CrossProductSign(path[0], pt1, pt) == 0; });
}

static inline Point64 GetPerpendic(const Point64& pt, const PointD& norm, double delta)
{
#ifdef USINGZ
	return Point64(pt.x + norm.x * delta, pt.y + norm.y * delta, pt.z);
#else
	return Point64(pt.x + norm.x * delta, pt.y + norm.y * delta);
#endif
}

inline PointD GetPerpendicD(const Point64& pt, const PointD& norm, double delta)
{
#ifdef USINGZ
	return PointD(pt.x + norm.x * delta, pt.y + norm.y * delta, pt.z);
#else
	return PointD(pt.x + norm.x * delta, pt.y + norm.y * delta);
#endif
}

inline void NegatePath(PathD& path)
{
	for (PointD& pt : path)
	{
		pt.x = -pt.x;
		pt.y = -pt.y;
#ifdef USINGZ
		pt.z = pt.z;
#endif
	}
}


// A static R-tree, packed by sorting the boxes into tiles. Tiling by rank rather than by
// coordinate keeps it balanced however the boxes cluster. Boxes can be marked active, and each
// node counts its active boxes, so a query for active boxes skips whole subtrees of the others.
class BoundsTree
{
public:
	explicit BoundsTree(std::vector<Rect64> boxes) : boxes_(std::move(boxes)), is_active_(boxes_.size())
	{
		std::vector<Rect64> rects = boxes_;

		while (!rects.empty())
		{
			Level level;
			level.items = TileOrder(rects);
			level.parents.resize(rects.size());
			std::vector<Rect64> node_rects;

			for (size_t first = 0; first < level.items.size(); first += node_size)
			{
				Node node{ InvalidRect64, first, (std::min)(node_size, level.items.size() - first), 0 };

				for (size_t i = first; i < first + node.count; ++i)
				{
					const Rect64& r = rects[level.items[i]];
					node.bounds.left = (std::min)(node.bounds.left, r.left);
					node.bounds.top = (std::min)(node.bounds.top, r.top);
					node.bounds.right = (std::max)(node.bounds.right, r.right);
					node.bounds.bottom = (std::max)(node.bounds.bottom, r.bottom);
				}

				for (size_t i = first; i < first + node.count; ++i)
					level.parents[level.items[i]] = level.nodes.size();

				level.nodes.push_back(node);
				node_rects.push_back(node.bounds);
			}

			levels_.push_back(std::move(level));

			if (node_rects.size() == 1) break;

			rects = std::move(node_rects);
		}
	}

	void SetActive(size_t box, bool is_active)
	{
		if (is_active_[box] == is_active) return;

		is_active_[box] = is_active;
		size_t item = box;

		for (Level& level : levels_)
		{
			item = level.parents[item];
			Node& node = level.nodes[item];
			node.active_count = is_active ? node.active_count + 1 : node.active_count - 1;
		}
	}

	// calls fn with the index of every box that meets, or of every active one, where meets must
	// also hold for every box enclosing one that it holds for
	template <typename Meets, typename Fn>
	void Query(Meets meets, Fn fn, bool active_only = false) const
	{
		if (levels_.empty()) return;

		std::vector<std::pair<size_t, size_t>> stack; // level and node

		for (size_t i = 0; i < levels_.back().nodes.size(); ++i)
			stack.emplace_back(levels_.size() - 1, i);

		while (!stack.empty())
		{
			const auto [level_idx, node_idx] = stack.back();
			stack.pop_back();
			const Level& level = levels_[level_idx];
			const Node& node = level.nodes[node_idx];

			if ((active_only && node.active_count == 0) || !meets(node.bounds)) continue;

			for (size_t i = node.first; i < node.first + node.count; ++i)
			{
				const size_t item = level.items[i];

				if (level_idx > 0) stack.emplace_back(level_idx - 1, item);
				else if ((!active_only || is_active_[item]) && meets(boxes_[item])) fn(item);
			}
		}
	}

private:
	static constexpr size_t node_size = 16;
	struct Node { Rect64 bounds; size_t first; size_t count; size_t active_count; };
	// each node covers a run of items, which index the nodes of the level below, or the boxes,
	// and parents maps each of those to the node covering it
	struct Level { std::vector<size_t> items; std::vector<size_t> parents; std::vector<Node> nodes; };
	std::vector<Rect64> boxes_;
	std::vector<bool> is_active_;
	std::vector<Level> levels_;

	// sorts by x into vertical slices about as many as there are tiles per slice, then by y
	static std::vector<size_t> TileOrder(const std::vector<Rect64>& rects)
	{
		std::vector<size_t> order(rects.size());
		std::iota(order.begin(), order.end(), size_t(0));
		auto mid_x = [&](size_t i) { return rects[i].left / 2.0 + rects[i].right / 2.0; };
		auto mid_y = [&](size_t i) { return rects[i].top / 2.0 + rects[i].bottom / 2.0; };
		std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return mid_x(a) < mid_x(b); });
		const size_t tiles = (rects.size() + node_size - 1) / node_size;
		const size_t slices = static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(tiles))));
		const size_t slice_len = ((tiles + slices - 1) / slices) * node_size;

		for (size_t first = 0; first < order.size(); first += slice_len)
		{
			auto last = order.begin() + (std::min)(first + slice_len, order.size());
			std::sort(order.begin() + first, last, [&](size_t a, size_t b) { return mid_y(a) < mid_y(b); });
		}

		return order;
	}
};

// Locates points in one polygon with the same result as PointInPolygon, but visits only the
// edges whose bounds hold the point, then those crossing the point's y to its left. The
// even-odd parity of a point off the edges doesn't depend on which ray counts the crossings.
class PolygonIndex
{
public:
	explicit PolygonIndex(const Path64& polygon) : polygon_(polygon), edges_(EdgeBounds(polygon)) {}

	PointInPolygonResult Locate(const Point64& pt) const
	{
		bool is_on = false;

		edges_.Query([&](const Rect64& r) {
			return !is_on && r.left <= pt.x && r.right >= pt.x && r.top <= pt.y && r.bottom >= pt.y;
		}, [&](size_t i) {
			is_on = CrossProductSign(Vertex(i), Vertex(i + 1), pt) == 0;
		});

		if (is_on) return PointInPolygonResult::IsOn;

		// with a half-open rule, an edge lying along the ray never crosses it
		bool is_inside = false;

		edges_.Query([&](const Rect64& r) { return r.left <= pt.x && r.top <= pt.y && r.bottom > pt.y; },
			[&](size_t i) {
				const Point64& a = Vertex(i);
				const Point64& b = Vertex(i + 1);
				const int d = CrossProductSign(a, b, pt);

				if ((a.y > pt.y) != (b.y > pt.y) && ((a.y < b.y) ? d < 0 : d > 0)) is_inside = !is_inside;
			});

		return is_inside ? PointInPolygonResult::IsInside : PointInPolygonResult::IsOutside;
	}

private:
	const Path64& polygon_;
	BoundsTree edges_;

	const Point64& Vertex(size_t i) const { return polygon_[(i == polygon_.size()) ? 0 : i]; }

	static std::vector<Rect64> EdgeBounds(const Path64& polygon)
	{
		std::vector<Rect64> result;
		result.reserve(polygon.size());

		for (size_t i = 0; i < polygon.size(); ++i)
		{
			const Point64& a = polygon[i];
			const Point64& b = polygon[(i + 1 == polygon.size()) ? 0 : i + 1];
			result.emplace_back((std::min)(a.x, b.x), (std::min)(a.y, b.y), (std::max)(a.x, b.x),
				(std::max)(a.y, b.y));
		}

		return result;
	}
};

// Locates points in a set of polygons. A large polygon gets an edge index on its first use, since
// many small paths may be tested against it.
class PolygonLocator
{
public:
	explicit PolygonLocator(size_t count) : indexes_(count) {}

	PointInPolygonResult Locate(const Point64& pt, const Path64& polygon, size_t polygon_idx)
	{
		if (polygon.size() < 64) return PointInPolygon(pt, polygon);

		if (!indexes_[polygon_idx]) indexes_[polygon_idx] = std::make_unique<PolygonIndex>(polygon);

		return indexes_[polygon_idx]->Locate(pt);
	}

private:
	std::vector<std::unique_ptr<PolygonIndex>> indexes_;
};

//------------------------------------------------------------------------------
// ClipperOffset::Group methods
//------------------------------------------------------------------------------

ClipperOffset::Group::Group(const Paths64& _paths, JoinType _join_type, EndType _end_type):
	paths_in(_paths), join_type(_join_type), end_type(_end_type)
{
	bool is_joined =
		(end_type == EndType::Polygon) ||
		(end_type == EndType::Joined);
	for (Path64& p: paths_in)
	  StripDuplicates(p, is_joined);

	// the offset code assumes every path has at least one vertex
	paths_in.erase(std::remove_if(paths_in.begin(), paths_in.end(),
		[](const Path64& p) { return p.empty(); }), paths_in.end());

	if (end_type != EndType::Polygon) return;

	areas.reserve(paths_in.size());

	for (const Path64& p : paths_in)
		areas.emplace_back(p);

	// sizes compare exactly, so paths of nearly equal size rank the same however they're wound.
	// The first path wins a tie.
	std::pair<uint64_t, uint64_t> largest;

	for (size_t i = 0; i < areas.size(); ++i)
	{
		if (areas[i].twice_magnitude > largest)
		{
			largest = areas[i].twice_magnitude;
			outer_sign = areas[i].sign;
			outer_idx = i;
		}
	}
}

ClipperOffset::Group::ExactArea::ExactArea(const Path64& path)
{
	// the shoelace sum may wrap around on the way, but it ends exact whenever twice the area fits
	// in 127 bits, as it does within Clipper's coordinate range unless the path winds around often
	uint64_t hi = 0, lo = 0;

	for (size_t i = 0, j = path.size() - 1; i < path.size(); j = i++)
	{
		AddProduct(hi, lo, path[j].x, path[i].y, false);
		AddProduct(hi, lo, path[i].x, path[j].y, true);
	}

	if (hi >> 63)
	{
		sign = -1;
		lo = ~lo + 1;
		hi = ~hi + (lo == 0 ? 1 : 0);
	}
	else if (hi || lo)
	{
		sign = 1;
	}

	twice_magnitude = { hi, lo };
}

//------------------------------------------------------------------------------
// ClipperOffset methods
//------------------------------------------------------------------------------

void ClipperOffset::AddPath(const Path64& path, JoinType jt_, EndType et_)
{
    groups_.emplace_back(Paths64(1, path), jt_, et_);
}

void ClipperOffset::AddPaths(const Paths64 &paths, JoinType jt_, EndType et_)
{
	if (paths.size() == 0) return;
    groups_.emplace_back(paths, jt_, et_);
}

void ClipperOffset::BuildNormals(const Path64& path)
{
	norms.clear();
	norms.reserve(path.size());
	if (path.size() == 0) return;
	Path64::const_iterator path_iter, path_stop_iter = --path.cend();
	for (path_iter = path.cbegin(); path_iter != path_stop_iter; ++path_iter)
        norms.emplace_back(GetUnitNormal(*path_iter,*(path_iter +1)));
    norms.emplace_back(GetUnitNormal(*path_stop_iter, *(path.cbegin())));
}

void ClipperOffset::SetArcSteps(double delta)
{
	// calculate the number of steps required to approximate a circle
	// (see https://github.com/PolyClipping/Clipper2/wiki/Trigonometry)
	// arcTol - when arc_tolerance_ is undefined (0) then curve imprecision
	// will be relative to the size of the offset (delta). Obviously very
	//large offsets will almost always require much less precision.
	double abs_delta = std::fabs(delta);
	double arcTol = (arc_tolerance_ > floating_point_tolerance) ?
		std::min(abs_delta, arc_tolerance_) : abs_delta * arc_const;

	double steps_per_360 = std::min(PI / std::acos(1 - arcTol / abs_delta), abs_delta * PI);
	step_sin_ = std::sin(2 * PI / steps_per_360);
	step_cos_ = std::cos(2 * PI / steps_per_360);
	if (delta < 0.0) step_sin_ = -step_sin_;
	steps_per_rad_ = steps_per_360 / (2 * PI);
}

void ClipperOffset::DoBevel(const Path64& path, size_t j, size_t k)
{
	PointD pt1, pt2;
	if (j == k)
	{
		double abs_delta = std::abs(group_delta_);
#ifdef USINGZ
		pt1 = PointD(path[j].x - abs_delta * norms[j].x, path[j].y - abs_delta * norms[j].y, path[j].z);
		pt2 = PointD(path[j].x + abs_delta * norms[j].x, path[j].y + abs_delta * norms[j].y, path[j].z);
#else
		pt1 = PointD(path[j].x - abs_delta * norms[j].x, path[j].y - abs_delta * norms[j].y);
		pt2 = PointD(path[j].x + abs_delta * norms[j].x, path[j].y + abs_delta * norms[j].y);
#endif
	}
	else
	{
#ifdef USINGZ
		pt1 = PointD(path[j].x + group_delta_ * norms[k].x, path[j].y + group_delta_ * norms[k].y, path[j].z);
		pt2 = PointD(path[j].x + group_delta_ * norms[j].x, path[j].y + group_delta_ * norms[j].y, path[j].z);
#else
		pt1 = PointD(path[j].x + group_delta_ * norms[k].x, path[j].y + group_delta_ * norms[k].y);
		pt2 = PointD(path[j].x + group_delta_ * norms[j].x, path[j].y + group_delta_ * norms[j].y);
#endif
	}
    path_out.emplace_back(pt1);
    path_out.emplace_back(pt2);
}

void ClipperOffset::DoSquare(const Path64& path, size_t j, size_t k)
{
	PointD vec;
	if (j == k)
		vec = PointD(norms[j].y, -norms[j].x);
	else
		vec = GetAvgUnitVector(
			PointD(-norms[k].y, norms[k].x),
			PointD(norms[j].y, -norms[j].x));

	double abs_delta = std::abs(group_delta_);

	// now offset the original vertex delta units along unit vector
	PointD ptQ = PointD(path[j]);
	ptQ = TranslatePoint(ptQ, abs_delta * vec.x, abs_delta * vec.y);
	// get perpendicular vertices
	PointD pt1 = TranslatePoint(ptQ, group_delta_ * vec.y, group_delta_ * -vec.x);
	PointD pt2 = TranslatePoint(ptQ, group_delta_ * -vec.y, group_delta_ * vec.x);
	// get 2 vertices along one edge offset
	PointD pt3 = GetPerpendicD(path[k], norms[k], group_delta_);
	PointD pt4 = (j == k) ?
		PointD(pt3.x + vec.x * group_delta_, pt3.y + vec.y * group_delta_) :
		GetPerpendicD(path[j], norms[k], group_delta_);
	PointD pt = ptQ;
	GetLineIntersectPt(pt1, pt2, pt3, pt4, pt);
#ifdef USINGZ
	pt.z = ptQ.z; // GetLineIntersectPt clears z, but this point belongs to path[j]
#endif

	//get the second intersect point through reflecion
	if (j == k)
	{
		path_out.emplace_back(ReflectPoint(pt, ptQ));
		path_out.emplace_back(pt);
	}
	else
	{
		path_out.emplace_back(pt);
		path_out.emplace_back(ReflectPoint(pt, ptQ));
	}
}

void ClipperOffset::DoMiter(const Path64& path, size_t j, size_t k, double cos_a)
{
	double q = group_delta_ / (cos_a + 1);
#ifdef USINGZ
    path_out.emplace_back(
		path[j].x + (norms[k].x + norms[j].x) * q,
		path[j].y + (norms[k].y + norms[j].y) * q,
        path[j].z);
#else
    path_out.emplace_back(
		path[j].x + (norms[k].x + norms[j].x) * q,
        path[j].y + (norms[k].y + norms[j].y) * q);
#endif
}

void ClipperOffset::DoRound(const Path64& path, size_t j, size_t k, double angle)
{
	// when deltaCallback64_ is assigned, group_delta_ won't be constant,
	// so the arc steps need recalculating for *every* vertex.
	if (deltaCallback64_) SetArcSteps(group_delta_);

	Point64 pt = path[j];
	PointD offsetVec = PointD(norms[k].x * group_delta_, norms[k].y * group_delta_);

	if (j == k) offsetVec.Negate();
#ifdef USINGZ
    path_out.emplace_back(pt.x + offsetVec.x, pt.y + offsetVec.y, pt.z);
#else
    path_out.emplace_back(pt.x + offsetVec.x, pt.y + offsetVec.y);
#endif
	int steps = static_cast<int>(std::ceil(steps_per_rad_ * std::abs(angle))); // #448, #456
	for (int i = 1; i < steps; ++i) // ie 1 less than steps
	{
		offsetVec = PointD(offsetVec.x * step_cos_ - step_sin_ * offsetVec.y,
			offsetVec.x * step_sin_ + offsetVec.y * step_cos_);
#ifdef USINGZ
        path_out.emplace_back(pt.x + offsetVec.x, pt.y + offsetVec.y, pt.z);
#else
        path_out.emplace_back(pt.x + offsetVec.x, pt.y + offsetVec.y);
#endif
	}
    path_out.emplace_back(GetPerpendic(path[j], norms[j], group_delta_));
}

void ClipperOffset::OffsetPoint(Group& group, const Path64& path, size_t j, size_t k)
{
	// Let A = change in angle where edges join
	// A == 0: ie no change in angle (flat join)
	// A == PI: edges 'spike'
	// sin(A) < 0: right turning
	// cos(A) < 0: change in angle is more than 90 degree

	if (path[j] == path[k]) return;

	double sin_a = CrossProduct(norms[j], norms[k]);
	double cos_a = DotProduct(norms[j], norms[k]);
	if (sin_a > 1.0) sin_a = 1.0;
	else if (sin_a < -1.0) sin_a = -1.0;

	if (deltaCallback64_) {
		group_delta_ = deltaCallback64_(path, norms, j, k);
		if (group.is_reversed) group_delta_ = -group_delta_;
	}
	if (std::fabs(group_delta_) <= floating_point_tolerance)
	{
        path_out.emplace_back(path[j]);
		return;
	}

	if (cos_a > -0.999 && (sin_a * group_delta_ < 0)) // test for concavity first (#593)
	{
		// is concave
		// by far the simplest way to construct concave joins, especially those joining very 
		// short segments, is to insert 3 points that produce negative regions. These regions 
		// will be removed later by the finishing union operation. This is also the best way 
		// to ensure that path reversals (ie over-shrunk paths) are removed.
#ifdef USINGZ
        path_out.emplace_back(GetPerpendic(path[j], norms[k], group_delta_), path[j].z);
        path_out.emplace_back(path[j]); // (#405, #873, #916)
        path_out.emplace_back(GetPerpendic(path[j], norms[j], group_delta_), path[j].z);
#else
        path_out.emplace_back(GetPerpendic(path[j], norms[k], group_delta_));
        path_out.emplace_back(path[j]); // (#405, #873, #916)
        path_out.emplace_back(GetPerpendic(path[j], norms[j], group_delta_));
#endif
	}
	else if (cos_a > 0.999 && join_type_ != JoinType::Round)
	{
		// almost straight - less than 2.5 degree (#424, #482, #526 & #724)
		DoMiter(path, j, k, cos_a);
	}
	else if (join_type_ == JoinType::Miter)
	{
		// miter unless the angle is sufficiently acute to exceed ML
		if (cos_a > temp_lim_ - 1) DoMiter(path, j, k, cos_a);
		else DoSquare(path, j, k);
	}
	else if (join_type_ == JoinType::Round)
		DoRound(path, j, k, std::atan2(sin_a, cos_a));
	else if ( join_type_ == JoinType::Bevel)
		DoBevel(path, j, k);
	else
		DoSquare(path, j, k);
}

void ClipperOffset::OffsetPolygon(Group& group, const Path64& path)
{
	path_out.clear();
	for (Path64::size_type j = 0, k = path.size() - 1; j < path.size(); k = j, ++j)
		OffsetPoint(group, path, j, k);

	// a reversed group's outers come out negative, but the solution's outers are always positive
	if (group.is_reversed) std::reverse(path_out.begin(), path_out.end());

	solution->emplace_back(path_out);
}

void ClipperOffset::OffsetOpenJoined(Group& group, const Path64& path)
{
	OffsetPolygon(group, path);
	Path64 reverse_path(path);
	std::reverse(reverse_path.begin(), reverse_path.end());

	//rebuild normals 
	std::reverse(norms.begin(), norms.end());
    norms.emplace_back(norms[0]);
	norms.erase(norms.begin());
	NegatePath(norms);

	OffsetPolygon(group, reverse_path);
}

void ClipperOffset::OffsetOpenPath(Group& group, const Path64& path, EndType end_type)
{
	// do the line start cap
	if (deltaCallback64_) group_delta_ = deltaCallback64_(path, norms, 0, 0);

	if (std::fabs(group_delta_) <= floating_point_tolerance)
        path_out.emplace_back(path[0]);
	else
	{
		switch (end_type)
		{
		case EndType::Butt:
			DoBevel(path, 0, 0);
			break;
		case EndType::Round:
			DoRound(path, 0, 0, PI);
			break;
		default:
			DoSquare(path, 0, 0);
			break;
		}
	}

	size_t highI = path.size() - 1;
	// offset the left side going forward
	for (Path64::size_type j = 1, k = 0; j < highI; k = j, ++j)
		OffsetPoint(group, path, j, k);

	// reverse normals
	for (size_t i = highI; i > 0; --i)
		norms[i] = PointD(-norms[i - 1].x, -norms[i - 1].y);
	norms[0] = norms[highI];

	// do the line end cap
	if (deltaCallback64_)
		group_delta_ = deltaCallback64_(path, norms, highI, highI);

	if (std::fabs(group_delta_) <= floating_point_tolerance)
        path_out.emplace_back(path[highI]);
	else
	{
		switch (end_type)
		{
		case EndType::Butt:
			DoBevel(path, highI, highI);
			break;
		case EndType::Round:
			DoRound(path, highI, highI, PI);
			break;
		default:
			DoSquare(path, highI, highI);
			break;
		}
	}

	for (size_t j = highI -1, k = highI; j > 0; k = j, --j)
		OffsetPoint(group, path, j, k);
    solution->emplace_back(path_out);
}

void ClipperOffset::DoGroupOffset(Group& group)
{
	if (group.end_type == EndType::Polygon)
	{
		// a straight path (2 points) can now also be 'polygon' offset
		// where the ends will be treated as (180 deg.) joins
		double delta = (group.outer_sign != 0) ? delta_ : std::abs(delta_);
		group_delta_ = (group.is_reversed) ? -delta : delta;
	}
	else
		group_delta_ = std::abs(delta_);// *0.5;

	double abs_delta = std::fabs(group_delta_);
	join_type_	= group.join_type;

	if (group.join_type == JoinType::Round || group.end_type == EndType::Round)
		SetArcSteps(group_delta_);

	//double min_area = PI * Sqr(group_delta_);
	Paths64::const_iterator path_in_it = group.paths_in.cbegin();
	for ( ; path_in_it != group.paths_in.cend(); ++path_in_it)
	{
		if (group.IsStray(path_in_it - group.paths_in.cbegin())) continue;

		Path64::size_type pathLen = path_in_it->size();
		path_out.clear();
		BuildNormals(*path_in_it);

		if (pathLen == 1) // single point
		{
			double point_delta = (group.end_type == EndType::Polygon) ? delta_ : std::abs(delta_);

			if (deltaCallback64_)
			{
				point_delta = deltaCallback64_(*path_in_it, norms, 0, 0);
				group_delta_ = (group.is_reversed) ? -point_delta : point_delta;
				abs_delta = std::fabs(group_delta_);
			}

			// a point can only grow, so the caller's delta decides and not the group's orientation
			if (point_delta < 1) continue;

			const Point64& pt = (*path_in_it)[0];
			//single vertex so build a circle or square ...
			if (group.join_type == JoinType::Round)
			{
				// DoRound may have left the arc steps of another path's delta
				if (deltaCallback64_) SetArcSteps(group_delta_);

				double radius = abs_delta;
                size_t steps = steps_per_rad_ > 0 ? static_cast<size_t>(std::ceil(steps_per_rad_ * 2 * PI)) : 0; //#617
				path_out = Ellipse(pt, radius, radius, steps);
#ifdef USINGZ
				for (auto& p : path_out) p.z = pt.z;
#endif
			}
			else
			{
				int d = (int)std::ceil(abs_delta);
				Rect64 r = Rect64(pt.x - d, pt.y - d, pt.x + d, pt.y + d);
				path_out = r.AsPath();
#ifdef USINGZ
				for (auto& p : path_out) p.z = pt.z;
#endif
			}

            solution->emplace_back(path_out);
			continue;
		} // end of offsetting a single point

		EndType end_type = group.end_type;

		if ((pathLen == 2) && (end_type == EndType::Joined))
			end_type = (group.join_type == JoinType::Round) ? EndType::Round : EndType::Square;

		if (end_type == EndType::Polygon)
		{
			// a path without area can only grow, so a negative delta erodes it to nothing
			const bool flat = IsFlat(*path_in_it);

			if (flat && !deltaCallback64_ && delta_ < 0) continue;

			OffsetPolygon(group, *path_in_it);

			// a callback can grow some vertices and erode others, so keep only what grew
			if (flat && deltaCallback64_) KeepGrownRegions();
		}
		else if (end_type == EndType::Joined) OffsetOpenJoined(group, *path_in_it);
		else OffsetOpenPath(group, *path_in_it, end_type);
	}
}

#ifdef USINGZ
void ClipperOffset::ZCB(const Point64& bot1, const Point64& top1,
	const Point64& bot2, const Point64& top2, Point64& ip)
{
	if (bot1.z && ((bot1.z == bot2.z) || (bot1.z == top2.z))) ip.z = bot1.z;
	else if (bot2.z && (bot2.z == top1.z)) ip.z = bot2.z;
	else if (top1.z && (top1.z == top2.z)) ip.z = top1.z;
	else if (zCallback64_) zCallback64_(bot1, top1, bot2, top2, ip);
}
#endif

size_t ClipperOffset::CalcSolutionCapacity()
{
	size_t result = 0;
	for (const Group& g : groups_)
		result += (g.end_type == EndType::Joined) ? g.paths_in.size() * 2 : g.paths_in.size();
	return result;
}

void ClipperOffset::SetGroupOrientations()
{
	// a group's outers share the orientation of its largest closed path, which a hole touching
	// its outer or a stray sliver left by clipping can't outweigh. Flagging groups as 'reversed'
	// (negating delta etc.) is much more efficient than reversing every path.
	struct PathRef { std::pair<uint64_t, uint64_t> size; size_t group_idx; size_t path_idx; };
	std::vector<PathRef> paths;
	std::vector<Rect64> bounds;
	std::vector<size_t> group_ends; // where each group's paths end in paths
	std::vector<size_t> outer_refs(groups_.size()); // each group's largest path in paths
	bool has_ccw = false, has_cw = false, has_holes = false;

	for (size_t i = 0; i < groups_.size(); ++i)
	{
		Group& g = groups_[i];
		g.is_reversed = g.outer_sign < 0;
		g.is_stray.clear();
		has_ccw = has_ccw || (g.outer_sign > 0);
		has_cw = has_cw || (g.outer_sign < 0);

		for (size_t j = 0; j < g.areas.size(); ++j)
		{
			if (g.areas[j].sign == 0) continue;

			has_holes = has_holes || (g.areas[j].sign != g.outer_sign);

			if (j == g.outer_idx) outer_refs[i] = paths.size();

			paths.push_back({ g.areas[j].twice_magnitude, i, j });
			bounds.push_back(GetBounds(g.paths_in[j]));
		}

		group_ends.push_back(paths.size());
	}

	// only a group wound against another can hold that group's holes
	const bool is_mixed = has_ccw && has_cw;

	if (!is_mixed && !has_holes) return;

	// A group wound against another holds that group's holes when its largest path lies inside one
	// of that group's outers, and the innermost enclosing path decides, so a group inside another
	// group's hole stays an island. A hole that no outer at least as large encloses is stray and
	// contributes nothing, since offsetting it alone would leave the loops its concave joins add.
	// Each decision depends only on paths at least as large, so settling from the largest path down,
	// with a group's own largest path first among equals, settles everything in one pass whatever
	// the AddPaths order. Without mixed orientations no group turns into holes, so any order will do.
	std::vector<size_t> order(paths.size());
	std::iota(order.begin(), order.end(), size_t(0));

	if (is_mixed)
	{
		std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
			if (paths[a].size != paths[b].size) return paths[a].size > paths[b].size;

			return paths[a].path_idx == groups_[paths[a].group_idx].outer_idx &&
				paths[b].path_idx != groups_[paths[b].group_idx].outer_idx;
		});
	}

	auto path_of = [&](size_t i) -> const Path64& { return groups_[paths[i].group_idx].paths_in[paths[i].path_idx]; };
	auto sign_of = [&](size_t i) { return groups_[paths[i].group_idx].areas[paths[i].path_idx].sign; };
	auto is_outer = [&](size_t i) { return (sign_of(i) < 0) == groups_[paths[i].group_idx].is_reversed; };

	// the outers are active, so a hole looking for one skips the holes around it. Holes inside
	// their own group's outer don't need the tree at all.
	std::optional<BoundsTree> tree;

	auto get_tree = [&]() -> BoundsTree& {
		if (tree) return *tree;

		tree.emplace(bounds);

		for (size_t i = 0; i < paths.size(); ++i)
			tree->SetActive(i, is_outer(i));

		return *tree;
	};

	PolygonLocator locator(paths.size());
	std::vector<size_t> enclosers;

	auto locate_in = [&](size_t i) {
		return [&, i](const Point64& pt) { return locator.Locate(pt, path_of(i), i); };
	};

	for (size_t k : order)
	{
		const size_t group_idx = paths[k].group_idx;
		Group& g = groups_[group_idx];
		const Path64& path = path_of(k);
		const Rect64& path_bounds = bounds[k];

		if (paths[k].path_idx == g.outer_idx)
		{
			if (!is_mixed) continue;

			enclosers.clear();

			get_tree().Query([&](const Rect64& r) { return r.Contains(path_bounds); }, [&](size_t i) {
				if (paths[i].group_idx != group_idx && paths[i].size > paths[k].size &&
					!groups_[paths[i].group_idx].IsStray(paths[i].path_idx))
					enclosers.push_back(i);
			});

			// a heap, since only the smallest few enclosers are usually needed
			auto larger = [&](size_t a, size_t b) { return paths[a].size > paths[b].size; };
			std::make_heap(enclosers.begin(), enclosers.end(), larger);
			size_t innermost = paths.size();
			bool is_hole = false;

			while (!enclosers.empty())
			{
				std::pop_heap(enclosers.begin(), enclosers.end(), larger);
				const size_t i = enclosers.back();
				enclosers.pop_back();

				if (innermost < paths.size() && paths[i].size > paths[innermost].size) break;

				if (!IsInside(path, locate_in(i))) continue;

				innermost = i;

				// the enclosing path must be one of its group's outers, wound against this group
				if (is_outer(i) && sign_of(i) != g.outer_sign) is_hole = true;
			}

			if (!is_hole) continue;

			g.is_reversed = !g.is_reversed;

			for (size_t i = group_idx ? group_ends[group_idx - 1] : 0; i < group_ends[group_idx]; ++i)
				get_tree().SetActive(i, is_outer(i));

			continue;
		}

		if (is_outer(k)) continue;

		// a hole with a vertex inside an outer counts as enclosed, so one poking out still cuts it
		const size_t own_outer = outer_refs[group_idx];
		bool is_enclosed = is_outer(own_outer) && HasVertexInside(path, locate_in(own_outer));

		if (!is_enclosed)
		{
			get_tree().Query([&](const Rect64& r) { return !is_enclosed && r.Intersects(path_bounds); }, [&](size_t i) {
				if (paths[i].size >= paths[k].size) is_enclosed = HasVertexInside(path, locate_in(i));
			}, true);
		}

		if (is_enclosed) continue;

		if (g.is_stray.empty()) g.is_stray.resize(g.paths_in.size());

		g.is_stray[paths[k].path_idx] = true;
	}
}

void ClipperOffset::InitClipper(Clipper64& c)
{
	c.PreserveCollinear(preserve_collinear_);
#ifdef USINGZ
	auto fp = std::bind(&ClipperOffset::ZCB, this, std::placeholders::_1,
		std::placeholders::_2, std::placeholders::_3,
		std::placeholders::_4, std::placeholders::_5);
	c.SetZCallback(fp);
#endif
}

void ClipperOffset::KeepGrownRegions()
{
	// growth is always wound positively, so a positive union removes the eroded regions
	Clipper64 c;
	InitClipper(c);
	c.AddSubject(Paths64{ std::move(solution->back()) });
	solution->pop_back();
	Paths64 grown;
	c.Execute(ClipType::Union, FillRule::Positive, grown);

	for (Path64& path : grown)
		solution->push_back(std::move(path));
}

void ClipperOffset::ExecuteInternal(double delta)
{
	error_code_ = 0;
	if (groups_.size() == 0) return;
	solution->reserve(CalcSolutionCapacity());
	SetGroupOrientations();

	if (std::abs(delta) < 0.5) // ie: offset is insignificant
	{
		Paths64::size_type sol_size = 0;
		for (const Group& group : groups_) sol_size += group.paths_in.size();
		solution->reserve(sol_size);
		for (const Group& group : groups_)
		{
			// an open path is only as wide as its delta, so an insignificant delta leaves nothing
			if (group.end_type != EndType::Polygon) continue;

			for (size_t i = 0; i < group.paths_in.size(); ++i)
			{
				if (group.IsStray(i)) continue;

				solution->push_back(group.paths_in[i]);

				if (group.is_reversed) std::reverse(solution->back().begin(), solution->back().end());
			}
		}
	}
	else
	{

		temp_lim_ = (miter_limit_ <= 1) ?
			2.0 :
			2.0 / (miter_limit_ * miter_limit_);

		delta_ = delta;
		std::vector<Group>::iterator git;
		for (git = groups_.begin(); git != groups_.end(); ++git)
		{
			DoGroupOffset(*git);
			if (!error_code_) continue; // all OK
			solution->clear();
		}
	}

	if (!solution->size()) return;

	//clean up self-intersections ...
	Clipper64 c;
	InitClipper(c);
	c.ReverseSolution(reverse_solution_);
	c.AddSubject(*solution);
	// outers are wound positively, whatever the orientation of the input
	if (solution_tree)
		c.Execute(ClipType::Union, FillRule::Positive, *solution_tree);
	else
		c.Execute(ClipType::Union, FillRule::Positive, *solution);
}

void ClipperOffset::Execute(double delta, Paths64& paths64)
{
	paths64.clear();
	solution = &paths64;
	solution_tree = nullptr;
	ExecuteInternal(delta);
}


void ClipperOffset::Execute(double delta, PolyTree64& polytree)
{
	polytree.Clear();
	Paths64 paths;
	solution_tree = &polytree;
	solution = &paths;
	ExecuteInternal(delta);
	solution = nullptr;
}

void ClipperOffset::Execute(DeltaCallback64 delta_cb, Paths64& paths)
{
	deltaCallback64_ = delta_cb;
	Execute(1.0, paths);
}

} // namespace
