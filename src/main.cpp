#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cassert>
#include <sstream> 
#include <math.h>
#include <time.h>
#include <queue>

using namespace std;

using VertexIdx = size_t;
using FigureIdx = size_t;
using Decimal = double;

pair<int, int> MOVE = {1, 25};
pair<int, int> ROTATE = {26, 50};
pair<int, int> MIRROR_ALONG_CUT = {51, 75};
pair<int, int> ROTATE_BRIDGE = {76, 100};

struct Point
{
	Decimal x_ = 0.0;
	Decimal y_ = 0.0;

	static Decimal distance(const Point &p1, const Point &p2)
	{
		Decimal d1 = (p1.x_ - p2.x_) * (p1.x_ - p2.x_);
		Decimal d2 = (p1.y_ - p2.y_) * (p1.y_ - p2.y_);
		return  d1 + d2;
	}

	static Decimal discreteDistance(const Point &p1, const Point &p2)
	{
		Decimal d1 = (round(p1.x_) - round(p2.x_)) * (round(p1.x_) - round(p2.x_));
		Decimal d2 = (round(p1.y_) - round(p2.y_)) * (round(p1.y_) - round(p2.y_));
		return  d1 + d2;
	}


	bool operator==(const Point &other) const
  	{
  		return x_ == other.x_ && y_ == other.y_;
	}

	/// https://stackoverflow.com/questions/8954326/how-to-calculate-the-mirror-point-along-a-line
	Decimal distnaceToLine(const Point &lp1, const Point &lp2, Decimal &nA, Decimal &nB) const
	{
		const auto A = lp2.y_ - lp1.y_;
		const auto B = -(lp2.x_ - lp1.x_);
		const auto C = -A * lp1.x_ - B * lp1.y_;

		const auto M = sqrt(A * A + B * B);

		if(M == 0) return 0.0;

		nA = A / M;
		nB = B / M;
		const auto nC = C / M;
		const auto D = nA * x_ + nB * y_ + nC;

		return D;
	}

	Decimal distnaceToLine(const Point &lp1, const Point &lp2) const
	{
		Decimal nA;
		Decimal nB;
		return distnaceToLine(lp1, lp2, nA, nB);
	}

	Point mirrorAlongLine(const Point &lp1, const Point &lp2) const
	{
		Decimal nA;
		Decimal nB;

		auto D = distnaceToLine(lp1, lp2, nA, nB);
	//	cout << "D = " << D << endl;
		return {x_ - 2 * nA * D, y_ - 2 * nB * D};
	}
};
struct Rate
{
	Decimal restriction_ = 0;
	Decimal dislikes_ = 0;

	bool operator < (const Rate &other) const
	{
		if(restriction_ < other.restriction_) return true;
		else if(restriction_ == other.restriction_) return dislikes_ < other.dislikes_;
		return false;
	}
};
using RateTable = unordered_map<Point, Rate>;

namespace std {

	template <>
	struct hash<Point>
	{
		std::size_t operator()(const Point& p) const
		{
			using std::size_t;
			using std::hash;
			using std::string;

			// Compute individual hash values for first,
			// second and third and combine them using XOR
			// and bit shifting:

			return hash<Decimal>()(p.x_) ^ (hash<Decimal>()(p.y_) << 1);
		}
	};

	template <>
	struct hash<pair<VertexIdx, VertexIdx>>
	{
		std::size_t operator()(const pair<VertexIdx, VertexIdx>& p) const
		{
			using std::size_t;
			using std::hash;
			using std::string;

			// Compute individual hash values for first,
			// second and third and combine them using XOR
			// and bit shifting:

			return hash<VertexIdx>()(p.first) ^ (hash<VertexIdx>()(p.second) << 1);
		}
	};
}

struct Edge
{
	VertexIdx from_ = 0;
	VertexIdx to_ = 0;
};

vector<Point> readPoints(ifstream &s)
{
	vector<Point> points;
	char sink;
	while(true)
	{
		Point p;
		s >> sink;

		s >> p.x_;
		s >> sink;
		s >> p.y_;

		s >> sink;
		s >> sink;
		points.push_back(p);

	//	cout << p.x << " " << p.y << " " << sink << endl;

		if(sink == ']') break;
	}
	return points;
}

vector<Edge> readEdges(ifstream &s)
{
	vector<Edge> edges;
	char sink;
	while(true)
	{
		Edge e;
		s >> sink;

		s >> e.from_;
		s >> sink;
		s >> e.to_;

		s >> sink;
		s >> sink;
		edges.push_back(e);

	//	cout << e.from << " " << e.to << " " << sink << endl;

		if(sink == ']') break;
	}
	return edges;
}

bool isPointLiesToTheLeft(const Point &point,
						  Point linePoint1,
						  Point linePoint2)
{
	if(linePoint1.y_ > linePoint2.y_) swap(linePoint1, linePoint2);

	if(point.y_ < linePoint1.y_ || point.y_ > linePoint2.y_) return false;

	/*
		k * x1 + b = y1
		k * x2 + b = y2
		k * (x1 - x2) = y1 - y2
		k = (y1 - y2) / (x1 - x2)
		b = y1 - k * x1
	*/

	if(linePoint1.x_ == linePoint2.x_)
	{
		return point.x_ <= linePoint1.x_;
	}
	else
	{
		auto k = (linePoint1.y_ - linePoint2.y_)
				 / (linePoint1.x_ - linePoint2.x_);
		auto b = linePoint1.y_ - k * linePoint1.x_;
		auto y = k * point.x_ + b;
		if(linePoint1.x_ < linePoint2.x_)
		{
			return point.y_ <= y;
		}
		else
		{
			return point.y_ >= y;
		}

	}
	assert(false);
}

struct StaticFigureData
{
	vector<Edge> edgeList_;
	unordered_map<VertexIdx, unordered_set<VertexIdx>> adjacencyList_;
	unordered_map<VertexIdx, unordered_set<VertexIdx>> transitiveClosure_;
	unordered_map<VertexIdx, pair<unordered_set<VertexIdx>, unordered_set<VertexIdx>>> bridges_;
	unordered_map<pair<VertexIdx, VertexIdx>, pair<unordered_set<VertexIdx>, unordered_set<VertexIdx>>> cuts_;
};

struct Figure
{
	unordered_set<VertexIdx> reachableVertexes(const VertexIdx source,
											   const VertexIdx direction,
											   const unordered_set<VertexIdx> &ignored)
	{
		assert(data_);
		assert(data_->adjacencyList_.size() == vertices_.size());

		unordered_set<VertexIdx> visited;
		visited.insert(source);

		queue<VertexIdx> q;
		q.push(direction);


		while(!q.empty())
		{
			const auto v = q.front(); q.pop();
			visited.insert(v);

			for(const auto av : data_->adjacencyList_[v])
			{
				if(visited.count(av) == 0 && ignored.count(av) == 0) q.push(av);
			}
		}

		visited.erase(source);

		return visited;
	}

	vector<unordered_set<VertexIdx>> makePartitions(const unordered_set<VertexIdx> &ignored)
	{
		assert(data_);

		unordered_set<VertexIdx> unvisited;
		for(VertexIdx vi = 0; vi < vertices_.size(); ++vi)
		{
			if(ignored.count(vi) == 0) unvisited.insert(vi);
		}


		vector<unordered_set<VertexIdx>> partitions;
		while(!unvisited.empty())
		{
			unordered_set<VertexIdx> partition;
			queue<VertexIdx> q;
			q.push(*(unvisited.begin()));

			while(!q.empty())
			{
				auto vi = q.front(); q.pop();
				partition.insert(vi);
				unvisited.erase(vi);
				for(const auto av : data_->adjacencyList_[vi])
				{
					if(unvisited.count(av) > 0 && ignored.count(av) == 0) q.push(av);
				}
			}

			partitions.push_back(partition);
		}

		return partitions;
	}

	void init(const vector<Edge> &edges, const vector<Point> &vertices)
	{		
		for(const auto &v: vertices)
		{
			vertices_.push_back(v);
		}

		data_ = new StaticFigureData();

		data_->edgeList_ = edges;
		for(const auto &e: edges)
		{
			data_->adjacencyList_[e.from_].insert(e.to_);
			data_->adjacencyList_[e.to_].insert(e.from_);
		}

		cout << "bridges:";
		for(VertexIdx vid = 0; vid < vertices.size(); ++vid)
		{
			const auto &al = data_->adjacencyList_[vid];
			if(al.size() >= 2)
			{
				int size = 0;
				unordered_set<VertexIdx> partition1;
				unordered_set<VertexIdx> partition2;
				bool isBridge = true;
				for(VertexIdx av : al)
				{
					auto rv = reachableVertexes(vid, av, {});
					if(partition1.empty())
					{
						partition1 = rv;
					}
					else if(partition2.empty() && rv != partition1)
					{
						partition2 = rv;
						isBridge = vertices.size() - 1 == partition1.size() + partition2.size();
					}
					else
					{
						isBridge = (rv == partition1) || (rv == partition2);
					}
					if(!isBridge) break;
				}
	
				if(isBridge && !partition1.empty() && !partition2.empty())
				{
					data_->bridges_[vid] = {partition1, partition2};
					cout << " " << vid;
				}
				
			}
		}
		cout << endl;
		
		cout << "cuts:";
		for(VertexIdx vid1 = 0; vid1 < vertices.size(); ++vid1)
		{
			if(data_->bridges_.find(vid1) != data_->bridges_.end()) continue;
			for(VertexIdx vid2 = vid1 + 1; vid2 < vertices.size(); ++vid2)
			{
				if(data_->adjacencyList_[vid1].count(vid2) != 0) continue;
				if(data_->bridges_.find(vid2) != data_->bridges_.end()) continue;

				auto partitions = makePartitions({vid1, vid2});
				if(partitions.size() == 2) 
				{
					cout << " {" << vid1 << ", " << vid2 << "}";
					data_->cuts_[{vid1, vid2}] = {partitions[0], partitions[1]};
				}
			}
		}
		cout << endl;

		updateCenter();

		if(data_->bridges_.empty())
		{
			MOVE = {1, 33};
			ROTATE = {34, 67};
			MIRROR_ALONG_CUT = {68, 100};

			ROTATE_BRIDGE = {101, 101};
		}
		else if(data_->cuts_.empty())
		{
			MOVE = {1, 33};
			ROTATE = {34, 67};
			ROTATE_BRIDGE = {68, 100};

			MIRROR_ALONG_CUT = {101, 101};
		}
		
		if(data_->bridges_.empty() && data_->cuts_.empty())
		{
			MOVE = {1, 50};
			ROTATE = {51, 100};

			ROTATE_BRIDGE = {101, 101};
			MIRROR_ALONG_CUT = {101, 101};
		}
	}

	void moveVertex(const VertexIdx idx, const Decimal dx, const Decimal dy)
	{
		auto &v = vertices_[idx];
		v.x_ += dx;
		v.y_ += dy;
	}

	void move(const Decimal dx, const Decimal dy)
	{
		for(auto &v : vertices_)
		{
			v.x_ += dx;
			v.y_ += dy;
		}
	}

	// 1
	// static constexpr Decimal SIN = 0.01745240644;
	// static constexpr Decimal COS = 0.99984769516;

	// 45
	// static constexpr Decimal SIN = 0.70710678119;
	// static constexpr Decimal COS = 0.70710678119;

	// 5
	// static constexpr Decimal SIN = 0.08715574275;
	// static constexpr Decimal COS = 0.99619469809;

	static constexpr Decimal PI = 3.14159265;

	static constexpr int LEFT = 0;
	static constexpr int RIGHT = 1;
	void rotateAround(const VertexIdx centerIdx, 
					  const unordered_set<VertexIdx> &rotatedVertices,
					  const int direction,
					  const Decimal degree)
	{
		const auto center = vertices_[centerIdx];
		for(auto vi : rotatedVertices)
		{
			vertices_[vi].x_ -= center.x_;
			vertices_[vi].y_ -= center.y_;

			if(direction == LEFT) rotateLeft(vi, degree);
			else if(direction == RIGHT) rotateRight(vi, degree);
			else assert(direction == LEFT || direction == RIGHT);

			vertices_[vi].x_ += center.x_;
			vertices_[vi].y_ += center.y_;
		}		
	}

	static Decimal degree(Point from, Point to, const Point &base)
	{
		from.x_ -= base.x_;
		from.y_ -= base.y_;
		to.x_ -= base.x_;
		to.y_ -= base.y_;

		Decimal radians1 = atan((from.x_ - to.x_) / from.y_);
		Decimal radians2 = atan((from.y_ - to.y_) / from.x_); 

		cout << "degree1 = " << (radians1 * 180.0 / PI) << " degree2 = " << (radians2 * 180.0 / PI) << endl;

		return radians1 * 180.0 / PI;
	}

	void rotateLeft(const VertexIdx idx, const Decimal degree)
	{
		auto &v = vertices_[idx];
		const Decimal x = v.x_;
		const Decimal y = v.y_;
		const auto radians = degree * PI / 180.0;
		v.x_ = x * cos(radians) - y * sin(radians);
		v.y_ = y * cos(radians) + x * sin(radians);
	}

	void rotateRight(const VertexIdx idx, const Decimal degree)
	{
		auto &v = vertices_[idx];
		const Decimal x = v.x_;
		const Decimal y = v.y_;
		const auto radians = degree * PI / 180.0;
		v.x_ = x * cos(radians) + y * sin(radians);
		v.y_ = y * cos(radians) - x * sin(radians);
	}

	void rotateLeft(const Decimal degree)
	{
		Decimal x;
		Decimal y;
		const auto radians = degree * PI / 180.0;
		for(auto &v : vertices_)
		{
			x = v.x_;
			y = v.y_;
			v.x_ = x * cos(radians) - y * sin(radians);
			v.y_ = y * cos(radians) + x * sin(radians);
		}
	}

	void rotateRight(const Decimal degree)
	{
		Decimal x;
		Decimal y;
		const auto radians = degree * PI / 180.0;
		for(auto &v : vertices_)
		{
			x = v.x_;
			y = v.y_;
			v.x_ = x * cos(radians) + y * sin(radians);
			v.y_ = y * cos(radians) - x * sin(radians);
		}
	}

	string toJSONString() const
	{
		string json = "{\"vertices\":[";
		for(int i = 0; i < vertices_.size(); ++i)
		{
			const auto &v = vertices_[i];
			json += "[" + to_string(int(round(v.x_))) + "," + to_string(int(round(v.y_))) + "]";
			if(i != vertices_.size() - 1) json += ",";
		}
		json += "]}";

		return json;
	}

	void updateCenter()
	{
		center_ = {0, 0};
		for(auto v : vertices_)
		{
			center_.x_ += v.x_;
			center_.y_ += v.y_;
		}
		center_.x_ /= vertices_.size();	
		center_.y_ /= vertices_.size();	
	}


	void roundAllVertices()
	{
		for(auto &v : vertices_)
		{
			v.x_ = round(v.x_);
			v.y_ = round(v.y_);
		}
	}


	vector<Point> vertices_;
	StaticFigureData *data_;

	Point center_;
};

struct Problem
{
	const string solutionFile_;
	const string svgFile_;

	Problem(const string file, const string &solutionFile, const string &svgFile) :
		solutionFile_(solutionFile),
		svgFile_(svgFile)
	{
		ifstream input(file);

		char sink;
		while(sink != 'h')
		{
			input >> sink;
		}

		input.seekg(6, ios_base::cur);
		hole_ = readPoints(input);

		input.seekg(11, ios_base::cur);
		input >> epsilon_;

		input.seekg(20, ios_base::cur);
		auto edges = readEdges(input);

		input.seekg(13, ios_base::cur);
		auto vertices = readPoints(input);

		initFigure_.init(edges, vertices);
	}

	void move(Decimal dx, Decimal dy)
	{
		for(auto &v : hole_)
		{
			v.x_ += dx;
			v.y_ += dy;
		}
		initFigure_.move(dx, dy);
	}

	bool isVertexInsideHole(const Point &point) const
	{
		int counter = 0;
		for(int i = 0; i < hole_.size(); ++i)
		{
			const auto &lp1 = hole_[i];
			const auto &lp2 = hole_[(i + 1) % hole_.size()];
			if(isPointLiesToTheLeft(point, lp1, lp2)) ++counter;
		}
		return counter % 2 == 1;
	}

	bool isSegmentsIntersect1(const Point &s1p1, const Point &s1p2,
							 const Point &s2p1, const Point &s2p2) const
	{
		const bool b1 = isPointLiesToTheLeft(s1p1, s2p1, s2p2);
		const bool b2 = isPointLiesToTheLeft(s1p2, s2p1, s2p2);

		const bool b3 = isPointLiesToTheLeft(s2p1, s1p1, s1p2);
		const bool b4 = isPointLiesToTheLeft(s2p2, s1p1, s1p2);
		return (b1 && !b2) || (!b1 && b2) || (!b3 && b4) || (!b3 && b4);
	}

/*
!!! https://e-maxx.ru/algo/segments_intersection_checking

inline int area (pt a, pt b, pt c) {
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
 
inline bool intersect_1 (int a, int b, int c, int d) {
	if (a > b)  swap (a, b);
	if (c > d)  swap (c, d);
	return max(a,c) <= min(b,d);
}
 
bool intersect (pt a, pt b, pt c, pt d) {
	return intersect_1 (a.x, b.x, c.x, d.x)
		&& intersect_1 (a.y, b.y, c.y, d.y)
		&& area(a,b,c) * area(a,b,d) <= 0
		&& area(c,d,a) * area(c,d,b) <= 0;
}
*/
	Decimal area(const Point &a, const Point &b, const Point &c) const
	{
		return (b.x_ - a.x_) * (c.y_ - a.y_) - (b.y_ - a.y_) * (c.x_ - a.x_);
	}

	bool intersect_1 (Decimal a, Decimal b, Decimal c, Decimal d) const
	{
		if (a > b)  swap(a, b);
		if (c > d)  swap(c, d);
		return max(a,c) < min(b,d);
	}
 

	bool isSegmentsIntersect2(const Point &s1p1, const Point &s1p2,
							 const Point &s2p1, const Point &s2p2) const
	{
			return intersect_1 (s1p1.x_, s1p2.x_, s2p1.x_, s2p2.x_)
				&& intersect_1 (s1p1.y_, s1p2.y_, s2p1.y_, s2p2.y_)
				&& area(s1p1,s1p2,s2p1) * area(s1p1,s1p2,s2p2) < 0
				&& area(s2p1,s2p2,s1p1) * area(s2p1,s2p2,s1p2) < 0;
	}

	Rate fitRate(const Figure &figure, RateTable *rateTable, const bool isVerbose) const
	{
		Rate rate{0.0, 0.0};
		for(const auto &e : figure.data_->edgeList_)
		{
			const auto &p1 = figure.vertices_[e.from_];
			const auto &p2 = figure.vertices_[e.to_];

			if(isVerbose) cout << e.from_ << " " << e.to_ << ":" << endl;

			for(int i = 0; i < hole_.size(); ++i)
			{
				const auto &hp1 = hole_[i];
				const auto &hp2 = hole_[(i + 1) % hole_.size()];

				const bool isP1InsideHole = isVertexInsideHole(p1);
				const bool isP2InsideHole = isVertexInsideHole(p2);

				if(!isP1InsideHole)
				{
					auto r = abs(p1.distnaceToLine(hp1, hp2));
					if(rateTable) (*rateTable)[p1].restriction_ += r;
					rate.restriction_ += r;

					if(isVerbose) cout << "!isP1InsideHole " << i << " " << ((i + 1) % hole_.size()) << " +" << r << " to " << e.from_ << endl;
				} 
				if(!isP2InsideHole)
				{
					auto r = abs(p1.distnaceToLine(hp1, hp2));
					if(rateTable) (*rateTable)[p2].restriction_ += r;
					rate.restriction_ += r;

					if(isVerbose) cout << "!isP2InsideHole " << i << " " << ((i + 1) % hole_.size()) << " +" << r << " to " << e.to_ << endl;
				}
				if(isSegmentsIntersect2(p1, p2, hp1, hp2))
				{
					auto r1 = abs(p1.distnaceToLine(hp1, hp2));
					if(rateTable) (*rateTable)[p1].restriction_ += r1;
					rate.restriction_ += r1;

					if(isVerbose) cout << "isSegmentsIntersect2 " << i << " " << ((i + 1) % hole_.size()) << " +" << r1 << " to " << e.from_ << endl;

					auto r2 = abs(p1.distnaceToLine(hp1, hp2));
					if(rateTable) (*rateTable)[p2].restriction_ += r2;
					rate.restriction_ += r2;

					if(isVerbose) cout << "isSegmentsIntersect2 " << i << " " << ((i + 1) % hole_.size()) << " +" << r2 << " to " << e.from_ << endl;
				}
			}
		}


		for(VertexIdx hvi = 0; hvi < hole_.size(); ++hvi)
		{
			Decimal minDistance = 10000000.0;
			for(VertexIdx fvi = 0; fvi < figure.vertices_.size(); ++fvi)
			{
				Decimal d = Point::distance(hole_[hvi], figure.vertices_[fvi]);
				if(d < minDistance) minDistance = d;
			}
			if(rateTable) (*rateTable)[hole_[hvi]].dislikes_ += minDistance;
			rate.dislikes_ += minDistance;
		}

		return rate;
	}

	bool isRigidityRetained(const Figure &figure, const bool verbose) const
	{
	//	if(rate >= 1000.0) return false;

		Decimal epsilon = epsilon_ / 1000000.0;
		for(int i = 0; i < initFigure_.data_->edgeList_.size(); ++i)
		{
			const auto &e = figure.data_->edgeList_[i];
			const auto &p1 = figure.vertices_[e.from_];
			const auto &p2 = figure.vertices_[e.to_];

			const auto &ie = initFigure_.data_->edgeList_[i];
			const auto &ip1 = initFigure_.vertices_[ie.from_];
			const auto &ip2 = initFigure_.vertices_[ie.to_];
			
			const Decimal d1 = Point::discreteDistance(p1, p2);
			const Decimal d2 = Point::discreteDistance(ip1, ip2);
			const Decimal t = abs(d1 / d2 - 1.0);

			if(verbose)
			{
			cout 
				<< " i = "<< i
				<< " from = " << e.from_ << " to = " << e.to_ << " " 
				<< " p1.x = " << p1.x_ << " p1.y = " << p1.y_
				<< " p2.x = " << p2.x_ << " p2.y = " << p2.y_
				<< " ip1.x = " << ip1.x_ << " ip1.y = " << ip1.y_
				<< " ip2.x = " << ip2.x_ << " ip2.y = " << ip2.y_
				<< " d1 = " << d1 << " d2 = " << d2 
				<< " t = " << t << " epsilon = " << epsilon << endl;
			}
			if(t > epsilon)
			{	
				return false;
			}
		}
		return true;
	}

	vector<Point> hole_;
	Decimal epsilon_ = 0.0;
	Figure initFigure_;
};

struct Solver
{
	static string makeSVGResult(const Problem &problem,
 					 	  const Figure &solution,
 						  const Decimal scale,
 						  const RateTable &rateTable)
	{
		ostringstream svg;

		svg << "<svg>";
		for(int i = 0; i < problem.hole_.size(); ++i)
		{
			const auto hp1 = problem.hole_[i];
			const auto hp2 = problem.hole_[(i + 1) % problem.hole_.size()];
			svg << "<line x1=\"";
			svg << hp1.x_ * scale;
			svg << "\" y1=\"";
			svg << hp1.y_ * scale;
			svg << "\" x2=\"";
			svg << hp2.x_ * scale;
			svg << "\" y2=\"";
			svg << hp2.y_ * scale;
			svg << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />";

			svg << "<text x=\"";
			svg << hp1.x_ * scale;
			svg << "\" y=\"";
			svg << hp1.y_ * scale;
			svg << "\" fill=\"black\">";
			svg << "#"<< i;
			svg << "</text>";
		}
		for(const auto &e : problem.initFigure_.data_->edgeList_)
		{
			const auto vp1 = problem.initFigure_.vertices_[e.from_];
			const auto vp2 = problem.initFigure_.vertices_[e.to_];
			svg << "<line x1=\"";
			svg << vp1.x_ * scale;
			svg << "\" y1=\"";
			svg << vp1.y_ * scale;
			svg << "\" x2=\"";
			svg << vp2.x_ * scale;
			svg << "\" y2=\"";
			svg << vp2.y_ * scale;
			svg << "\" style=\"stroke:rgb(255,0,0);stroke-width:4\" />";
		}
		for(const auto &e : solution.data_->edgeList_)
		{
			const auto vp1 = solution.vertices_[e.from_];
			const auto vp2 = solution.vertices_[e.to_];
			svg << "<line x1=\"";
			svg << vp1.x_ * scale;
			svg << "\" y1=\"";
			svg << vp1.y_ * scale;
			svg << "\" x2=\"";
			svg << vp2.x_ * scale;
			svg << "\" y2=\"";
			svg << vp2.y_ * scale;
			svg << "\" style=\"stroke:rgb(0,255,0);stroke-width:2\" />";
		}

		for(VertexIdx vi = 0; vi < solution.vertices_.size(); ++vi)
		{
			const auto p = solution.vertices_[vi];
			svg << "<text x=\"";
			svg << p.x_ * scale;
			svg << "\" y=\"";
			svg << p.y_ * scale;
			svg << "\" fill=\"blue\">";
			svg << "#"<< vi;
			svg << "</text>";
		}

		for(auto &p : rateTable)
		{
			// <text x="0" y="15" fill="red">I love SVG!</text>
			svg << "<text x=\"";
			svg << p.first.x_ * scale;
			svg << "\" y=\"";
			svg << p.first.y_ * scale;
			svg << "\" fill=\"blue\">";
			svg << "___(";
			svg << int(round(p.first.x_));
			svg << ",";
			svg << int(round(p.first.y_));
			svg << ")=";
			svg << int(round(p.second.restriction_));
			svg << ", ";
			svg << int(round(p.second.dislikes_));
			svg << "</text>";
		}
		svg << "</svg>";

		return svg.str();
	}

	static void saveSolutionToSVG(const Problem &p, const string &filePath, const Figure &f, const RateTable &rt)
	{
		ofstream htmlStream(filePath, std::ofstream::out);
		htmlStream << makeSVGResult(p, f, 10.0, rt);
		htmlStream.close();
	}

	static void saveSolutionToJSON(const Problem &p, const Figure &f)
	{
		ofstream solutionStream(p.solutionFile_, std::ofstream::out);
		solutionStream << f.toJSONString();
		solutionStream.close();
	}

	Decimal epsilon_;
	int populationSize_;
	int generationCount_;

	Solver(const Decimal epsilon, const int populationSize, const int generationCount) :
		epsilon_(epsilon / 1000000),
		populationSize_(populationSize),
		generationCount_(generationCount)
	{
	}

	void mutate(Figure &figure, const Problem &problem) const
	{
		assert(figure.vertices_.size() > 0);		

		int decision = rand() % 100 + 1;

		if(MOVE.first <= decision && decision <= MOVE.second)
		{
			VertexIdx randomIdx = rand() % figure.vertices_.size();

			Decimal dx = rand() % 21;
			dx = (rand() % 2 == 0) ? dx : -dx;
			Decimal dy = rand() % 21;
			dy = (rand() % 2 == 0) ? dy : -dy;
			figure.move(dx, dy);
			return;
		}


		static Figure oldFigure = figure;		

		if(MIRROR_ALONG_CUT.first <= decision && decision <= MIRROR_ALONG_CUT.second)
		{
			const auto &cuts = problem.initFigure_.data_->cuts_;
			auto randomCut = cuts.begin();
			for(int i = rand() % cuts.size(); i > 0; --i, ++randomCut);

			unordered_set<VertexIdx> verticeIdxsForMirroring =
														rand() % 2 == 0
														? randomCut->second.first
														: randomCut->second.second;
			const Point p1 = figure.vertices_[randomCut->first.first];
			const Point p2 = figure.vertices_[randomCut->first.second];
			for(const VertexIdx vi : verticeIdxsForMirroring)
			{
				figure.vertices_[vi] = figure.vertices_[vi].mirrorAlongLine(p1, p2);
			}

		}
		else if(ROTATE.first <= decision && decision <= ROTATE.second)
		{
			figure.updateCenter();
			figure.move(-round(figure.center_.x_), -round(figure.center_.y_));

			if(rand() % 2) figure.rotateLeft(rand() % 45 + 1);
			else figure.rotateRight(rand() % 45 + 1);

			figure.move(round(figure.center_.x_), round(figure.center_.y_));
		}
		else if(ROTATE_BRIDGE.first <= decision && decision <= ROTATE_BRIDGE.second)
		{
			const auto &bridges = problem.initFigure_.data_->bridges_;
			if(bridges.empty()) return;

			auto randomBridge = bridges.begin();
			for(int i = rand() % bridges.size(); i > 0; ++randomBridge, --i);

			const VertexIdx randomBridgeIdx = randomBridge->first;
			const unordered_set<VertexIdx> randomBridgeSide = 
				rand() % 2 == 0
				? randomBridge->second.first
				: randomBridge->second.second; 

			figure.rotateAround(randomBridgeIdx,
							    randomBridgeSide,
							    rand() % 2,
							    rand() % 45 + 1);
		}

	//	figure.roundAllVertices();
		if(!problem.isRigidityRetained(figure, false)) figure = oldFigure;
	}

	void crossover(const Figure &parent1, const Figure &parent2,
				   Figure &child1, Figure &child2) const
	{
		assert(parent1.vertices_.size() == parent2.vertices_.size());
		assert(parent1.vertices_.size() == child1.vertices_.size());
		assert(parent1.vertices_.size() == child2.vertices_.size());

		if(parent1.data_->cuts_.empty() || parent2.data_->cuts_.empty())
		{
			child1 = parent1;
			child2 = parent2;
			return;
		}

		auto randomCut = parent1.data_->cuts_.begin();
		for(int i = rand() % parent1.data_->cuts_.size(); i > 0; --i, ++randomCut);

		for(const VertexIdx vi : randomCut->second.first)
		{
			child1.vertices_[vi] = parent1.vertices_[vi];
			child2.vertices_[vi] = parent2.vertices_[vi];
		}

		for(const VertexIdx vi : randomCut->second.second)
		{
			child1.vertices_[vi] = parent2.vertices_[vi];
			child2.vertices_[vi] = parent1.vertices_[vi];
		}

		child1.vertices_[randomCut->first.first] = parent1.vertices_[randomCut->first.first];
		child1.vertices_[randomCut->first.second] = parent1.vertices_[randomCut->first.second];

		child2.vertices_[randomCut->first.first] = parent1.vertices_[randomCut->first.first];
		child2.vertices_[randomCut->first.second] = parent1.vertices_[randomCut->first.second];
	}

	Figure solve(const Problem &problem) const
	{
		vector<Figure> population(populationSize_, problem.initFigure_);
		vector<Figure> prevBestPopulation(populationSize_);
		vector<Rate> fitRates(population.size());

		vector<FigureIdx> populationIdxs(population.size());
		for(FigureIdx i = 0; i < populationIdxs.size(); ++i)
		{
			populationIdxs[i] = i;
		}

		Rate prevBestRate {1000000000, 1000000000};
		int noFitDecreaseCounter = 0;
		for(int n = 0; n < generationCount_; ++n)
		{
			for(VertexIdx i = 0; i < population.size(); ++i)
			{
				mutate(population[i], problem);
				mutate(population[i], problem);
				mutate(population[i], problem);
				mutate(population[i], problem);
				mutate(population[i], problem);

				mutate(population[i], problem);
				mutate(population[i], problem);
				mutate(population[i], problem);
				mutate(population[i], problem);
				mutate(population[i], problem);
			}

			for(VertexIdx i = 0; i < fitRates.size(); ++i)
			{
				fitRates[i] = problem.fitRate(population[i], nullptr, false);
			}	

			sort(populationIdxs.begin(),
				 populationIdxs.end(),
				 [&fitRates](const FigureIdx &fi1, const FigureIdx &fi2)
				 {
				 	return fitRates[fi1] < fitRates[fi2];
				 });

			const FigureIdx currentBest = populationIdxs[0];
		//	if(n % 100 == 0) cout<< n << " " << prevBestRate.restriction_ << " " << fitRates[currentBest].restriction_ << endl;

			if(fitRates[currentBest] < prevBestRate)
			{
				const auto &bestFigure = population[currentBest];

				RateTable rateTable;
				problem.fitRate(bestFigure, &rateTable, true);
				cout << endl;
				for(auto p : rateTable)
				{
					cout << p.first.x_ << " " << p.first.y_ << " -> " << p.second.restriction_ << ", " << p.second.dislikes_ << endl;
				}
				cout << endl;

				const string svgFile =  "../gifs/" + to_string(n) + ".svg";
				//saveSolutionToSVG(problem, problem.svgFile_, population[populationIdxs[0]], rateTable);	
				saveSolutionToSVG(problem, svgFile, population[populationIdxs[0]], rateTable);	

				prevBestRate = fitRates[currentBest];

				cout << bestFigure.toJSONString() << endl;
				auto isRigidityRetained = problem.isRigidityRetained(bestFigure, true);
				cout << "isRigidityRetained = " << isRigidityRetained << endl;

				prevBestPopulation = population;
				noFitDecreaseCounter = 0;

				if(prevBestRate.restriction_ == 0.0 && isRigidityRetained)					
				{
					saveSolutionToJSON(problem, population[populationIdxs[0]]);

				}
			}
			else
			{
				++noFitDecreaseCounter;
			}

			if(noFitDecreaseCounter == 100)
			{
				cout << "!!! RESET !!!" << endl;
				population = prevBestPopulation;
				noFitDecreaseCounter = 0;
			}

			for(int i = populationIdxs.size() / 2; i < populationIdxs.size() - 1; i += 2)
			{
				const VertexIdx randomParentIdx1 = populationIdxs[rand() % (populationIdxs.size() / 2)];
				const VertexIdx randomParentIdx2 = populationIdxs[rand() % (populationIdxs.size() / 2)];

				const VertexIdx childIdx1 = populationIdxs[i];
				const VertexIdx childIdx2 = populationIdxs[i + 1];

				crossover(population[randomParentIdx1], population[randomParentIdx2],
						  population[childIdx1], population[childIdx2]);	
			}			
		}		

		return population[populationIdxs[0]];
	}

	Figure solveByTrivialRotation(const Problem &problem) const
	{
	//	if(problem.hole_.size() != problem.initFigure_.vertices_.size()) return Figure();

		const auto &hole = problem.hole_;
		const Figure &figure = problem.initFigure_;
		Figure candidate = problem.initFigure_;

		unordered_set<VertexIdx> allVerticesIdxs;
		for(VertexIdx i = 0; i < figure.vertices_.size(); ++i) allVerticesIdxs.insert(i);

		Decimal epsilon = problem.epsilon_ / 1000000.0;
		for(VertexIdx hvi = 0; hvi < hole.size(); ++hvi)
		{
			const Point hp = hole[hvi];
			const Point hap1 = hole[(hvi - 1) % hole.size()];
			const Point hap2 = hole[(hvi + 1) % hole.size()];

			const Decimal hd1 = Point::distance(hp, hap1);
			const Decimal hd2 = Point::distance(hp, hap2);

		//	cout << hvi << " " << ((hvi - 1) % hole.size()) << " " << ((hvi + 1) % hole.size()) << endl;
			for(VertexIdx fvi = 0; fvi < figure.vertices_.size(); ++fvi)
			{
				const auto &al = figure.data_->adjacencyList_[fvi]; 
				if(al.size() != 2) continue;

				const Point fp = figure.vertices_[fvi];
				auto i = al.begin();
				const VertexIdx fapi1 = *i;
				const VertexIdx fapi2 = *(++i);
				const Point fap1 = figure.vertices_[fapi1];
				const Point fap2 = figure.vertices_[fapi2];

				const Decimal fd1 = Point::distance(fp, fap1);
				const Decimal fd2 = Point::distance(fp, fap2);

				cout << hd1 << " " << hd2 << endl;
				cout << fd1 << " " << fd2 << endl;

				bool chekCandidate = false;
				if(abs(fd1 / hd1 - 1) <= epsilon  && abs(fd2 / hd2 - 1) <= epsilon)
				{
					candidate = figure;
					candidate.move(hp.x_ - fp.x_, hp.y_ - fp.y_);
					auto degree = Figure::degree(candidate.vertices_[fapi1], hap1, fp);

					cout << "points:" << endl;
					cout << fp.x_ << " " << fp.y_ << endl;
					cout << candidate.vertices_[fapi1].x_ << " " << candidate.vertices_[fapi1].y_ << " " << hap1.x_ << " " << hap1.y_ << endl;
					cout << "degree " << degree << endl;
					candidate.rotateAround(fvi, allVerticesIdxs, Figure::LEFT, degree);
					chekCandidate = true;
				}
				else if(abs(fd1 / hd2 - 1) <= epsilon  && abs(fd1 / hd2 - 1) <= epsilon)
				{
					candidate = figure;
					candidate.move(hp.x_ - fp.x_, hp.y_ - fp.y_);
					auto degree = Figure::degree(candidate.vertices_[fapi1], hap2, fp);
					cout << "points:" << endl;
					cout << fp.x_ << " " << fp.y_ << endl;
					cout << candidate.vertices_[fapi1].x_ << " " << candidate.vertices_[fapi1].y_ << " " << hap2.x_ << " " << hap2.y_ << endl;
					cout << "degree " << degree << endl;
					candidate.rotateAround(fvi, allVerticesIdxs, Figure::LEFT, degree);
					chekCandidate = true;
				}

				if(chekCandidate)
				{
					auto rate = problem.fitRate(candidate, nullptr, false);
					auto isRigidityRetained = problem.isRigidityRetained(candidate, true);

					cout << isRigidityRetained << " " << rate.dislikes_ << endl;

					if(isRigidityRetained && rate.dislikes_ == 0)
					{
						 saveSolutionToSVG(problem, problem.svgFile_, candidate, RateTable());	
						 saveSolutionToJSON(problem, candidate);
						 return candidate;
					}
				}
			}
		}

		
		return Figure();
	}
};


int main(int argc, char* argv[])
{
	srand(time(0));

	cout << argc;
	for(int i = 0; i < argc; ++i)
	{
		cout << " " << argv[i];
	}
	cout << endl;

	Problem problem(string("../problems/") + argv[1] + ".problem",
					string("../solutions/") + argv[1] + ".solution",
					string("../solutions/") + argv[1] + ".svg");
		
	Solver solver(problem.epsilon_, 1000, 10000);
	auto solution = solver.solve(problem);
	//auto solution = solver.solveByTrivialRotation(problem);

	cout << "final solution:" << endl << solution.toJSONString() << endl;

	return 0;
}