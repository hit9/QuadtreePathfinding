// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_QUADTREE_MAP_HPP
#define QDPF_INTERNAL_QUADTREE_MAP_HPP

#include <functional> // for std::function

#include "base.hpp"
#include "graph.hpp"
#include "Quadtree-hpp/Quadtree.hpp"

// QuadtreeMap
// ~~~~~~~~~~~
// 1. 2d grid map maintained by a quadtree.
// 2. It's **relateless** to agent sizes and terrains.

namespace qdpf
{
	namespace internal
	{

		// ObstacleChecker is the type of the function that returns true if the given
		// cell (x,y) is an obstacle.
		using ObstacleChecker = std::function<bool(int x, int y)>;

		// DistanceCalculator calculates the distance between cell (x1,y1) and (x2,y2);
		using DistanceCalculator = std::function<int(int x1, int y1, int x2, int y2)>;

		// StepFunction specifics a dynamic step to pick gate cells.
		// Where the length is the number of cells of the adjacent side of two neighbor nodes.
		using StepFunction = std::function<int(int length)>;

		// QdTree is the type alias of a quadtree.
		using QdTree = Quadtree::Quadtree<bool>;
		// QdNode is the type alias of a quadtree node.
		using QdNode = Quadtree::Node<bool>;
		// QdNodeVisitor is the type of the function that visits quadtree nodes of the path finder.
		using QdNodeVisitor = std::function<void(QdNode* node)>;

		// Gate between two adjacent quadtree nodes from cell a in aNode to cell b in bNode.
		//  +-------+--------+
		//  |    [a => b]    |
		//  +-------+--------+
		//    aNode   bNode
		// 1. a and b are called 'gate cell's.
		struct Gate
		{
			QdNode *aNode, *bNode;
			int		a, b;
			Gate(QdNode* aNode, QdNode* bNode, int a, int b);
		};

		// GateVisitor the type of the function to visit gates.
		using GateVisitor = std::function<void(const Gate*)>;

		// Graph of gate cells.
		using GateGraph = SimpleDirectedGraph;

		// Graph of nodes.
		using NodeGraph = SimpleUnorderedMapDirectedGraph<QdNode*>;

		// QuadtreeMap is a 2D map maintained by a quadtree.
		// QuadtreeMap is nothing to do with agent size and terrain types.
		class QuadtreeMap
		{
		public:
			QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance, int step = 1,
				StepFunction stepf = nullptr, int maxNodeWidth = -1, int maxNodeHeight = -1);
			~QuadtreeMap();

			// ~~~~~~~~~~~~~~~ Cell ID Packing ~~~~~~~~~~~

			// PackXY packs a cell position (x,y) to an integral id v.
			int PackXY(int x, int y) const;

			// UnpackXY unpacks a vertex id v to a two-dimensional position (x,y).
			Cell UnpackXY(int v) const;

			// Unpacks a cell id v's x axis.
			int UnpackX(int v) const;

			// Unpacks a cell id v's y axis.
			int UnpackY(int v) const;

			// ~~~~~~~~~~~~~ Basic methods ~~~~~~~~~~~~~~~~~
			int W() const { return w; }
			int H() const { return h; }

			// Returns the distance between two vertices u and v.
			int Distance(int u, int v) const;
			int Distance(int x1, int y1, int x2, int y2) const;

			// Returns true if the given cell (x,y) is an obstacle.
			// if the given (x,y) is out of bounds, it's also considered an obstacle.
			bool IsObstacle(int x, int y) const;

			// Approximate distance between two quadtree nodes.
			// Using the provided distance calculator on their center cells.
			int DistanceBetweenNodes(QdNode* aNode, QdNode* bNode) const;

			// Returns the gates's graph.
			const GateGraph& GetGateGraph() const { return g2; }

			// Returns the nodes's graph.
			const NodeGraph& GetNodeGraph() const { return g1; }

			// ~~~~~~~~~~~~~ Visits and Reads ~~~~~~~~~~~~~~~~~

			// Get the quadtree node where the given cell (x,y) locates.
			// Returns nullptr if (x,y) is invalid (out of bound).
			QdNode* FindNode(int x, int y) const;

			// Is given cell u locating at given node a gate cell?
			bool IsGateCell(QdNode* node, int u) const;

			// Is given cell u is a gate?
			// higher level version method based on IsGateCell(node, u).
			bool IsGateCell(int u) const;

			// Visit each gate cell inside a node and call given visitor with it.
			void ForEachGateInNode(const QdNode* node, GateVisitor& visitor) const;

			// Visit all the quadtree's leaf nodes.
			void Nodes(QdNodeVisitor& visitor) const;

			// Visit all gate cells.
			// Note that dual gates (a => b) and (b => a) are visited twice (once for each).
			void Gates(GateVisitor& visitor) const;

			// Visit reachable neighbor nodes for given node on the node graph.
			void ForEachNeighbourNodes(QdNode* node, NeighbourVertexVisitor<QdNode*>& visitor) const;

			// Visit quadtree nodes inside given rectangle range.
			void NodesInRange(const Rectangle& rect, QdNodeVisitor& visitor) const;

			// ~~~~~~~~~~~~~ Graphs Maintaining ~~~~~~~~~~~~~~~~~

			// Build the underlying quadtree right after construction.
			// This will call tree.Build() for the underlying quadtree and add all existing obstacles.
			void Build();

			// Update should be called after any cell (x,y)'s value is changed.
			void Update(int x, int y);

		private:
			const int w, h, step;
			const int s; // max side of (w,h)
			const int maxNodeWidth, maxNodeHeight;

			ObstacleChecker	   isObstacle;
			DistanceCalculator distance;
			StepFunction	   stepf;

			// the quadtree on this grid map.
			QdTree tree;

			// ~~~~~~~~~~~~~~~ Graphs ~~~~~~~~~~~
			// the 1st level abstract graph: graph of nodes.
			NodeGraph g1;
			// the 2st level abstract graph: graph of gate cells.
			GateGraph g2;

			// ~~~~~~~~~~~~~~ Gates ~~~~~~~~~~~~~
			// manages memory of gates.
			std::unordered_set<Gate*> gates;
			// gates group by node for faster quering.
			// gates1[aNode][a][b] => Gate*
			// there may exist 1~3 gates starting from a cell.
			// for an example below, the 3 gates are:
			// (a => c), (a => b) and (a => d).
			//   b | c
			//   --+--
			//   a | d
			using Gates1Map = NestedNestedDefaultedUnorderedMap<QdNode*, int, int, Gate*, nullptr>;
			Gates1Map gates1;

			// ~~~~~~~~~~~~~~~~ Internals ~~~~~~~~~~~~~~~
			void ForEachGateInNode(QdNode* node, std::function<void(Gate*)>& visitor) const;
			void HandleNewNode(QdNode* aNode);
			void HandleRemovedNode(QdNode* aNode);
			void ConnectCellsInGateGraphs(int u, int v);
			void ConnectGateCellsInNodeToNewGateCell(QdNode* aNode, int a);
			void DisconnectCellInGateGraphs(int u);
			void ConnectNodesOnNodeGraph(QdNode* aNode, QdNode* bNode);
			void DisconnectNodeFromNodeGraph(QdNode* aNode);
			void CreateGate(QdNode* aNode, int a, QdNode* bNode, int b);
			void GetNeighbourCellsDiagonal(int direction, QdNode* aNode, int& a, int& b) const;
			void GetNeighbourCellsHV(int direction, QdNode* aNode, QdNode* bNode,
				std::vector<std::pair<int, int>>& ncs) const;
		};

	} // namespace internal
} // namespace qdpf

#endif
