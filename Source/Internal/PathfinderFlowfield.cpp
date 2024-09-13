// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "PathfinderFlowfield.h"

#include <cassert>
#include <cstdlib>
#include <queue>

#include "Base.h"

namespace qdpf
{
	namespace internal
	{

		///////////////////////////////
		// UnpackedCellFlowField
		////////////////////////////////

		bool UnpackedCellFlowField::Exist(const Cell& v) const
		{
			return m.find(v) != m.end();
		}

		void UnpackedCellFlowField::Clear()
		{
			m.clear();
		}

		std::size_t UnpackedCellFlowField::Size() const
		{
			return m.size();
		}

		const UnpackedCellFlowField::P& UnpackedCellFlowField::operator[](const Cell& v) const
		{
			auto it = m.find(v);
			if (it == m.end())
				return NullP;
			return it->second;
		}

		UnpackedCellFlowField::P& UnpackedCellFlowField::operator[](const Cell& v)
		{
			return m.try_emplace(v, NullP).first->second;
		}

		const Cell& UnpackedCellFlowField::Next(const Cell& v) const
		{
			return this->operator[](v).first;
		}

		int UnpackedCellFlowField::Cost(const Cell& v) const
		{
			return this->operator[](v).second;
		}

		const UnpackedCellFlowField::UnderlyingMap& UnpackedCellFlowField::GetUnderlyingMap() const
		{
			return m;
		}

		////////////////////////////////
		// FlowFieldPathFinderImpl
		////////////////////////////////

		// ffa1 is the flowfield pathfinder to work on the node graph.
		// ffa2 is the flowfield pathfinder to work on the gate graph.
		// In the constructor, we just initialized some lambda function for further reusing.
		FlowFieldPathFinderImpl::FlowFieldPathFinderImpl()
		{
			// nodesOverlappingQueryRangeCollector is to collect nodes overlapping with the query range.
			nodesOverlappingQueryRangeCollector = [this](QdNode* node) {
				// we care about only leaf nodes with no obstacles
				if (node->isLeaf && node->objects.empty())
					nodesOverlappingQueryRange.insert(node);
			};

			// gatesInNodesOverlappingQueryRangeCollector is to collect gates inside a single node within the
			// nodesOverlappingQueryRange.
			gatesInNodesOverlappingQueryRangeCollector = [this](const Gate* gate) {
				gatesInNodesOverlappingQueryRange.insert(gate->a);
			};

			// ffa1NeighborsCollector is to compute node flow field, it's used to visit every neighbour
			// vertex for given node.
			ffa1NeighborsCollector = [this](QdNode* u, NeighbourVertexVisitor<QdNode*>& visitor) {
				m->ForEachNeighbourNodes(u, visitor);
			};

			// ffa2NeighborsCollector is for computing gate flow field, it's used to visit every neighbour
			// gate cells for given gate cell u.
			// It collects neighbour on the { tmp + map } 's gate graph.
			ffa2NeighborsCollector = [this](int u, NeighbourVertexVisitor<int>& visitor) {
				ForEachNeighbourGateWithST(u, visitor);
			};
		}

		void FlowFieldPathFinderImpl::Reset(const QuadtreeMap* m, int x2, int y2,
			const Rectangle& qrange)
		{
			// debug mode, checks m, it's nullptr if mapx didn't find one.
			assert(m != nullptr);

			// resets the attributes.
			this->m = m;
			this->x2 = x2, this->y2 = y2;
			this->qrange = qrange; // copy updated
			tNode = nullptr;

			// the given qrange is invalid.
			if (!(qrange.x1 <= qrange.x2 && qrange.y1 <= qrange.y2))
				return;

			this->qrangeCenterX = (qrange.x1 + (qrange.x2 - qrange.x1) / 2);
			this->qrangeCenterY = (qrange.y1 + (qrange.y2 - qrange.y1) / 2);

			t = m->PackXY(x2, y2);
			tNode = m->FindNode(x2, y2);
			// tNode is not found, indicating that t is out of bounds.
			if (tNode == nullptr)
				return;

			// find all nodes overlapping with qrange.
			nodesOverlappingQueryRange.clear();
			m->NodesInRange(qrange, nodesOverlappingQueryRangeCollector);

			// find all gates inside nodesOverlappingQueryRange.
			gatesInNodesOverlappingQueryRange.clear();
			for (auto node : nodesOverlappingQueryRange)
			{ // node is const QdNode*
				m->ForEachGateInNode(node, gatesInNodesOverlappingQueryRangeCollector);
			}

			gateCellsOnNodeFields.clear();

			// Rebuild the tmp graph.
			PathFinderHelper::Reset(this->m);

			// Add the target cell to the gate graph
			bool tIsGate = m->IsGateCell(tNode, t);

			if (!tIsGate)
			{
				AddCellToNodeOnTmpGraph(t, tNode);
				// t is a virtual gate cell now.
				// we should check if it is inside the qrange,
				// and add it to gatesInNodesOverlappingQueryRange if it is.
				if (x2 >= qrange.x1 && x2 <= qrange.x2 && y2 >= qrange.y1 && y2 <= qrange.y2)
				{
					gatesInNodesOverlappingQueryRange.insert(t);
				}
			}

			// Special case:
			// if the target node overlaps the query range, we should connects the overlapping cells to
			// the target, since the best path is a straight line then.
			Rectangle tNodeRectangle{ tNode->x1, tNode->y1, tNode->x2, tNode->y2 };
			Rectangle overlap;

			auto hasOverlap = GetOverlap(tNodeRectangle, qrange, overlap);

			if (hasOverlap)
			{
				for (int x = overlap.x1; x <= overlap.x2; ++x)
				{
					for (int y = overlap.y1; y <= overlap.y2; ++y)
					{
						int u = m->PackXY(x, y);
						// detail notice is: we should skip u if it's a gate cell on the map's graph,
						// since we already connect all gate cells with t.
						if (u != t && !m->IsGateCell(tNode, u))
						{
							ConnectCellsOnTmpGraph(u, t);
							// We should consider u as a new tmp "gate" cell.
							//  we should add it to overlapping gates collection.
							gatesInNodesOverlappingQueryRange.insert(u);
						}
					}
				}
			}
		}

		// Computes node flow field.
		// 1. Perform flowfield algorithm on the node graph.
		// 2. Stops earlier if all nodes overlapping the query range are checked.
		int FlowFieldPathFinderImpl::ComputeNodeFlowField(NodeFlowField& nodeFlowField)
		{
			if (nodeFlowField.Size())
				nodeFlowField.Clear();

			// unreachable
			if (tNode == nullptr)
				return -1;

			// the target is an obstacle, unreachable
			if (m->IsObstacle(x2, y2))
				return -1;

			// stops earlier if all nodes overlapping with the query range are checked.

			// n counts the number of nodes marked by flowfield algorithm, for nodes inside
			// nodesOverlappingQueryRange.
			int n = 0;

			// stopf is a function to stop the flowfield algorithm from execution after given vertex (the
			// node) is marked.
			FFA1::StopAfterFunction stopf = [&n, this](QdNode* node) {
				if (nodesOverlappingQueryRange.find(node) != nodesOverlappingQueryRange.end())
					++n;
				// nodesOverlappingQueryRange's size will always > 0.
				return n >= nodesOverlappingQueryRange.size();
			};

			// Heuristic function for node level astar.
			// node's center to qrange's center.
			FFA1::HeuristicFunction ffa1Heuristic = [this](QdNode* node) {
				// node's center
				int nodeCenterX = node->x1 + (node->x2 - node->x1) / 2;
				int nodeCenterY = node->y1 + (node->y2 - node->y1) / 2;
				return m->Distance(nodeCenterX, nodeCenterY, qrangeCenterX, qrangeCenterY);
			};

			// Compute flowfield on the node graph.
			ffa1.Compute(tNode, nodeFlowField, ffa1Heuristic, ffa1NeighborsCollector, nullptr, stopf);

			ShrinkNodeFlowField(nodeFlowField);
			return 0;
		}

		// Shrink the computed node flowfield.
		// We traverse from the nodesOverlappingQueryRange, and picks all the nodes on the visited path,
		// and remove all the unrelated nodes.
		// This is to reduce number of nodes that will participate the further ComputeGateFlowField().
		void FlowFieldPathFinderImpl::ShrinkNodeFlowField(NodeFlowField& nodeFlowField)
		{
			NodeFlowField f;

			std::queue<QdNode*> q;
			for (auto node : nodesOverlappingQueryRange)
				q.push(node);

			while (q.size())
			{
				auto node = q.front();
				q.pop();
				if (f.Exist(node))
					continue;
				auto [next, cost] = nodeFlowField[node];
				if (cost == inf)
					continue;
				q.push(next);
				f[node] = { next, cost };
			}

			std::swap(f, nodeFlowField);
		}

		// collects the gate cells on the node flow field if ComputeNodeFlowField is successfully called
		// and ComputeGateFlowField is called with useNodeFlowField is set true.
		void FlowFieldPathFinderImpl::CollectGateCellsOnNodeField(const NodeFlowField& nodeFlowField)
		{
			gateCellsOnNodeFields.insert(t);

			// We have to add all non-gate neighbours of t on the tmp graph.
			NeighbourVertexVisitor<int> tmpNeighbourVisitor = [this](int v, int cost) {
				if (!m->IsGateCell(tNode, v))
					gateCellsOnNodeFields.insert(v);
			};
			tmp.ForEachNeighbours(t, tmpNeighbourVisitor);

			// gateVisitor collects the gate between current node and nextNode.
			QdNode *node = nullptr, *nextNode = nullptr;

			// gateVisitor is to collect gates inside current node.
			GateVisitor gateVisitor = [this, &node, &nextNode](const Gate* gate) {
				// tNode has no next
				if (node == tNode || node == nullptr || nextNode == nullptr)
					return;
				// collect only the gates between current node and next node.
				if (gate->bNode == nextNode)
				{
					gateCellsOnNodeFields.insert(gate->a);
					gateCellsOnNodeFields.insert(gate->b);
				};
			};

			// nodeVisitor visits each node inside the nodeField.
			for (auto& [v, p] : nodeFlowField.GetUnderlyingMap())
			{
				node = v;
				nextNode = p.first;
				m->ForEachGateInNode(node, gateVisitor);
			}
		}

		// Computes gate flow field.
		// 1. Perform flowfield algorithm on the gate graph.
		// 2. Stops earlier if all gate inside the nodes overlapping the query range are checked.
		// 3. If there's a previous ComputeNodeFlowField() call, use only the gates on the node field.
		int FlowFieldPathFinderImpl::ComputeGateFlowField(GateFlowField& gateFlowField,
			const NodeFlowField&										 nodeFlowField)
		{
			if (gateFlowField.Size())
				gateFlowField.Clear();

			if (tNode == nullptr)
				return -1;
			if (m->IsObstacle(x2, y2))
				return -1;

			// Collects the gate cells between nodes.
			if (nodeFlowField.Size())
			{
				if (gateCellsOnNodeFields.size())
					gateCellsOnNodeFields.clear();
				CollectGateCellsOnNodeField(nodeFlowField);
			}

			// stops earlier if all gates inside the query range are checked.

			// n counts the number of gates marked by flowfield algorithm, for gates inside
			// gatesOverlappingQueryRange.
			int n = 0;

			FFA2::StopAfterFunction stopf = [this, &n](int u) {
				if (gatesInNodesOverlappingQueryRange.find(u) != gatesInNodesOverlappingQueryRange.end())
					++n;
				return n >= gatesInNodesOverlappingQueryRange.size();
			};

			// if useNodeFlowField is true, we visit only the gate cells on the node field.
			FFA2::NeighbourFilterTesterT neighbourTester = [this, &nodeFlowField](int v) {
				if (nodeFlowField.Size() > 0 && gateCellsOnNodeFields.find(v) == gateCellsOnNodeFields.end())
					return false;
				return true;
			};

			// Heuristic function for gate level astar.
			// gate cell to qrange's center.
			FFA2::HeuristicFunction ffa2Heuristic = [this](int u) {
				auto [x, y] = m->UnpackXY(u);
				return m->Distance(x, y, qrangeCenterX, qrangeCenterY);
			};

			// Why we use a packedGateFlowField over the original unpacked gateFlowField?
			// reason: the gate graph is built on top of packed cell ids, so we have to do packings and
			// unpackings during the flowfield algorithm. Thus it's better to unpack the cell ids later on
			// the results.
			PackedCellFlowField packedGateFlowField;
			ffa2.Compute(t, packedGateFlowField, ffa2Heuristic, ffa2NeighborsCollector, neighbourTester,
				stopf);

			// Unpack into the gate flowfield.
			for (auto& [v, p] : packedGateFlowField.GetUnderlyingMap())
			{
				auto [next, cost] = p;
				auto [x, y] = m->UnpackXY(v);
				auto [x1, y1] = m->UnpackXY(next);
				gateFlowField[{ x, y }] = { { x1, y1 }, cost };
			}
			return 0;
		}

		int FlowFieldPathFinderImpl::ComputeGateFlowField(GateFlowField& gateFlowField)
		{
			NodeFlowField emptyNodeFlowField;
			return ComputeGateFlowField(gateFlowField, emptyNodeFlowField);
		}

		// Computes the final flow field via dynamic programming.
		// Time Complexity O(dest.w * dest.h);
		//
		// DP in brief:
		//
		// 1. Suppose the cost to target for cell (x,y) is f[x][y].
		// 2. For each node overlapping with the query range:
		//
		//     1. scan from left to right, up to bottom:
		//      // directions: left-up, up, left, right-up
		//      f[x][y] <= min(f[x][y], f[x-1][y-1], f[x-1][y], f[x][y-1], f[x-1][y+1]) + cost
		//
		//     2. scan from right to left, bottom to up:
		//      // directions: right-bottom, bottom, right, left-bottom
		//      f[x][y] <= min(f[x][y], f[x+1][y+1], f[x+1][y], f[x][y+1], f[x+1][y-1]) + cost
		//
		// This DP process is a bit faster than performing a Dijkstra on the dest rectangle.
		// O(M*N) vs O(M*N*logMN), since the optimal path will always come from a cell on the
		// node's borders. The optimal path should be a straight line, but there's no better
		// algorithm than O(M*N).
		int FlowFieldPathFinderImpl::ComputeFinalFlowField(FinalFlowField& finalFlowField,
			const GateFlowField&										   gateFlowField)
		{
			// ensures the finalFlowField is empty
			if (finalFlowField.Size())
				finalFlowField.Clear();

			if (tNode == nullptr)
				return -1;
			if (m->IsObstacle(x2, y2))
				return -1;

			// f[x][y] is the cost from the cell (x,y) to the target, all cells are initialized to inf.
			Final_F f;
			// from[x][y] stores which neighbour cell where the min value comes from.
			Final_From from;
			// b[x][y] indicates whether the (x,y)'s f and from values should be derived via DP.
			Final_B b;

			// initialize f from computed gate flow field.
			for (auto& [v, p] : gateFlowField.GetUnderlyingMap())
			{
				auto [next, cost] = p;

				auto [x, y] = v;
				auto [x1, y1] = next;

				// for a gate cell or virtual gate cell on the gateFlowField.
				// force it pointing to a neighbour cell on the direction to its next.
				// (x2,y2) is the neighbour cell to be computed.
				int x2, y2;
				FindNeighbourCellByNext(x, y, x1, y1, x2, y2);

				// find out the node of (x,y), O(log (Tree Depth))
				QdNode* node1 = m->FindNode(x, y);
				// find out the node of (x2,y2), O(log (Tree Depth))
				QdNode* node2 = m->FindNode(x2, y2);

				// for a cell A (x,y) on the gate flow field:
				if (node1 != tNode)
				{
					// If A is not inside tNode, and its next neighbour B is also inside node1, we should skip A:
					// 1. B's result is computed via DP, which is different with A. That may result in cyclic
					//    flows: A pointing B and B pointing A.
					// 2. Since A is pointing another gate C inside its node (on the gateFlowField), and A's node
					//    is not tNode, then C should point some cell outside. We can use C to derived A's result
					//    via DP instead.
					if (node1 == node2)
						continue;
				}
				else
				{ // node1 == tNode
					// If A's node is tNode, in the similar consideration, we should use it only if B is also
					// inside tNode.
					if (node2 != tNode)
						continue;
				}

				b[x][y] = true;
				f[x][y] = cost;
				from[x][y] = m->PackXY(x2, y2);
			}

			// cost unit on HV(horizonal and vertical) and diagonal directions.
			int c1 = m->Distance(0, 0, 0, 1), c2 = m->Distance(0, 0, 1, 1);

			// computes dp for each node, from node borders to inner.
			// why dp works: every node is empty (without obstacles inside it).
			for (auto node : nodesOverlappingQueryRange)
			{
				// cells inside both in tNode and the qrange are already computed in the ComputeGateFlowField.
				if (node == tNode)
					continue;
				ComputeFinalFlowFieldDP1(node, f, from, b, c1, c2);
				ComputeFinalFlowFieldDP2(node, f, from, b, c1, c2);
			}

			// computes the flow field in the query range.
			// note: we only collect the results for cells inside the qrange.
			for (int x = qrange.x1; x <= qrange.x2; ++x)
			{
				for (int y = qrange.y1; y <= qrange.y2; ++y)
				{
					// (x1,y1) is the next cell to go.
					auto [x1, y1] = m->UnpackXY(from[x][y]);
					// f is inf: unreachable
					if (f[x][y] == inf || from[x][y] == inf)
						continue;
					// {x,y} => next{x1,y1}, cost
					finalFlowField[{ x, y }] = { { x1, y1 }, f[x][y] };
				}
			}

			return 0;
		}

		// DP 1 of ComputeFinalFlowFieldInQueryRange inside a single leaf node.
		// From left-top corner to right-bottom corner.
		// c1 and c2 is the unit cost for HV and diagonal directions.
		void FlowFieldPathFinderImpl::ComputeFinalFlowFieldDP1(const QdNode* node, Final_F& f,
			Final_From& from, Final_B& b, int c1,
			int c2)
		{
			int x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
			for (int y = y1; y <= y2; ++y)
			{
				for (int x = x1; x <= x2; ++x)
				{
					// skipping the cells that already computed in the gate flow field.
					if (b[x][y])
						continue;

					int xfrom = -1, yfrom = -1;

					if (x > x1 && y > y1 && f[x][y] > f[x - 1][y - 1] + c2)
					{ // left-up
						f[x][y] = f[x - 1][y - 1] + c2;
						xfrom = x - 1, yfrom = y - 1;
					}
					if (y > y1 && f[x][y] > f[x][y - 1] + c1)
					{ // up
						f[x][y] = f[x][y - 1] + c1;
						xfrom = x, yfrom = y - 1;
					}
					if (x > x1 && f[x][y] > f[x - 1][y] + c1)
					{ // left
						f[x][y] = f[x - 1][y] + c1;
						xfrom = x - 1, yfrom = y;
					}
					if (y > y1 && x < x2 && f[x][y] > f[x + 1][y - 1] + c2)
					{ // right-up
						f[x][y] = f[x + 1][y - 1] + c2;
						xfrom = x + 1, yfrom = y - 1;
					}
					if (xfrom != -1)
						from[x][y] = m->PackXY(xfrom, yfrom);
				}
			}
		}

		// DP 2 of ComputeFinalFlowFieldInQueryRange  inside a single leaf node.
		// From right-bottom corner to left-top corner.
		// c1 and c2 is the unit cost for HV and diagonal directions.
		void FlowFieldPathFinderImpl::ComputeFinalFlowFieldDP2(const QdNode* node, Final_F& f,
			Final_From& from, Final_B& b, int c1,
			int c2)
		{
			int x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
			for (int y = y2; y >= y1; --y)
			{
				for (int x = x2; x >= x1; --x)
				{
					// skipping the cells that already computed in the gate flow field.
					if (b[x][y])
						continue;

					int xfrom = -1, yfrom = -1;

					if (x < x2 && y < y2 && f[x][y] > f[x + 1][y + 1] + c2)
					{ // right-bottom
						f[x][y] = f[x + 1][y + 1] + c2;
						xfrom = x + 1, yfrom = y + 1;
					}
					if (x < x2 && f[x][y] > f[x + 1][y] + c1)
					{ // right
						f[x][y] = f[x + 1][y] + c1;
						xfrom = x + 1, yfrom = y;
					}
					if (y < y2 && f[x][y] > f[x][y + 1] + c1)
					{ // bottom
						f[x][y] = f[x][y + 1] + c1;
						xfrom = x, yfrom = y + 1;
					}
					if (y < y2 && x > x1 && f[x][y] > f[x - 1][y + 1] + c2)
					{ // left-bottom
						f[x][y] = f[x - 1][y + 1] + c2;
						xfrom = x - 1, yfrom = y + 1;
					}
					if (xfrom != -1)
						from[x][y] = m->PackXY(xfrom, yfrom);
				}
			}
		}

		// (x,y) is a cell on the gateFlowField.
		// (x1,y1) is the next cell that (x,y) points to.
		// (x2,y2) is the result to compute, a neighbour cell of (x,y) on the direction to (x1,y1).
		//
		//  (x,y)
		//     \
        //      * (x2,y2)
		//       \
        //      (x1,y1)
		//
		void FlowFieldPathFinderImpl::FindNeighbourCellByNext(int x, int y, int x1, int y1, int& x2,
			int& y2)
		{
			int dx = x1 - x, dy = y1 - y;

			if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1)
			{
				// fast check: (x1,y1) is the neighbour.
				x2 = x1;
				y2 = y1;
				return;
			}

			// We draw a straight line from (x,y) to (x1,y1)
			// but we just stop the draw until the second cell, that is the neighbour.
			CellCollector collector = [&x2, &y2, x, y](int x3, int y3) {
				if (x3 == x && y3 == y)
					return;
				x2 = x3;
				y2 = y3;
			};
			ComputeStraightLine(x, y, x1, y1, collector, 2);
		}

	} // namespace internal
} // namespace qdpf
