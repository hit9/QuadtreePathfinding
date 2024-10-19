// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "QDPF.h"

namespace QDPF
{

	//////////////////////////////////////
	/// QuadtreeMapX
	//////////////////////////////////////

	QuadtreeMapX::QuadtreeMapX(int w, int h, DistanceCalculator distance,
		TerrainTypesChecker terrainChecker, QuadtreeMapXSettings settings,
		int step, StepFunction stepf, int maxNodeWidth, int maxNodeHeight,
		ClearanceFieldKind clearanceFieldKind)
		: impl(Internal::QuadtreeMapXImpl(w, h, distance, terrainChecker, settings, step, stepf,
			  maxNodeWidth, maxNodeHeight, clearanceFieldKind)) {}

	void QuadtreeMapX::Build()
	{
		impl.Build();
	}
	void QuadtreeMapX::Update(int x, int y)
	{
		impl.Update(x, y);
	}
	void QuadtreeMapX::Compute()
	{
		impl.Compute();
	}
	const Internal::QuadtreeMap* QuadtreeMapX::Get(int agentSize, int terrainTypes) const
	{
		return impl.Get(agentSize, terrainTypes);
	}

	//////////////////////////////////////
	/// AStarPathFinder
	//////////////////////////////////////

	AStarPathFinder::AStarPathFinder(const QuadtreeMapX& mx)
		: mx(mx) {}

	int AStarPathFinder::Reset(int x1, int y1, int x2, int y2, int agentSize, int terrainTypes)
	{
		auto m = mx.Get(agentSize, terrainTypes);
		if (m == nullptr)
			return -1;
		impl.Reset(m, x1, y1, x2, y2);
		return 0;
	}

	AstarResult AStarPathFinder::ComputeNodeRoutes(NodePath& nodePath)
	{
		return impl.ComputeNodeRoutes(nodePath);
	}

	AstarResult AStarPathFinder::ComputeGateRoutes(GateRouteCollector& collector, const NodePath& nodePath)
	{
		return impl.ComputeGateRoutes(collector, nodePath);
	}

	AstarResult AStarPathFinder::ComputeGateRoutes(GateRouteCollector& collector)
	{
		return impl.ComputeGateRoutes(collector);
	}

	AstarResult AStarPathFinder::ComputeGateRoutes(GatePath& path, const NodePath& nodePath)
	{
		GateRouteCollector collector = [&path](int x, int y, int cost) { path.push_back({ x, y, cost }); };
		return ComputeGateRoutes(collector, nodePath);
	}

	AstarResult AStarPathFinder::ComputeGateRoutes(GatePath& path)
	{
		GateRouteCollector collector = [&path](int x, int y, int cost) { path.push_back({ x, y, cost }); };
		return ComputeGateRoutes(collector);
	}

	//////////////////////////////////////
	/// FlowFieldPathFinder
	//////////////////////////////////////

	FlowFieldPathFinder::FlowFieldPathFinder(const QuadtreeMapX& mx)
		: mx(mx) {}

	int FlowFieldPathFinder::Reset(int x2, int y2, const Rectangle& dest, int agentSize,
		int walkableterrainTypes)
	{
		auto m = mx.Get(agentSize, walkableterrainTypes);
		if (m == nullptr)
			return -1;
		impl.Reset(m, x2, y2, dest);
		return 0;
	}

	FlowFieldResult FlowFieldPathFinder::ComputeNodeFlowField(NodeFlowField& nodeFlowfield)
	{
		return impl.ComputeNodeFlowField(nodeFlowfield);
	}

	FlowFieldResult FlowFieldPathFinder::ComputeGateFlowField(GateFlowField& gateFlowField)
	{
		return impl.ComputeGateFlowField(gateFlowField);
	}

	FlowFieldResult FlowFieldPathFinder::ComputeGateFlowField(GateFlowField& gateFlowField,
		const NodeFlowField&												 nodeFlowField)
	{
		return impl.ComputeGateFlowField(gateFlowField, nodeFlowField);
	}

	FlowFieldResult FlowFieldPathFinder::ComputeFinalFlowField(FinalFlowField& finalFlowfield,
		const GateFlowField&												   gateFlowField)
	{
		return impl.ComputeFinalFlowField(finalFlowfield, gateFlowField);
	}

} // namespace QDPF
