// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_QUADTREE_MAPX_HPP
#define QDPF_INTERNAL_QUADTREE_MAPX_HPP

// QuadtreeMapX
// ~~~~~~~~~~~~
// A manager of multiple quadtree maps to support different agent sizes and terrain types.

#include <initializer_list>
#include <unordered_map>

#include "ClearanceField/Source/ClearanceField.h"
#include "QuadtreeMap.h"

namespace QDPF
{
	namespace Internal
	{

		const float kClearanceFieldCostUnitScaleFactor = 1e4;

		struct QuadtreeMapXSetting
		{
			int AgentSize;
			// OR result of terrain type integers.
			int TerrainTypes;
		};

		// Supported clearance field kinds.
		// A clearance field must implement the interface ClearanceField::IClearanceField;
		enum class ClearanceFieldKind
		{
			TrueClearanceField = 0,
			BrushfireClearanceField = 1,
		};

		using QuadtreeMapXSettings = std::initializer_list<QuadtreeMapXSetting>;

		// TerrainTypesChecker is to check the terrain type value for given cell (x,y)
		using TerrainTypesChecker = std::function<int(int x, int y)>;

		class QuadtreeMapXImpl
		{
		public:
			QuadtreeMapXImpl(int w, int h, DistanceCalculator distance, TerrainTypesChecker terrainChecker,
				QuadtreeMapXSettings settings, int step = 1, StepFunction stepf = nullptr,
				int maxNodeWidth = -1, int maxNodeHeight = -1,
				ClearanceFieldKind clearanceFieldKind = ClearanceFieldKind::TrueClearanceField);
			~QuadtreeMapXImpl();

			int W() const { return w; }
			int H() const { return h; }

			// Creates quadtree maps, clearance fields and call Update on existing grid map for each cell.
			void Build();

			// Find a quadtree map by agent size and walkable terrain types.
			// Returns nullptr on not found.
			[[nodiscard]] const QuadtreeMap* Get(int agentSize, int walkableTerrainTypes) const;

			// Update should be called if cell (x,y)'s terrain is changed.
			// If the (x,y) is out of bound, nothing happens.
			void Update(int x, int y);

			// Compute should be called after one or more Update calls, to apply the changes to all related
			// quadtree maps.
			void Compute();

		private:
			const int				   w, h, maxNodeWidth, maxNodeHeight;
			const int				   step;
			StepFunction			   stepf;
			DistanceCalculator		   distance;
			TerrainTypesChecker		   terrainChecker;
			const QuadtreeMapXSettings settings;
			const ClearanceFieldKind   clearanceFieldKind;

			// ~~~~~~~ clearance fields ~~~~~~~~~~~
			// cfs[terrainTypes] => cf.
			std::unordered_map<int, ClearanceField::IClearanceField*> cfs;

			// ~~~~~~~~~~~ quadtree maps ~~~~~~~~~~~
			// maps[agentSize][terrainTypes] => map.
			std::unordered_map<int, std::unordered_map<int, QuadtreeMap*>> maps;
			// redundancy map for: maps1[terrainTypes] => list of quadtree map pointers.
			std::unordered_map<int, std::vector<QuadtreeMap*>> maps1;

			// records the dirty cells where the clearance value changed.
			// they are cleared after Compute().
			// dirties[terrainTypes] => {(x,y), ...}
			std::unordered_map<int, std::vector<std::pair<int, int>>> dirties;

			// ~~~~~ clearance fields ~~~~~~~
			void CreateClearanceFields();
			void CreateClearanceFieldForTerrainTypes(int agentSizeBound, float costUnit, float costUnitDiagonal,
				int terrainTypes);
			void BuildClearanceFields();

			// ~~~~~ quadtree maps ~~~~~~~
			void CreateQuadtreeMaps();
			void CreateQuadtreeMapsForSetting(int agentSize, int terrainTypes);
			void BuildQuadtreeMaps();

			// ~~~~~ bind them ~~~~~~~
			void BindClearanceFieldAndQuadtreeMaps();
		};

	} // namespace Internal
} // namespace QDPF

#endif
