#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include "../PriorityQueue.h"

#include <vector>
#include <queue>
#include <chrono>
#include <map>
#include <unordered_map>
#include <iostream>
#include <cmath>

// CREDITS
// I based the implementation of this project off of provided lecture content. It was invaluable in my assembly of the project.

// This forum post helped me consider a lot of ways to optimize my algorithm. I would like to be clear that I did not reference
// the code in the initial post (its actual implementation differs greatly from mine, as I hope is visible), only the advice given
// by the commenters. https://codereview.stackexchange.com/questions/285421/a-algorithm-implementation-in-modern-c

using namespace std;

namespace ufl_cap4053
{
	namespace searches
	{
		class SearchNode { // SearchNodes are the unit by which we represent a tile and its neighbors
		public:
			Tile* tile = nullptr;
			vector<Tile*> neighbors;
			float weight;

		};

		class PlannerNode {
		public:
			//Tile* vertex;
			SearchNode* searchNode;
			PlannerNode* parent;

			float heuristicCost;
			float givenCost;
			float finalCost;
		};

		bool isGreaterThan(PlannerNode* const& lhs, PlannerNode* const& rhs) { // Helper method to compare tile values
			// Just a regular ol' method, not beholden to any class
			return (lhs->finalCost > rhs->finalCost);
		}

		class PathSearch
		{
			private:
				int constructed;
				int destructed;
				Tile* goalTile;			// The tile we're trying to reach
				bool done;				// Boolean for whether or not the search has finished
				float heuristicWeight;	// The weight by which our heuristic cost is adjusted when calculating final cost
				vector<vector<Tile*>> searchSpace;	// The entire search space

				unordered_map<Tile*, SearchNode*> searchSpace2;	// Making the search space better
				map<pair<int, int>, SearchNode*> searchSpace3;

				//queue<PlannerNode*> bfsQueue;		// Queue of PlannerNodes for executing BFS
				PriorityQueue<PlannerNode*> pQueue;	// Queue of PlannerNodes for doing anything useful

				unordered_map<Tile*, PlannerNode*> visited;	// The visited map is an associated between Tiles and PlannerNodes
				//map<SearchNode*, PlannerNode*> visited2;	// The visited map is an associated between Tiles and PlannerNodes

				chrono::time_point<chrono::system_clock> start, current;

				bool areAdjacent(const Tile* lhs, const Tile* rhs); // Helper method to check adjacency of two tiles
				void currentSearchMethod(PriorityQueue<PlannerNode*>& queue);	// We'll call this A* once we're done, but for now, is generic
				float distanceEstimate(float x1, float y1, float x2, float y2);
				

			public:
				DLLEXPORT PathSearch() : pQueue(isGreaterThan) {
					done = false;
					//goalRoot = nullptr;
					goalTile = nullptr;
					heuristicWeight = .5f;
				}; // EX: DLLEXPORT required for public methods - see platform.h
				DLLEXPORT ~PathSearch();
				DLLEXPORT void load(TileMap* _tileMap);
				DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
				DLLEXPORT void update(long timeslice);
				DLLEXPORT void shutdown();
				DLLEXPORT void unload();
				DLLEXPORT bool isDone() const;
				DLLEXPORT std::vector<Tile const*> const getSolution() const;
		};

	}
}  // close namespace ufl_cap4053::searches
