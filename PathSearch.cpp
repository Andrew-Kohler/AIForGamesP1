#include "PathSearch.h"

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#ifdef _DEBUG
	#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type
#else
	#define DBG_NEW new
#endif


using namespace std;

namespace ufl_cap4053
{
	namespace searches
	{
		PathSearch::~PathSearch() { // Destructor; performs final cleanup before deletion
			// These two take care of every piece of memory allocated as far as I'm aware
			shutdown();
			unload();
		}

		void PathSearch::load(TileMap* _tileMap) {	
			// After the tile map is loaded, call this - this is where the search graph is generated
			// NOTE: This method must take no longer than twice the benchmark example for any given map
			int width = _tileMap->getColumnCount();
			int height = _tileMap->getRowCount();

			// Load process
			for (int i = 0; i < height; i++) {
				for (int j = 0; j < width; j++) {
					SearchNode* node = new SearchNode();	// Create a search node
					node->tile = _tileMap->getTile(i, j);	// Assign it a tile
					node->weight = node->tile->getWeight();

				// Determine all of its neighbors (which is icky, but it's better to do it here than during update)
					for (int k = -1; k < 2; k++) {		// Check each of the surrounding tiles
						if (i + k >= 0 && i + k < height) { // Check that the row is in bounds
							for (int p = -1; p < 2; p++) {
								if (j + p >= 0 && j + p < width) { // Check that the column is in bounds
									// We can now perform the actual logic without having to worry abt OOB
									Tile* checkTile = _tileMap->getTile(i + k, j + p);

									if (areAdjacent(node->tile, checkTile)) {
										node->neighbors.push_back(checkTile);
									}

								}
							}
						}
					} // End of neighbor checking

					// Add the node to the map 
					searchSpace2[_tileMap->getTile(i, j)] = node;
					searchSpace3[make_pair(i, j)] = node;

				}

			} // End of iterating over each node to load them in


		}

		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
			done = false;

			PlannerNode* start = new PlannerNode(); // Create a PlannerNode for the start position and enqueue it into our queue
			start->parent = nullptr;

			start->searchNode = searchSpace3[make_pair(startRow, startCol)];
			goalTile = searchSpace3[make_pair(goalRow, goalCol)]->tile;
			
			//start->searchNode = searchSpace3.at(startRow).at(startCol);
			start->heuristicCost = distanceEstimate((float)start->searchNode->tile->getXCoordinate(), (float)start->searchNode->tile->getYCoordinate(), (float)goalTile->getXCoordinate(), (float)goalTile->getYCoordinate());
			start->givenCost = 0;
			start->finalCost = start->heuristicCost * heuristicWeight;

			pQueue.push(start);

			visited.insert(pair<Tile*, PlannerNode*>(start->searchNode->tile, start));	// Add this first node to our visited map
		}

		void PathSearch::update(long timeslice) {
			if (!done) {	// Only initiate an update if a solution has not been found
				if (timeslice == 0) { // Execute 1 iteration of the search
					currentSearchMethod(pQueue);
				}
				else {
					start = chrono::system_clock::now();	// Get the current time and store it
					current = start;	

					// Turn on the while loop, and keep it going until the timer hits 0

					while (duration_cast<std::chrono::milliseconds>(current - start).count() < timeslice && !done) { // && !pQueue.empty() -> if this is empty and we aren't done, something is wrong
						currentSearchMethod(pQueue);
						current = chrono::system_clock::now();
					}
				}
			}
			
		}

		void PathSearch::shutdown() {
			// Clears memory allocated for THIS SEARCH in particular - anything in the queue, and anything in the visited map		
			if (goalTile != nullptr) {
				goalTile = nullptr;
			}

			// Clear out the queue used to perform the search; we don't perform deletions here because clearing out the visited map will delete everything
			pQueue.clear();

			unordered_map<Tile*, PlannerNode*>::iterator iter;	// Clear out the visited map - DEALLOCATE MEMORY
			for (iter = visited.begin(); iter != visited.end(); iter++) {
				//if (iter->second != nullptr) {
					/*iter->second->parent = nullptr;
					iter->second->searchNode = nullptr;*/
					delete iter->second;
				//}	
			}
			visited.clear();
		}

		void PathSearch::unload() {
			// Clears out any memory allocated for the search space
			if (goalTile != nullptr) {
				goalTile = nullptr;
			}

			unordered_map<Tile*, SearchNode*>::iterator iterEnd;	// Clear out the visited map - DEALLOCATE MEMORY
			for (iterEnd = searchSpace2.begin(); iterEnd != searchSpace2.end(); iterEnd++) {
				//if (iterEnd->second != nullptr) {
					/*iterEnd->second->tile = nullptr;
					for (int i = 0; i < iterEnd->second->neighbors.size(); i++) {
						iterEnd->second->neighbors.at(i) = nullptr;
						
					}
					iterEnd->second->neighbors.clear();*/
					delete (iterEnd->second);
				//}
				
			}

			searchSpace2.clear();
			searchSpace3.clear();	
		}

		bool PathSearch::isDone() const {
			// Have update set a bool when it's done so that this can tell us if we're done
			// Don't reset the done bool until a call to initialize
			return done;
		}

		vector<Tile const*> const PathSearch::getSolution() const {
			vector<Tile const*> temp;

			PlannerNode* pathHome = pQueue.front();
			while (pathHome->parent != nullptr) { // Sets up the vector from finish to start
				
				pathHome->searchNode->tile->addLineTo(pathHome->parent->searchNode->tile, 0xFF7F0000);
				temp.push_back(pathHome->searchNode->tile);
				pathHome = pathHome->parent;
				if (pathHome->parent == nullptr) {
					temp.push_back(pathHome->searchNode->tile);
				}
				
			}
			return temp;
		}

		// Private methods
		bool PathSearch::areAdjacent(const Tile* lhs, const Tile* rhs) {
			bool adjacent = false;
			int leftRow = lhs->getRow();	// Set up all of these so I don't have to call them over and over
			int leftCol = lhs->getColumn();
			int rightRow = rhs->getRow();
			int rightCol = rhs->getColumn();

			// Initial check to make sure we're in the ballpark
			if (rightRow > leftRow - 2 && rightRow < leftRow + 2 && rightCol > leftCol - 2 && rightCol < leftCol + 2 && rhs->getWeight() != 0) {
				if (leftRow % 2 == 0) { // Different checks for even or odd rows
					if (rightCol == leftCol + 1) {
						if (rightRow == leftRow) {
							adjacent = true;
						}
					}
					else {
						adjacent = true;
					}
				}
				else {
					if (rightCol == leftCol - 1) {
						if (rightRow == leftRow) {
							adjacent = true;
						}
					}
					else {
						adjacent = true;
					}
				}
			}
			
			return adjacent;
		}

		void PathSearch::currentSearchMethod(PriorityQueue<PlannerNode*>& queue) {
			PlannerNode* current = queue.front();

			if (current->searchNode->tile == goalTile) { // If the node we're currently on IS the goal :)
				// We can just leave the thing we were looking for on the front of the queue and save the trouble of an extra deletion
				done = true;
			}

			else {	// If it isn't :(
				queue.pop();
				for (Tile* tile : current->searchNode->neighbors) {
					float tempGivenCost = current->givenCost + tile->getWeight();

					if (visited[tile] != NULL) {
						//tile->setFill(0x7F00FFF0);
						if (tempGivenCost < visited[tile]->givenCost) { // If we find something lower cost, that's just good business
							PlannerNode* successor = visited[tile];
							queue.remove(successor);
							successor->parent = current;
							successor->givenCost = tempGivenCost;
							successor->finalCost = tempGivenCost + successor->heuristicCost * heuristicWeight;
							queue.push(successor);
						}

					}
					else { // If we haven't been here before
						//tile->setFill(0x7F00FF00);
						PlannerNode* successor = new PlannerNode();	// Make a new planner node
						successor->parent = current;
						successor->searchNode = searchSpace2[tile]; 
						successor->heuristicCost = distanceEstimate((float)tile->getXCoordinate(), (float)tile->getYCoordinate(), (float)goalTile->getXCoordinate(), (float)goalTile->getYCoordinate());
						successor->givenCost = tempGivenCost;
						successor->finalCost = successor->givenCost + successor->heuristicCost * heuristicWeight;
						visited[tile] = successor;	// We have now visited this tile
						queue.push(successor);
					}
				}
			
			}
			
		} // End of method

		float PathSearch::distanceEstimate(float x1, float y1, float x2, float y2) { // Just an implementation of the distance function

			return (float)sqrt((pow((x2 - x1), 2) + pow((y2 - y1), 2)));
		}


	}
}  // close namespace ufl_cap4053::searches

