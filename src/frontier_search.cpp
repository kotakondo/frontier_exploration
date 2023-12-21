#include "frontier_exploration/frontier_search.h"

#include "frontier_exploration/costmap_tools.h"

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <mutex>

namespace frontier_exploration {
    using costmap_2d::FREE_SPACE;
    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;

    FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap, double potentialWeight,
                                   double gainWeight, double closeFrontierWeight, double minFrontierSize,
                                   double frontierProximityThreshold) :
        _costmap(costmap),
        _potentialWeight(potentialWeight),
        _gainWeight(gainWeight),
        _closeFrontierWeight(closeFrontierWeight),
        _minFrontierSize(minFrontierSize),
        _frontierProximityThreshold(frontierProximityThreshold),
        _map(nullptr),
        _sizeX{},
        _sizeY{}
    {
    }

    std::vector<Frontier> FrontierSearch::searchFrom(const geometry_msgs::Point& position)
    {

        uint mx, my;
        if (!_costmap->worldToMap(position.x, position.y, mx, my))
        {
            ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
            return {};
        }

        std::vector<Frontier> frontierList;

        // make sure map is consistent and locked for duration of search
        std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

        _map = _costmap->getCharMap();
        _sizeX = _costmap->getSizeInCellsX();
        _sizeY = _costmap->getSizeInCellsY();

        // mark all cells as unchecked
        std::vector<CellState> cellStates(_sizeX * _sizeY, UNCHECKED);

        // initialize breadth first search (outer)
        std::queue<size_t> bfs;

        // find closest clear cell to start search
        size_t clear, pos = _costmap->getIndex(mx, my);
        if (nearestCell(clear, pos, FREE_SPACE, *_costmap))
        {
            bfs.push(clear);
        } else
        {
            bfs.push(pos);
            ROS_WARN("Could not find nearby clear cell to start search");
        }

        cellStates[bfs.front()] = MAP_OPEN;

        while (!bfs.empty())
        {
            size_t p = bfs.front();
            bfs.pop();

            if (cellStates[p] == MAP_CLOSED)
                continue;

            if (isFrontierCell(p))
            {
                // If we encountered a frontier cell, we run another bfs (inner BFS) starting from
                // this frontier cell to extract the complete frontier it is a part of. By keeping
                // track of the cell state we make sure no frontier cell is assigned to multiple frontiers
                Frontier newFrontier = buildFrontier(p, pos, cellStates);
                // Threshold based on frontier size
                if (newFrontier.size * _costmap->getResolution() >= _minFrontierSize)
                {
                    frontierList.push_back(newFrontier);
                }
            }
            // Iterate over neighbours of current point
            for (auto nbr : nhood8(p, *_costmap))
            {
                // Check neighbourhood of neighbour
                auto nHood = nhood8(nbr, *_costmap);
                bool hasFreeNeighbour =
                    std::any_of(nHood.begin(), nHood.end(),
                                [&map = this->_map](auto idx) { return map[idx] == FREE_SPACE; });
                // Include neighbour in search iff it has a free neighbour
                if (cellStates[nbr] != MAP_OPEN && cellStates[nbr] != MAP_CLOSED &&
                    hasFreeNeighbour)
                {
                    bfs.push(nbr);
                    cellStates[nbr] = MAP_OPEN;
                }
            }
            cellStates[p] = MAP_CLOSED;
        }

        // set costs of frontiers
        frontierCost(frontierList);
        
        // sort frontiers based on cost
        std::sort(frontierList.begin(), frontierList.end(),
                  [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

        return frontierList;
    }

    Frontier FrontierSearch::buildFrontier(size_t initialCell, size_t reference,
                                           std::vector<CellState>& cellStates)
    {
        Frontier output;
        std::vector<size_t> frontierIndices;

        output.centroid.x = 0;
        output.centroid.y = 0;
        output.size = 1;
        output.minDistance = std::numeric_limits<double>::infinity();

        uint ix, iy;
        _costmap->indexToCells(initialCell, ix, iy);
        _costmap->mapToWorld(ix, iy, output.initial.x, output.initial.y);

        std::queue<size_t> bfs;
        bfs.push(initialCell);
        cellStates[initialCell] = FRONTIER_OPEN;

        uint rx, ry;
        double referenceX, referenceY;
        _costmap->indexToCells(reference, rx, ry);
        _costmap->mapToWorld(rx, ry, referenceX, referenceY);

        while (!bfs.empty())
        {
            auto p = bfs.front();
            bfs.pop();

            if (cellStates[p] == FRONTIER_CLOSED || cellStates[p] == MAP_CLOSED)
                continue;

            if (isFrontierCell(p))
            {
                uint mx, my;
                double wx, wy;
                _costmap->indexToCells(p, mx, my);
                _costmap->mapToWorld(mx, my, wx, wy);

                geometry_msgs::Point point;
                point.x = wx;
                point.y = wy;
                output.points.push_back(point);

                // update frontier size
                output.size++;

                // update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;

                // determine frontier's distance from robot, going by closest grid cell
                // to robot
                double distance = sqrt(pow((double(referenceX) - double(wx)), 2.0) +
                                       pow((double(referenceY) - double(wy)), 2.0));
                if (distance < output.minDistance)
                {
                    output.minDistance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                // add to queue for breadth first search
                bfs.push(p);
                frontierIndices.push_back(p);
                for (auto nbr : nhood8(p, *_costmap))
                {
                    if (cellStates[nbr] == MAP_OPEN || cellStates[nbr] == UNCHECKED)
                    {
                        bfs.push(nbr);
                        cellStates[nbr] = FRONTIER_OPEN;
                    }
                }
            }
            cellStates[p] = FRONTIER_CLOSED;
        }
        // mark all frontier points as processed
        std::for_each(frontierIndices.begin(), frontierIndices.end(),
                      [&cellStates](auto idx) { cellStates[idx] = MAP_CLOSED; });

        output.centroid.x /= output.size;
        output.centroid.y /= output.size;

        return output;
    }

    bool FrontierSearch::isFrontierCell(size_t idx)
    {
        if (_map[idx] != NO_INFORMATION)
            return false;
        auto nHood = nhood4(idx, *_costmap);

        return std::any_of(nHood.begin(), nHood.end(),
                           [map = this->_map](auto idx) { return map[idx] == FREE_SPACE; });
    }

    std::vector<Frontier> FrontierSearch::frontierCost(std::vector<Frontier> frontierList)
    {

        // copy frontier list
        std::vector<Frontier> frontierListCopy = frontierList;

        for (auto frontier : frontierList)
        {

            // calculate num of close frontiers
            int numCloseFrontiers = 0;
            for (const auto otherFrontier : frontierList)
            {

                // skip if it is the same frontier
                if (frontier.centroid.x == otherFrontier.centroid.x &&
                    frontier.centroid.y == otherFrontier.centroid.y)
                {
                    continue;
                }

                // calculate distance between frontiers
                double dist = sqrt(pow((double(frontier.centroid.x) - double(otherFrontier.centroid.x)), 2.0) +
                                   pow((double(frontier.centroid.y) - double(otherFrontier.centroid.y)), 2.0));
                
                // if the distance is less than the threshold, increment the counter
                if (dist < _frontierProximityThreshold)
                {
                    numCloseFrontiers++;
                }
            }

            // option 1:
            // costs = min_distance - expected coverage - num of close frontiers
            frontier.cost = (_potentialWeight * frontier.minDistance * _costmap->getResolution()) // min_distance
                            - (_gainWeight * frontier.size * _costmap->getResolution())           // expected coverage
                            - (_closeFrontierWeight * numCloseFrontiers);                         // num of close frontiers

            // print out the values for tuning
            // ROS_INFO("min_distance: %f", frontier.minDistance * _costmap->getResolution());
            // ROS_INFO("expected coverage: %f", frontier.size * _costmap->getResolution());
            // ROS_INFO("num of close frontiers: %d", numCloseFrontiers);

            // option 2:
            // cost = min_distance - expected coverage
            // frontier.cost = (_potentialWeight * frontier.minDistance * _costmap->getResolution()) // min_distance
            //                 - (_gainWeight * frontier.size * _costmap->getResolution());          // expected coverage

            // push back the frontier
            frontierListCopy.push_back(frontier);

        } // end for

        // return the frontier list
        return frontierListCopy;

    } // end frontierCost function

} // namespace frontier_exploration
