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

    FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap, double potentialScale,
                                   double gainScale, double minFrontierSize) :
        _costmap(costmap),
        _potentialScale(potentialScale),
        _gainScale(gainScale),
        _minFrontierSize(minFrontierSize),
        _map(nullptr),
        _sizeX{},
        _sizeY{}
    {
    }

    std::vector<Frontier> FrontierSearch::searchFrom(const geometry_msgs::Point& position)
    {
        std::vector<Frontier> frontierList;

        uint mx, my;
        if (!_costmap->worldToMap(position.x, position.y, mx, my))
        {
            ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
            return frontierList;
        }

        // make sure map is consistent and locked for duration of search
        std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

        _map = _costmap->getCharMap();
        _sizeX = _costmap->getSizeInCellsX();
        _sizeY = _costmap->getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontierFlag(_sizeX * _sizeY, false);
        std::vector<bool> visitedFlag(_sizeX * _sizeY, false);

        // initialize breadth first search
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
        visitedFlag[bfs.front()] = true;

        while (!bfs.empty())
        {
            size_t idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (auto nbr : nhood4(idx, *_costmap))
            {
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell
                if (_map[nbr] <= _map[idx] && !visitedFlag[nbr])
                {
                    visitedFlag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
                    // neighbour)
                } else if (isNewFrontierCell(nbr, frontierFlag))
                {
                    frontierFlag[nbr] = true;
                    Frontier newFrontier = buildNewFrontier(nbr, pos, frontierFlag);
                    if (newFrontier.size * _costmap->getResolution() >= _minFrontierSize)
                    {
                        frontierList.push_back(newFrontier);
                    }
                }
            }
        }

        // set costs of frontiers
        for (auto& frontier : frontierList)
        {
            frontier.cost = frontierCost(frontier);
        }
        std::sort(frontierList.begin(), frontierList.end(),
                  [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

        return frontierList;
    }

    Frontier FrontierSearch::buildNewFrontier(size_t initialCell, size_t reference,
                                              std::vector<bool>& frontierFlag)
    {
        // initialize frontier structure
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;
        output.size = 1;
        output.minDistance = std::numeric_limits<double>::infinity();

        // record initial contact point for frontier
        uint ix, iy;
        _costmap->indexToCells(initialCell, ix, iy);
        _costmap->mapToWorld(ix, iy, output.initial.x, output.initial.y);

        // push initial cell onto queue
        std::queue<size_t> bfs;
        bfs.push(initialCell);

        // cache reference position in world coords
        uint rx, ry;
        double referenceX, referenceY;
        _costmap->indexToCells(reference, rx, ry);
        _costmap->mapToWorld(rx, ry, referenceX, referenceY);

        while (!bfs.empty())
        {
            size_t idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (size_t nbr : nhood8(idx, *_costmap))
            {
                // check if neighbour is a potential frontier cell
                if (isNewFrontierCell(nbr, frontierFlag))
                {
                    // mark cell as frontier
                    frontierFlag[nbr] = true;
                    uint mx, my;
                    double wx, wy;
                    _costmap->indexToCells(nbr, mx, my);
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

                    // determine frontier's distance from robot, going by closest gridcell
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
                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.size;
        output.centroid.y /= output.size;
        return output;
    }

    bool FrontierSearch::isNewFrontierCell(size_t idx, const std::vector<bool>& frontierFlag)
    {
        // check that cell is unknown and not already marked as frontier
        if (_map[idx] != NO_INFORMATION || frontierFlag[idx])
            return false;

        auto nHood = nhood4(idx, *_costmap);
        return std::any_of(nHood.begin(), nHood.end(),
                           [&map = this->_map](size_t idx) { return map[idx] == FREE_SPACE; });
    }

    double FrontierSearch::frontierCost(const Frontier& frontier)
    {
        return (_potentialScale * frontier.minDistance * _costmap->getResolution()) -
               (_gainScale * frontier.size * _costmap->getResolution());
    }
} // namespace frontier_exploration
