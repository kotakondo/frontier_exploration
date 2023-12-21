#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration {
    // Represents a frontier
    struct Frontier
    {
        std::uint32_t size; // number of points
        double minDistance; // minimum distance from robot position
        double cost;
        geometry_msgs::Point initial;
        geometry_msgs::Point centroid;
        geometry_msgs::Point middle;
        std::vector<geometry_msgs::Point> points;
    };

    class FrontierSearch
    {
        // Represents state of a cell in costmap during frontier search
        enum CellState
        {
            // Enqueued by outer BFS
            MAP_OPEN,
            // Dequeued by outer BFS
            MAP_CLOSED,
            // Enqueued by inner BFS
            FRONTIER_OPEN,
            // Dequeued by inner BFS
            FRONTIER_CLOSED,
            // Cell hasn't been checked yet
            UNCHECKED
        };

    public:
        FrontierSearch() = default;

        FrontierSearch(costmap_2d::Costmap2D* costmap, double potentialWeight, double gainWeight,
                       double closeFrontierWeight, double minFrontierSize, double frontierProximityThreshold);

        std::vector<Frontier> searchFrom(const geometry_msgs::Point& position);

    protected:
        Frontier buildFrontier(size_t initialCell, size_t reference, std::vector<CellState>& cellStates);
        bool isFrontierCell(size_t idx);
        std::vector<Frontier> frontierCost(std::vector<Frontier> frontierList);

    private:
        costmap_2d::Costmap2D* _costmap;
        uint8_t* _map;
        size_t _sizeX, _sizeY;
        double _potentialWeight, _gainWeight, _closeFrontierWeight;
        double _minFrontierSize, _frontierProximityThreshold;
    };
} // namespace frontier_exploration
#endif
