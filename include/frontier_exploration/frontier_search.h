#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration {

    struct Frontier
    {
        std::uint32_t size;
        double minDistance;
        double cost;
        geometry_msgs::Point initial;
        geometry_msgs::Point centroid;
        geometry_msgs::Point middle;
        std::vector<geometry_msgs::Point> points;
    };

    class FrontierSearch
    {
    public:
        FrontierSearch() = default;

        FrontierSearch(costmap_2d::Costmap2D* costmap, double potentialScale, double gainScale,
                       double minFrontierSize);

        std::vector<Frontier> searchFrom(const geometry_msgs::Point& position);

    protected:
        Frontier buildNewFrontier(size_t initialCell, size_t reference,
                                  std::vector<bool>& frontierFlag);

        bool isNewFrontierCell(size_t idx, const std::vector<bool>& frontierFlag);

        double frontierCost(const Frontier& frontier);

    private:
        costmap_2d::Costmap2D* _costmap;
        uint8_t* _map;
        size_t _sizeX, _sizeY;
        double _potentialScale, _gainScale;
        double _minFrontierSize;
    };
} // namespace frontier_exploration
#endif
