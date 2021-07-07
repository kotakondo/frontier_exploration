#include "frontier_exploration/explore.h"

#include <thread>

#define ATTRIBUTE_UNUSED(expr) (void)(expr)

inline static bool operator==(const geometry_msgs::Point& one, const geometry_msgs::Point& two)
{
    double dx = one.x - two.x;
    double dy = one.y - two.y;
    double dist = sqrt(dx * dx + dy * dy);
    return dist < 0.01;
}

namespace frontier_exploration {
    Explore::Explore(ros::NodeHandle* nh, ros::NodeHandle* pnh) :
        _nh(nh),
        _pnh(pnh),
        _tf(ros::Duration(10.0)),
        _costmapClient(*nh, *pnh, &_tf),
        _mbClient("move_base"),
        _prevDistance(0),
        _lastMarkersCount(0),
        _plannerFrequency(1.0),
        _potentialScale(1.e-3),
        _orientationScale(0.0),
        _gainScale(1.0),
        _visualize(false),
        _search{}
    {
        double timeout;
        double minFrontierSize;
        _pnh->param("planner_frequency", _plannerFrequency, 1.0);
        _pnh->param("progress_timeout", timeout, 30.0);
        _progressTimeout = ros::Duration(timeout);
        _pnh->param("visualize", _visualize, false);
        _pnh->param("potential_scale", _potentialScale, 1e-3);
        _pnh->param("orientation_scale", _orientationScale, 0.0);
        _pnh->param("gain_scale", _gainScale, 1.0);
        _pnh->param("min_frontier_size", minFrontierSize, 0.5);

        _search = frontier_exploration::FrontierSearch(_costmapClient.getCostmap(), _potentialScale,
                                                       _gainScale, minFrontierSize);

        if (_visualize)
        {
            _markerPub = _pnh->advertise<visualization_msgs::MarkerArray>("frontiers", 10);
        }

        ROS_INFO("Waiting to connect to move_base server");
        _mbClient.waitForServer();
        ROS_INFO("Connected to move_base server");

        _explorationTimer = _nh->createTimer(ros::Duration(1. / _plannerFrequency),
                                             [this](const ros::TimerEvent&) { makePlan(); });
    }

    Explore::~Explore() { stop(); }

    void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
    {
        std_msgs::ColorRGBA blue;
        blue.r = 0;
        blue.g = 0;
        blue.b = 1.0;
        blue.a = 1.0;
        std_msgs::ColorRGBA red;
        red.r = 1.0;
        red.g = 0;
        red.b = 0;
        red.a = 1.0;
        std_msgs::ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0;
        green.a = 1.0;

        ROS_DEBUG("visualising %lu frontiers", frontiers.size());
        visualization_msgs::MarkerArray markersMsg;
        std::vector<visualization_msgs::Marker>& markers = markersMsg.markers;
        visualization_msgs::Marker m;

        m.header.frame_id = _costmapClient.getGlobalFrameID();
        m.header.stamp = ros::Time::now();
        m.ns = "frontiers";
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 255;
        m.color.a = 255;
        // lives forever
        m.lifetime = ros::Duration(0);
        m.frame_locked = true;

        m.action = visualization_msgs::Marker::ADD;
        size_t id = 0;
        for (auto& frontier : frontiers)
        {
            m.type = visualization_msgs::Marker::POINTS;
            m.id = int(id);
            m.pose.position = {};
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.points = frontier.points;
            if (goalOnBlacklist(frontier.centroid))
            {
                m.color = red;
            } else
            {
                m.color = blue;
            }
            markers.push_back(m);
            ++id;
        }
        size_t currentMarkersCount = markers.size();

        // delete previous markers, which are now unused
        m.action = visualization_msgs::Marker::DELETE;
        for (; id < _lastMarkersCount; ++id)
        {
            m.id = int(id);
            markers.push_back(m);
        }

        _lastMarkersCount = currentMarkersCount;
        _markerPub.publish(markersMsg);
    }

    void Explore::makePlan()
    {
        // find frontiers
        auto pose = _costmapClient.getRobotPose();
        // get frontiers sorted according to cost
        auto frontiers = _search.searchFrom(pose.position);
        ROS_DEBUG("found %lu frontiers", frontiers.size());
        for (size_t i = 0; i < frontiers.size(); ++i)
        {
            ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
        }

        if (frontiers.empty())
        {
            stop();
            return;
        }

        // publish frontiers as visualization markers
        if (_visualize)
        {
            visualizeFrontiers(frontiers);
        }

        // find non blacklisted frontier
        auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                                         [this](const frontier_exploration::Frontier& f) {
                                             return goalOnBlacklist(f.centroid);
                                         });
        if (frontier == frontiers.end())
        {
            stop();
            return;
        }
        geometry_msgs::Point targetPosition = frontier->centroid;

        // time out if we are not making any progress
        bool sameGoal = _prevGoal == targetPosition;
        _prevGoal = targetPosition;
        if (!sameGoal || _prevDistance > frontier->minDistance)
        {
            // we have different goal or we made some progress
            _lastProgress = ros::Time::now();
            _prevDistance = frontier->minDistance;
        }
        // black list if we've made no progress for a long time
        if (ros::Time::now() - _lastProgress > _progressTimeout)
        {
            _frontierBlacklist.push_back(targetPosition);
            ROS_DEBUG("Adding current goal to black list");
            makePlan();
            return;
        }

        // we don't need to do anything if we still pursuing the same goal
        if (sameGoal)
            return;

        // send goal to move_base if we have something new to pursue
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position = targetPosition;
        goal.target_pose.pose.orientation.w = 1.;
        goal.target_pose.header.frame_id = _costmapClient.getGlobalFrameID();
        goal.target_pose.header.stamp = ros::Time::now();
        _mbClient.sendGoal(
            goal, [this, targetPosition](const actionlib::SimpleClientGoalState& status,
                                         const move_base_msgs::MoveBaseResultConstPtr& result) {
                reachedGoal(status, result, targetPosition);
            });
    }

    bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
    {
        costmap_2d::Costmap2D* costmap = _costmapClient.getCostmap();

        // check if a goal is on the blacklist for goals that we're pursuing
        return std::any_of(_frontierBlacklist.begin(), _frontierBlacklist.end(),
                           [goal, tol = 5, res = costmap->getResolution()](auto& frontierGoal) {
                               double dx = fabs(goal.x - frontierGoal.x);
                               double dy = fabs(goal.y - frontierGoal.y);

                               return (dx < tol * res && dy < tol * res);
                           });
    }

    void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                              const move_base_msgs::MoveBaseResultConstPtr&,
                              const geometry_msgs::Point& frontier_goal)
    {
        ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
        if (status == actionlib::SimpleClientGoalState::ABORTED)
        {
            _frontierBlacklist.push_back(frontier_goal);
            ROS_DEBUG("Adding current goal to black list");
        }

        // find new goal immediately regardless of planning frequency.
        // execute via timer to prevent dead lock in move_base_client (this is
        // callback for sendGoal, which is called in makePlan). the timer must live
        // until callback is executed.
        _oneShotTimer = _nh->createTimer(
            ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); }, true);
        ATTRIBUTE_UNUSED(_oneShotTimer);
    }

    void Explore::start() { _explorationTimer.start(); }

    void Explore::stop()
    {
        _mbClient.cancelAllGoals();
        _explorationTimer.stop();
        ROS_INFO("Exploration stopped.");
    }

} // namespace frontier_exploration
