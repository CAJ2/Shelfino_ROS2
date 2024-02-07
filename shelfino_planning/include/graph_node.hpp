#include <vector>
#include <limits>
#include <cmath>
#include "planning_msgs/msg/point2_d.hpp"

namespace graph_search {

    //CHANGED from manhattan to euclidean distance
    double distance(const planning_msgs::msg::Point2D& p1, const planning_msgs::msg::Point2D& p2) {
        // Euclidean distance formula: sqrt((x2 - x1)^2 + (y2 - y1)^2)
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    class Node {
    public:
        int nodeID; // Unique identifier for the node
        std::vector<int> neighbors; // List of nodeIDs representing neighbors
        planning_msgs::msg::Point2D position; // Position of the node in 2D space

        double gCost; // Cost from start to current node
        double hCost; // Heuristic cost from current node to goal
        double fCost; // Total cost (f = g + h)

        int parentID; // Pointer to parent node

        Node(int id) : nodeID(id), gCost(std::numeric_limits<double>::max()), hCost(0), fCost(0), parentID(0) {}

        // Compute heuristic based on some function (this will be application-specific)
        void computeHeuristic(const Node& goal, const Node& parent) {
            gCost = parent.gCost + distance(position, parent.position);
            hCost = distance(position, goal.position);
            fCost = gCost + hCost;
        }

        bool operator==(const Node& other) const {
            return nodeID == other.nodeID;
        }

        bool operator>(const Node& other) const {
            return fCost > other.fCost;
        }

        bool operator<(const Node& other) const {
            return fCost < other.fCost;
        }
    };
} // namespace graph_search
