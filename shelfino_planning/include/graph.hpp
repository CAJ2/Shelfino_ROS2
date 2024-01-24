#ifndef GRAPH_H
#define GRAPH_H
#include <map>
#include <vector>

namespace graph
{
    /**
     * @brief Point class used for the visibility graph generation
     * 
     */
    class Point
    {
    public:
        
        /**
         * @brief X coordinate of the point
         * 
         */
        double x;

        /**
         * @brief Y coordinate of the point
         * 
         */
        double y;

        /**
         * @brief ID of the polygon that contains this point
         * 
         */
        int polygonId;

        /**
         * @brief Scale factor used for the calculations to avoid double precision problems
         * 
         */
        int SCALE_FACTOR = 1000;

        /**
        * @brief Point constructor
        * 
        * @param x coordinate
        * @param Y coordinate
        * @param polygonId the ID of the polygon
        */
        Point(double x_coor, double y_coor, int polygonId = -1);

        /**
         * @brief Prints the coordinates of the point
         * 
         */
        void print();

        /**
        * @brief Scale point by a SCALE_FACTOR.
        * In this way during the calculations for the visibility graph
        * there will be no floating point calculations errors due to the
        * fact that we are dealing with too small numbers.
        * 
        */
        void scale();

        /**
        * @brief Convert the point to the original size.
        * After having converted a point with scale(),
        * we can convert it to the original size using this function
        * 
        */
        void rescaleToOriginal();

        /**
         * @brief Override the operator <
         * 
         * @param ob Point to compare
         * @return true If this point is smaller than ob
         * @return false If ob is smaller than this point
         */
        bool operator<(const Point &ob) const;

        /**
         * @brief Override the operator ==
         * 
         * @param ob Point to compare
         * @return true If this point is equal to ob
         * @return false If this point is not equal to ob
         */
        bool operator==(const Point &ob) const;
    };

    /**
     * @brief Edge class used for the visibility graph generation
     * 
     */
    class Edge
    {
    public:
        /**
         * @brief First point of the edge
         * 
         */
        Point p1;
        
        /**
         * @brief Second point of the edge
         * 
         */
        Point p2;

        /**
        * @brief Constructor for Edge
        * 
        * @param p1 First point
        * @param p2 Second point
        */
        Edge(Point point1, Point point2);

        /**
        * @brief Returns the euclidean distance between two points
        * 
        * @param p1 A simple point (x,y)
        * @param p2 A simple point (x,y)
        * @return double Distance between two points
        */
        double weight();

        /**
        * @brief Fetches the adjacent points to a certain point
        * 
        * @param point Point we're interested in
        * @return returns the adjacent
        */
        Point getAdjacent(Point point);

        /**
        * @brief Checks if a point is part of the edge
        * 
        * @param point Point we need to find
        * @return return whether or not the point is present
        */
        bool contains(Point point);

        /**
        * @brief Prints the basic information of the edge
        * 
        */
        void print();

        /**
         * @brief Override operator ==
         * 
         * @param ob Edge to compare
         * @return true If this edge is equal to ob
         * @return false If this edge is not equal to ob
         */
        bool operator==(const Edge &ob) const;
    };

    /**
     * @brief Data structure to represent the graph in which we have a Map with as keys structure Point and as values a vector of Edge
     * 
     */
    typedef std::map<Point, std::vector<Edge>> DictG;

    /**
     * @brief Data structure to represent the polygons that are present in the arena: a Map which has as keys IDs and as values a vector of Edge
     * 
     */
    typedef std::map<int, std::vector<Edge>> DictP;

    /**
     * @brief Graph class used for the visibility graph generation
     * 
     * Used both to express a visibility graph and a normal graph of obstacles
     * We can understand what was it used for based on the isVisibilityGraph variable
     * 
     */
    class Graph
    {
    public:
        
        /**
         * @brief Maps points to edges
         * 
         */
        DictG graph;
        
        /**
         * @brief Set of edges of this graph
         * 
         */
        std::vector<Edge> edges;

        int pid;

        /**
         * @brief Maps Polygon IDs to the actual polygons
         * 
         */
        DictP polygons;
        
        /**
         * @brief Is this a visibility graph or a normal graph with some obstacles?
         * 
         */
        bool isVisibilityGraph;

        Graph() = default;

        /**
        * @brief Constructor of the graph
        * 
        * @param polygons a map of polygons, keys are IDs and values the edges
        * @returns the complete graph
        */
        Graph(std::vector<std::vector<Point>> polygons, bool isVisibilityGraph = false, bool isOriginalGraph = false);

        /**
        * @brief Computes Dijkstra Shortest path given the origin, a destination and the coordinates of the borders of the arena.
        * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
        * Complexity: O(E logV)
        * 
        * @param graph A map <Point, vector of adjacent edges>
        * @param origin The origin point in which we want to start
        * @param destination The destination we intend to reach
        * @param borderPoints Points that descrbe the borders of the arena
        * @return std::vector<Point> A map that represents the complete shortest path from origin to destination
        */
        std::vector<Point> shortestPath(Point origin, Point destination, std::vector<Point> borderPoints);

        /**
        * @brief Computes Dijkstra Shortest path and return a map that represents the complete shortest path from origin to destination.
        * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
        * Complexity: O(E logV)
        * 
        * @param graph A map <Point, vector of adjacent edges>
        * @param origin The origin point in which we want to start
        * @param destination The destination we intend to reach
        * @param borderPoints Points that describe the borders of the arena
        * @return std::map<Point, double> A map that represents the complete shortest path from origin to destination
        */
        std::map<Point, double> shortestPathDict(Point origin, Point destination, std::vector<Point> borderPoints);

        /**
        * @brief Computes Dijkstra Shortest path considering multiple destinations and return a map that represents the complete shortest path from origin to all possible destinations.
        * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
        * Complexity: O(E logV)
        * 
        * @param graph A map <Point, vector of adjacent edges>
        * @param origin The origin point in which we want to start
        * @param destinations All possible destinations we intend to reach
        * @return std::map<Point, double> A map that represents the complete shortest path from origin to destinations
        */
        std::map<Point, double> shortestPathMultipleDDict(Point origin, std::vector<Point> destinations, std::vector<Point> borderPoints);

        /**
         * @brief Computes Dijkstra Shortest path considering multiple destinations, takes into account also the border of the arena
         * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
         * Complexity: O(E logV)
         * 
         * @param graph A map <Point, vector of adjacent edges>
         * @param origin The origin point in which we want to start
         * @param destinations All possible destinations
         * @param borderPoints points of the borders of the arena
         * @return std::vector<Point> The complete shortest path from origin to destination
         */
        std::vector<Point> shortestPathMultipleD(Point origin, std::vector<Point> destinations, std::vector<Point> borderPoints);

        /**
        * @brief Returns the adjacent edges to a certain point
        * 
        * @param point the point we want to consider
        * @param edges an empty list to fill with the edges that have been found
        */
        std::vector<Point> getAdjacentPoints(Point point);

        /**
        * @brief returns a list of all the points in the map
        * 
        * @param points the empty vector of points
        */
        std::vector<Point> getPoints();

        /**
        * @brief returns all the edges
        * 
        * @param edges the empty vector of edges
        */
        std::vector<Edge> getEdges();

        /**
        * @brief adds an edge to the list of edges
        * 
        * @param edge the one we want to insert
        */
        void addEdge(Edge edge);

        /**
        * @brief get every edge associated to a point
        * 
        * @param point point we want to search
        */
        std::vector<Edge> getItems(Point point);

        int getOrientation(Point p, Point q, Point r);

        /** 
        * @brief checks if the map contains a point, if it does returns the edges of that point
        * 
        * @param point the point we need to check
        * @param edges the empty vector to fill
        */
        std::vector<Edge> containsP(Point point);

        /**
        * @brief checks if the vector of edges contains a specific edge, returns it after retrieving it
        * 
        * @param e the edge we are looking for
        */
        Edge containsE(Edge e);
    };

}

#endif
