#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include <set>
#include "graph.hpp"
#include "utils.hpp"
#include "dubins.hpp"

using namespace graph;

/* Point class, with a X and Y coordinate and an ID of the polygon */

/**
 * @brief Point constructor
 * @param x coordinate
 * @param Y coordinate
 * @param polygonId the ID of the polygon
*/
Point::Point(double x_coor, double y_coor, int polygonId) : x(x_coor), y(y_coor), polygonId(polygonId) {}

/**
 * @brief Prints the coordinates of the point
 */
void Point::print (){
    std::cout<<"Point("<< x << "," << y << "," << polygonId << ")\n";
}

/**
 * @brief Scale point by a SCALE_FACTOR.
 * In this way during the calculations for the visibility graph
 * there will be no floating point calculations errors due to the
 * fact that we are dealing with too small numbers.
 * 
 */
void Point::scale() {
    x *= SCALE_FACTOR;
    y *= SCALE_FACTOR;
}

/**
 * @brief Convert the point to the original size.
 * After having converted a point with scale(),
 * we can convert it to the original size using this function
 * 
 */
void Point::rescaleToOriginal() {
    x /= (1.0 * SCALE_FACTOR);
    y /= (1.0 * SCALE_FACTOR);
}

/**
 * @brief Override the operator <
 * 
 * @param ob Point to compare
 * @return true If this point is smaller than ob
 * @return false If ob is smaller than this point
 */
bool Point::operator<(const Point &ob) const {
    return x < ob.x || (x == ob.x && y < ob.y);
}

/**
 * @brief Override the operator ==
 * 
 * @param ob Point to compare
 * @return true If this point is equal to ob
 * @return false If this point is not equal to ob
 */
bool Point::operator==(const Point &ob) const{
    return ob.x == x && ob.y == y && ob.polygonId == polygonId;
}
/**
 * @brief Constructor for Edge
 * @param p1 First point
 * @param p2 Second point
 */
Edge::Edge(const Point point1, const Point point2): p1(point1) , p2(point2) {}

/**
 * @brief Fetches the adjacent points to a certain point
 * @param point Point we're interested in
 * @return returns the adjacent
 */
Point Edge::getAdjacent(Point point){
    if(point == p1){
        return p2;
    }
    return p1;
}

/**
 * @brief Checks if a point is part of the edge
 * @param point Point we need to find
 * @return return whether or not the point is present
 */
bool Edge::contains (Point point){
    return p1 == point || p2 == point;
}

/**
 * @brief Returns the euclidean distance between two points
 * 
 * @param p1 A simple point (x,y)
 * @param p2 A simple point (x,y)
 * @return double Distance between two points
 */
double Edge::weight() {
    return sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
}

/**
 * @brief Prints the basic information of the edge
 */
void Edge::print (){
    std::cout << "Edge((" << p1.x << "," << p1.y << ")(" << p2.x << "," << p2.y << "))\n";
}

/**
 * @brief Override operator ==
 * 
 * @param ob Edge to compare
 * @return true If this edge is equal to ob
 * @return false If this edge is not equal to ob
 */
bool Edge::operator==(const Edge &ob) const{
    return ob.p1 == p1 && ob.p2 == p2;
}
/**
 * @brief Constructor of the graph
 * @param polygons a map of polygons, keys are IDs and values the edges
 * @returns the complete graph
 */
Graph::Graph (std::vector<std::vector<Point>> shapes, bool isVisGraph, bool isOriginalGraph){
    isVisibilityGraph = isVisGraph; // Remember if we are dealing with a visibility graph
    if (!isVisibilityGraph && !isOriginalGraph) {
        // Work with enlarged numbers by a factor ENLARGE_FACTOR
        // In this way during the computation of the visibility graph,
        // there will be no floating point calculations errors due to too small numbers
        for(int i = 0; i < shapes.size(); i++) {
            for(int j = 0; j < shapes[i].size(); j++) {
                shapes[i][j].scale();
            }
        }
    }
    pid = 0;
    for(std::vector<Point> it : shapes){
        // If the last point coincides with the first one, remove it
        if(it[0] == it[it.size()-1] && it.size() > 1){
            it.pop_back();
        }
        int count = 0;
        for(int i =  0; i < it.size(); i++){
            // Create the edges and the polygons looping through their points
            // And keeping a polygonID
            Point siblingPoint = it[(count + 1)%(it.size())]; 
            if((it.size()) > 2){
                it[i].polygonId = pid;
                siblingPoint.polygonId = pid;
                polygons[pid].push_back(Edge(it[i], siblingPoint));
            }
            addEdge(Edge(it[i], siblingPoint)); 
            count++;
        }
        if(it.size() > 2){
            pid += 1;
        }
    }
}

/**
 * @brief returns the adjacent edges to a certain point
 * @param point the point we want to consider
 * @param edges an empty list to fill with the edges that have been found
 */
std::vector<Point> Graph::getAdjacentPoints(Point point){
    std::vector<Point> points;
    for(Edge e : graph[point]){
        Point p1 = e.getAdjacent(point);
        points.push_back(p1); 
    }
    return points;
}

/**
 * @brief returns a list of all the points in the map
 * @param points the empty vector of points
 */
std::vector<Point> Graph::getPoints(){
    std::vector<Point> points;
    std::map<Point, std::vector<Edge>>::iterator it;
    for(it = graph.begin(); it != graph.end(); it++){
        points.push_back(it->first);
    }
    return points;
}

/**
 * @brief returns all the edges
 * @param edges the empty vector of edges
 */
std::vector<Edge> Graph::getEdges(){
    std::vector<Edge> results;
    for(Edge e : edges){
        results.push_back(e);
    }
    return results;
}

/**
 * @brief adds an edge to the list of edges
 * @param edge the one we want to insert
 */
void Graph::addEdge(Edge edge){
    // If we are adding an edge to the resulting visibility graph,
    // rescale the size of the points to the original one
    if (isVisibilityGraph) {
        edge.p1.rescaleToOriginal();
        edge.p2.rescaleToOriginal();
    }
    graph[edge.p1].push_back(edge);
    graph[edge.p2].push_back(edge);
    edges.push_back(edge);
}

/**
 * @brief get every edge associated to a point
 * @param point point we want to search
 */
std::vector<Edge> Graph::getItems(Point point){
    std::vector<Edge> edges;
    if(graph.find(point) != graph.end()){
        for(Edge e : graph[point]){
            edges.push_back(e);
        }
    }
    return edges;
}

/** 
 * @brief checks if the map contains a point, if it does returns the edges of that point
 * @param point the point we need to check
 * @param edges the empty vector to fill
 */
std::vector<Edge> Graph::containsP(Point point){
    std::vector<Edge> edges;
    for(Edge e : graph[point]){
        edges.push_back(e);
    }
    return edges;
}

/**
 * @brief checks if the vector of edges contains a specific edge, returns it after retrieving it
 * @param e the edge we are looking for
 */
Edge Graph::containsE (Edge e) {
    if(std::find(edges.begin(), edges.end(), e) != edges.end()){
        std::vector<Edge>::iterator it = std::find(edges.begin(), edges.begin(), e);
        int index = std::distance(edges.begin(), it);
        Edge edge = edges.at(index);
        return edge;
    }
    return Edge(Point(-1, -1), Point(-1, -1));
}

/**
 * @brief Check if a certain points (x,y) is inside the arena
 * Points are in the order bottom-left, bottom-right, top-right, top-left
 * 
 * @param borderPoints Points representing the arena, in order bottom-left, bottom-right, top-right, top-left
 * @param xPoint X of the point we are considering
 * @param yPoint Y of the point we are considering
 * @return true If the point lies inside the arena
 * @return false If the point does not lie inside the arena
 */
bool isInsideArena(std::vector<graph::Point> borderPoints, double xPoint, double yPoint) {
return (xPoint >= borderPoints[0].x && xPoint <= borderPoints[1].x && yPoint >= borderPoints[0].y && yPoint <= borderPoints[2].y);
}

/**
 * @brief Dijkstra Shortest path considering multiple destinations, takes into account also the border of the arena.
 * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
 * Complexity: O(E logV)
 * 
 * @param graph A map <Point, vector of adjacent edges>
 * @param origin The origin point in which we want to start
 * @param destinations All possible destinations
 * @param borderPoints points of the borders of the arena
 * @return std::vector<Point> The complete shortest path from origin to destination
 */
std::vector<Point> Graph::shortestPathMultipleD(Point origin, std::vector<Point> destinations, std::vector<Point> borderPoints) 
{
    // Check the input
    if(graph.find(origin) == graph.end()) {
        std::cout << "ERROR: origin point not available in graph!" << std::endl;
        std::vector<Point> path;
        return path;
    }

    // Create a map to store the distances for all points
    std::map<Point, double> dist;
    // Create a map to reconstruct the final solution
    std::map<Point, Point> prev;
    // Create a set to store vertices that are being processed
    std::set<std::pair<double, Point>> setds;

    // Insert the origin point to start the algorithm
    setds.insert(std::make_pair(0, origin));
    dist[origin] = 0;

    // Loop until all shortest distances are finalized
    while(!setds.empty()) {
        // First vertex in the set - the min distance vertex
        std::pair<double, Point> tmp = *(setds.begin());
        setds.erase(setds.begin());

        Point u = tmp.second;

        // Loop through all adjacent vertices of the point u
        for (Edge e : graph[u]) {
            Point v = e.getAdjacent(u);
            // The weight is simply the distance between the points
            double weight = e.weight();

            // If the point is not inside the arena, put weight INFINITY so Dijkstra won't use it
            if (!isInsideArena(borderPoints, v.x, v.y)) {
                weight = INFINITY;
            }

            // If there is a shortest path from v through u (or it's the first one we find)
            if (!dist.count(v) || dist[v] > dist[u]+weight) {
                // If we already had a distance from v through u, erase it (then we will insert the new one)
                if (dist.count(v)) {
                    setds.erase(setds.find(std::make_pair(dist[v], v)));
                    prev.erase(prev.find(v));
                }
                
                // Update the distance from the point v
                dist[v] = dist[u] + weight;
                setds.insert(std::make_pair(dist[v], v));
                prev.insert(std::make_pair(v,u));
            }
        }
    }

    // Check the best destination available
    Point bestDestination = Point(-1, -1);
    double shortestDistance = INFINITY;
    for (int i = 0; i < destinations.size(); i++) {
        if (dist[destinations[i]] < shortestDistance) {
            shortestDistance = dist[destinations[i]];
            bestDestination = destinations[i];
        }
    }
    // Reconstruct the path using prev and starting from the best destination found
    std::vector<Point> path;
    while(true){
        path.push_back(bestDestination);
        if(bestDestination == origin) break;
        bestDestination = prev.find(bestDestination)->second;
    }
    // Reverse the path
    reverse(path.begin(), path.end());

    return path;
}

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
std::vector<Point> Graph::shortestPath(Point origin, Point destination, std::vector<Point> borderPoints) {
    // Checks the input by verifying if the origin and the destination are part of the graph
    if(graph.find(origin) == graph.end() || graph.find(destination) == graph.end()) {
        std::cout << "ERROR: origin point or destination point not available in graph!" << std::endl;
        std::vector<Point> path;
        return path;
    }

    // Create a map to store the distances for all points
    std::map<Point, double> dist;
    // Create a map to reconstruct the final solution
    std::map<Point, Point> prev;
    // Create a set to store vertices that are being processed
    std::set<std::pair<double, Point>> setds;

    // Insert the origin point to start the algorithm
    setds.insert(std::make_pair(0, origin));
    dist[origin] = 0;

    // Loop until all shortest distances are finalized (or until the destination is found)
    while(!setds.empty()) {
        // First vertex in the set - the min distance vertex
        std::pair<double, Point> tmp = *(setds.begin());
        setds.erase(setds.begin());

        Point u = tmp.second;

        // If we reached the destination, we can stop
        if(u == destination) {
            break;
        }

        // Loop through all adjacent vertices of the point u
        for (Edge e : graph[u]) {
            Point v = e.getAdjacent(u);
            // The weight is simply the distance between the points
            double weight = e.weight();

            // If the point is not inside the arena, put weight INFINITY so Dijkstra won't use it
            if (!isInsideArena(borderPoints, v.x, v.y)) {
                weight = INFINITY;
            }

            // If there is a shortest path from v through u (or it's the first one we find)
            if (!dist.count(v) || dist[v] > dist[u]+weight) {
                // If we already had a distance from v through u, erase it (then we will insert the new one)
                if (dist.count(v)) {
                    setds.erase(setds.find(std::make_pair(dist[v], v)));
                    prev.erase(prev.find(v));
                }
                
                // Update the distance from the point v
                dist[v] = dist[u] + weight;
                setds.insert(std::make_pair(dist[v], v));
                prev.insert(std::make_pair(v,u));
            }
        }
    }

    // Reconstruct the path using prev. Start from destination
    std::vector<Point> path;
    while(true){
        path.push_back(destination);
        if(destination == origin) break;
        destination = prev.find(destination)->second;
    }
    // Reverse the path
    reverse(path.begin(), path.end());

    return path;
}

/**
 * @brief Computes Dijkstra Shortest path and return a map that represents the complete shortest path from origin to destination.
 * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
 * Complexity: O(E logV)
 * 
 * @param graph A map <Point, vector of adjacent edges>
 * @param origin The origin point in which we want to start
 * @param destination Our destination
 * @param borderPoints Points of the borders of the arena
 * @return std::map<Point, double> The complete shortest path from origin to destination
 */
std::map<Point, double> Graph::shortestPathDict(Point origin, Point destination, std::vector<Point> borderPoints) {
    // Check the input by verifying if the origin and the destination are part of the graph
    if(graph.find(origin) == graph.end() || graph.find(destination) == graph.end()) {
        std::cout << "ERROR: origin point or destination point not available in graph!" << std::endl;
        std::map<Point, double> dist;
        return dist;
    }

    // Create a map to store the distances for all points
    std::map<Point, double> dist;
    // Create a map to reconstruct the final solution
    std::map<Point, Point> prev;
    // Create a set to store vertices that are being processed
    std::set<std::pair<double, Point>> setds;

    // Insert the origin point to start the algorithm
    setds.insert(std::make_pair(0, origin));
    dist[origin] = 0;

    // Loop until all shortest distances are finalized (or until the destination is found)
    while(!setds.empty()) {
        // First vertex in the set - the min distance vertex
        std::pair<double, Point> tmp = *(setds.begin());
        setds.erase(setds.begin());

        Point u = tmp.second;

        // If we reached the destination, we can stop
        if(u == destination) {
            break;
        }

        // Loop through all adjacent vertices of the point u
        for (Edge e : graph[u]) {
            Point v = e.getAdjacent(u);
            // The weight is simply the distance between the points
            double weight = e.weight();

            // If the point is not inside the arena, put weight INFINITY so Dijkstra won't use it
            if (!isInsideArena(borderPoints, v.x, v.y)) {
                weight = INFINITY;
            }

            // If there is a shortest path from v through u (or it's the first one we find)
            if (!dist.count(v) || dist[v] > dist[u]+weight) {
                // If we already had a distance from v through u, erase it (then we will insert the new one)
                if (dist.count(v)) {
                    setds.erase(setds.find(std::make_pair(dist[v], v)));
                    prev.erase(prev.find(v));
                }
                
                // Update the distance from the point v
                dist[v] = dist[u] + weight;
                setds.insert(std::make_pair(dist[v], v));
                prev.insert(std::make_pair(v,u));
            }
        }
    }

    return dist;
}

/**
 * @brief Computes Dijkstra Shortest path considering multiple destinations and return a map that represents the complete shortest path from origin to all possible destinations.
 * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
 * Complexity: O(E logV)
 * 
 * @param graph A map <Point, vector of adjacent edges>
 * @param origin The origin point in which we want to start
 * @param destinations All possible destinations
 * @param borderPoints Points of the borders of the arena
 * @return std::map<Point, double> The complete shortest path from origin to destination
 */
std::map<Point, double> Graph::shortestPathMultipleDDict(Point origin, std::vector<Point> destinations, std::vector<Point> borderPoints) 
{
    // Check the input by checking if the position of the origin point
    if(graph.find(origin) == graph.end()) {
        std::cout << "ERROR: origin point not available in graph!" << std::endl;
        std::map<Point, double> dist;
        return dist;
    }

    // Create a map to store the distances for all points
    std::map<Point, double> dist;
    
    // Create a set to store vertices that are being processed
    std::set<std::pair<double, Point>> setds;

    // Insert the origin point to start the algorithm
    setds.insert(std::make_pair(0, origin));
    dist[origin] = 0;

    // Loop until all shortest distances are finalized
    while(!setds.empty()) {
        // First vertex in the set - the min distance vertex
        std::pair<double, Point> tmp = *(setds.begin());
        setds.erase(setds.begin());

        Point u = tmp.second;

        // Loop through all adjacent vertices of the point u
        for (Edge e : graph[u]) {
            Point v = e.getAdjacent(u);
            // The weight is simply the distance between the points
            double weight = e.weight();

            // If the point is not inside the arena, put weight INFINITY so Dijkstra won't use it
            if (!isInsideArena(borderPoints, v.x, v.y)) {
                weight = INFINITY;
            }

            // If there is a shortest path from v through u (or it's the first one we find)
            if (!dist.count(v) || dist[v] > dist[u]+weight) {
                // If we already had a distance from v through u, erase it (then we will insert the new one)
                if (dist.count(v)) {
                    setds.erase(setds.find(std::make_pair(dist[v], v)));
                }
                
                // Update the distance from the point v
                dist[v] = dist[u] + weight;
                setds.insert(std::make_pair(dist[v], v));
            }
        }
    }

    return dist;
}
