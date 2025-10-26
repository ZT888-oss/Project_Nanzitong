/* 
 * Copyright 2025 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include "m1.h"
#include "m2.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <math.h>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <string>
using namespace std;

std::unordered_map<OSMID, int> osm_id_to_index_map; // Maps OSMID to its index
std::unordered_map<OSMID, double> way_length_cache; // Maps way_id to its length
std::unordered_map<OSMID, std::unordered_map<std::string, std::string>> osm_node_tag_cache; // Cache for node tags
struct Node {
    vector<StreetSegmentIdx> outgoingEdges;
    StreetSegmentIdx reachingEdge;
    double bestTime;
    bool visited;

    Node() : reachingEdge(-1), bestTime(numeric_limits<double>::max()), visited(false) {}
};
vector<vector<StreetSegmentIdx>> intersection_street_segments;
vector<unordered_set<IntersectionIdx>> street_intersections;
vector<unordered_set<StreetSegmentIdx>> street_segments_of_street;
vector<double> street_segment_travel_time;
vector<StreetSegmentInfo> street_segment_info;
vector<pair<string, StreetIdx>> sorted_street_names;
vector<LatLon> intersection_positions;
vector<Node> nodes;
#include <algorithm>
#include <cctype>
int max_speed = 0;


using namespace std;

string toLowercaseAndRemoveWhitespace(const string& input) {
    string result = input;

    transform(result.begin(), result.end(), result.begin(),
              [](unsigned char c) { return tolower(c); });

    result.erase(remove_if(result.begin(), result.end(),
                           [](unsigned char c) { return isspace(c); }),
                 result.end());

    return result;
}
vector<vector<OSMID>> subway_osm_ids;


// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection 
// data that is higher-level than the raw OSM data). 
// This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.
double findStreetSegmentLength(StreetSegmentIdx street_segment_id);

vector<const OSMNode*> NodeKeys;
std::unordered_map<OSMID, std::vector<const OSMNode*>> IdentifyNodefromID;
// List of valid road classifications
std::vector<std::string> way_classifications = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "service", "footway"
};

// Map to store OSMID -> Road Classification
std::unordered_map<OSMID, std::string> osm_id_to_classification;
vector<vector<OSMID>> bike_lane_ways;

void preprocessWayClassifications() {
    for (int i = 0; i < getNumberOfWays(); ++i) {
        const OSMWay* way = getWayByIndex(i);
        OSMID wayID = way->id();

        // Get the list of nodes in this way
        std::vector<OSMID> way_nodes = getWayMembers(way);

        // Compute the length of the way
        double way_length = 0.0;
        if (!way_nodes.empty()) {
            LatLon prev_coords = getNodeCoords(getNodeByIndex(osm_id_to_index_map[way_nodes[0]]));
            for (size_t j = 1; j < way_nodes.size(); ++j) {
                LatLon curr_coords = getNodeCoords(getNodeByIndex(osm_id_to_index_map[way_nodes[j]]));
                way_length += findDistanceBetweenTwoPoints(prev_coords, curr_coords);
                prev_coords = curr_coords;
            }
        }

        // Store the computed length in the cache
        way_length_cache[wayID] = way_length;

        // Check if the way has a "cycleway" or "subway" tag
        for (int j = 0; j < getTagCount(way); ++j) {
            std::pair<std::string, std::string> p = getTagPair(way, j);
            if (p.second == "cycleway") {
                bike_lane_ways.push_back(way_nodes);
            }
            if (p.second == "subway") {
                subway_osm_ids.push_back(way_nodes);
            }

            // Check if p.second exists in the way_classifications list
            if (std::find(way_classifications.begin(), way_classifications.end(), p.second) != way_classifications.end()) {
                osm_id_to_classification[wayID] = p.second;
                break; // No need to check further tags for this way
            }
        }
    }

    // Populate osm_node_tag_cache
    for (int i = 0; i < getNumberOfNodes(); ++i) {
        const OSMNode* node = getNodeByIndex(i);
        OSMID node_id = node->id();

        // Cache all tags for this node
        std::unordered_map<std::string, std::string> tag_cache;
        int num_tags = getTagCount(node);
        for (int j = 0; j < num_tags; ++j) {
            std::pair<std::string, std::string> tag = getTagPair(node, j);
            tag_cache[tag.first] = tag.second;
        }

        osm_node_tag_cache[node_id] = tag_cache;
    }
}


bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
    string maps_osm = map_streets_database_filename;
    size_t pos = maps_osm.find("streets.bin");
    bool load_OSM = false;
    if (pos != string::npos) {
        maps_osm.replace(pos, 11, "osm.bin");
        load_OSM = loadOSMDatabaseBIN(maps_osm);
    }

    if (!(load_successful && load_OSM)) {
        return false;
    }

    preprocessWayClassifications();

    int number_intersections = getNumIntersections();
    int number_street_segments = getNumStreetSegments();
    int number_streets = getNumStreets();

    intersection_street_segments.resize(number_intersections);
    intersection_positions.resize(number_intersections);

    for (int intersection = 0; intersection < number_intersections; ++intersection) {
        int num_segments = getNumIntersectionStreetSegment(intersection);
        for (int i = 0; i < num_segments; ++i) {
            int ss_id = getIntersectionStreetSegment(intersection, i);
            intersection_street_segments[intersection].push_back(ss_id);
        }
        intersection_positions[intersection] = getIntersectionPosition(intersection);
    }

    street_segment_travel_time.resize(number_street_segments);
    street_segment_info.resize(number_street_segments);
    street_intersections.resize(number_streets);
    street_segments_of_street.resize(number_streets);
    nodes.resize(number_intersections);
    for (int street_segment = 0; street_segment < number_street_segments; ++street_segment) {
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(street_segment);
        street_segment_info[street_segment] = segmentInfo;
        double distance = findStreetSegmentLength(street_segment);
        street_segment_travel_time[street_segment] = (distance / segmentInfo.speedLimit);
        int street_id = segmentInfo.streetID;
        if (segmentInfo.speedLimit>max_speed){
            max_speed = segmentInfo.speedLimit;
        }
            street_intersections[street_id].insert(segmentInfo.from);
            street_intersections[street_id].insert(segmentInfo.to);
            street_segments_of_street[street_id].insert(street_segment);
             // Populate the outgoing edges for each node
            // Add edge to the 'from' intersection
            nodes[segmentInfo.from].outgoingEdges.push_back(street_segment);
            // // Add edge to the 'to' intersection if the street is not one-way
            // if (!segmentInfo.oneWay) {
            //     nodes[segmentInfo.to].outgoingEdges.push_back(street_segment);
            // }
    }
    

    std::vector<std::pair<std::string, int>> temp_street_names;
    temp_street_names.reserve(number_streets);
    street_intersections.resize(number_streets);
    for (int street = 0; street < number_streets; ++street) {
        std::string name = toLowercaseAndRemoveWhitespace(getStreetName(street));
        temp_street_names.emplace_back(name, street);
    }

    std::sort(temp_street_names.begin(), temp_street_names.end());
    sorted_street_names.swap(temp_street_names);

    cout << "done loading" << endl;
    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    closeStreetDatabase();
    closeOSMDatabase();
}
// Returns the distance between two (latitude, longitude) coordinates in meters.
// Speed Requirement --> moderate
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){//serewaya
    double lat1 = point_1.latitude() * kDegreeToRadian;
    double lon1 = point_1.longitude() * kDegreeToRadian;
    double lat2 = point_2.latitude() * kDegreeToRadian;
    double lon2 = point_2.longitude() * kDegreeToRadian;
    double latAvg = (lat1 + lat2) / 2.0;

    double x1 = kEarthRadiusInMeters * lon1 * cos(latAvg);
    double y1 = kEarthRadiusInMeters * lat1;
    double x2 = kEarthRadiusInMeters * lon2 * cos(latAvg);
    double y2 = kEarthRadiusInMeters * lat2;
    double distance = sqrt(((x2-x1)*(x2-x1)) +((y2-y1)*(y2-y1)));
    return distance;
}

// Returns the length of the given street segment in meters.
// Speed Requirement --> moderate
double findStreetSegmentLength(StreetSegmentIdx street_segment_id){//nithura
    StreetSegmentInfo segmentInfo = getStreetSegmentInfo(street_segment_id);
    if (segmentInfo.numCurvePoints==0){
        return findDistanceBetweenTwoPoints(getIntersectionPosition(segmentInfo.to), getIntersectionPosition(segmentInfo.from));
    }
    LatLon start = getIntersectionPosition(segmentInfo.from);
    LatLon end = getIntersectionPosition(segmentInfo.to);
    double d = 1;
    LatLon point = start;
    for (int i=0; i<segmentInfo.numCurvePoints; i++){
        d += findDistanceBetweenTwoPoints(point, getStreetSegmentCurvePoint(street_segment_id, i));
        point = getStreetSegmentCurvePoint(street_segment_id, i);
    }
    d += findDistanceBetweenTwoPoints(end, getStreetSegmentCurvePoint(street_segment_id, segmentInfo.numCurvePoints-1));
    return d-1;
}

// Returns the travel time to drive from one end of a street segment
// to the other, in seconds, when driving at the speed limit.
// Note: (time = distance/speed_limit)
// Speed Requirement --> high
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    return street_segment_travel_time[street_segment_id];
}

// Returns the angle (in radians) that you would need to turn as you exit
// src_street_segment_id and enter dst_street_segment_id, if they share an
// intersection.
// If a street segment is not completely straight, use the last piece of the
// segment closest to the shared intersection.
// If the two street segments do not share an intersection, return a constant
// NO_ANGLE, which is defined above.
// Speed Requirement --> none

vector<double> getDirectionVector(LatLon start, LatLon end) {
    vector<double> direction;
    double lat1 = start.latitude() * kDegreeToRadian;
    double lon1 = start.longitude() * kDegreeToRadian;
    double lat2 = end.latitude() * kDegreeToRadian;
    double lon2 = end.longitude() * kDegreeToRadian;
    double latAvg = (lat1 + lat2) / 2.0;

    double x1 = kEarthRadiusInMeters * lon1 * cos(latAvg);
    double y1 = kEarthRadiusInMeters * lat1;
    double x2 = kEarthRadiusInMeters * lon2 * cos(latAvg);
    double y2 = kEarthRadiusInMeters * lat2;

    double dx = x2 - x1;
    double dy = y2 - y1;

    direction.push_back(dx);
    direction.push_back(dy);
    return direction;
}

double getAngle(LatLon a, LatLon b, LatLon intersection) {
    vector<double> vector1 = getDirectionVector(a, intersection);
    vector<double> vector2 = getDirectionVector(b, intersection);

    if (vector1.size() < 2 || vector2.size() < 2) {
        return NO_ANGLE;
    }

    double magnitude1 = sqrt((vector1[0] * vector1[0]) + (vector1[1] * vector1[1]));
    double magnitude2 = sqrt((vector2[0] * vector2[0]) + (vector2[1] * vector2[1]));

    if (magnitude1 == 0 || magnitude2 == 0) {
        return NO_ANGLE;
    }

    double dotProduct = (-vector1[0] * vector2[0]) + (-vector1[1] * vector2[1]);
    double cosTheta = dotProduct / (magnitude1 * magnitude2);
    
    cosTheta = max(-1.0, min(1.0, cosTheta));

    double rawAngle = acos(cosTheta);

    return rawAngle;
}



double findStreetSegmentTurnAngle(StreetSegmentIdx src_street_segment_id,
                                  StreetSegmentIdx dst_street_segment_id) {
    StreetSegmentInfo sourceStreetInfo = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo destinationStreetInfo = getStreetSegmentInfo(dst_street_segment_id);
    LatLon a;
    LatLon b;
    IntersectionIdx intersectionId = -1;
    if (destinationStreetInfo.from == sourceStreetInfo.from ||
        destinationStreetInfo.from == sourceStreetInfo.to) {
        intersectionId = destinationStreetInfo.from;
    } else if (destinationStreetInfo.to == sourceStreetInfo.from ||
               destinationStreetInfo.to == sourceStreetInfo.to) {
        intersectionId = destinationStreetInfo.to;
    }

    if (intersectionId == -1) {
        return NO_ANGLE;
    }

    LatLon IntersectionPosition = getIntersectionPosition(intersectionId);
    if (IntersectionPosition==getIntersectionPosition(destinationStreetInfo.from)){
        b= getIntersectionPosition(destinationStreetInfo.to);
    }else{
        b=getIntersectionPosition(destinationStreetInfo.from);
    }
    if (IntersectionPosition==getIntersectionPosition(sourceStreetInfo.from)){
        a= getIntersectionPosition(sourceStreetInfo.to);
    }else{
        a=getIntersectionPosition(sourceStreetInfo.from);
    }
    if (sourceStreetInfo.numCurvePoints!=0){
        if (IntersectionPosition==getIntersectionPosition(sourceStreetInfo.from)){
            a = getStreetSegmentCurvePoint(src_street_segment_id,0 );
        }else{
           a = getStreetSegmentCurvePoint(src_street_segment_id,sourceStreetInfo.numCurvePoints-1);
        }
    }if (destinationStreetInfo.numCurvePoints!=0){
        if (IntersectionPosition==getIntersectionPosition(destinationStreetInfo.from)){
            b = getStreetSegmentCurvePoint(dst_street_segment_id,0 );
        }else{
           b = getStreetSegmentCurvePoint(dst_street_segment_id,destinationStreetInfo.numCurvePoints-1);
        }
    }
    double angle = getAngle(a, b, IntersectionPosition);
    return angle;
}




// Returns all intersections reachable by traveling down one street segment
// from the given intersection. (hint: you can't travel the wrong way on a 1-way
// street)
// The returned vector should NOT contain duplicate intersections.
// Corner case: cul-de-sacs can connect an intersection to itself (from and to
// intersection on street segment are the same). In that case include the
// intersection in the returned vector (no special handling needed).
// Speed Requirement --> high
std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id) {
    vector<StreetSegmentIdx> street_segments = findStreetSegmentsOfIntersection(intersection_id);
    vector<IntersectionIdx> intersection_point;

    for (int i = 0; i < street_segments.size(); i++) {
        StreetSegmentInfo info = street_segment_info[street_segments[i]];
        IntersectionIdx start = info.from;
        IntersectionIdx end = info.to;
        if (start != intersection_id) {
            if (!info.oneWay){
                if (std::find(intersection_point.begin(), intersection_point.end(), start) == intersection_point.end()) {
                    intersection_point.push_back(start);
                }
            }
        }

        // Check if end is not equal to intersection_id and not already in the vector
        if (end != intersection_id) {
            if (std::find(intersection_point.begin(), intersection_point.end(), end) == intersection_point.end()) {
                intersection_point.push_back(end);
            }
        }
    }

    return intersection_point;
}


// Returns the geographically nearest intersection (i.e. as the crow flies) to
// the given position.
// Speed Requirement --> none


IntersectionIdx findClosestIntersection(LatLon my_position){
    double max = 10000000000;
    IntersectionIdx closest_Intersection;
    for(int i = 0; i < getNumIntersections(); i++){
        LatLon intersection_position = getIntersectionPosition(i);
        double distance = findDistanceBetweenTwoPoints(my_position, intersection_position);
        if (distance < max){
            max = distance;
            closest_Intersection = i;
        }
    }
   return closest_Intersection;
}

// Returns the street segments that connect to the given intersection.
// Speed Requirement --> high
std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
    vector <StreetSegmentIdx> empty;
    return intersection_street_segments[intersection_id];
}

// Returns all intersections along the given street.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
    const auto& intersectionsSet = street_intersections[street_id];

    // Reserve space in the vector to avoid reallocations
    std::vector<IntersectionIdx> intersections;
    intersections.reserve(intersectionsSet.size());

    // Copy elements from the unordered_set to the vector
    intersections.assign(intersectionsSet.begin(), intersectionsSet.end());

    return intersections;
}

// Return all intersection ids at which the two given streets intersect.
// This function will typically return one intersection id for streets that
// intersect and a length 0 vector for streets that do not. For unusual curved
// streets it is possible to have more than one intersection at which two
// streets cross.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high

std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids) {
    // Retrieve the intersections for the two streets
    const auto& intersectionsStreet1 = street_intersections[street_ids.first];
    const auto& intersectionsStreet2 = street_intersections[street_ids.second];

    // Use the smaller set for iteration to minimize lookups
    const auto& smallerSet = (intersectionsStreet1.size() <= intersectionsStreet2.size()) 
                             ? intersectionsStreet1 : intersectionsStreet2;
    const auto& largerSet = (intersectionsStreet1.size() > intersectionsStreet2.size()) 
                            ? intersectionsStreet1 : intersectionsStreet2;

    // Reserve space in the result vector for efficiency
    std::vector<IntersectionIdx> commonIntersections;
    commonIntersections.reserve(smallerSet.size());

    // Find common elements using hash-based lookup
    for (const auto& intersection : smallerSet) {
        if (largerSet.find(intersection) != largerSet.end()) {
            commonIntersections.push_back(intersection);
        }
    }

    return commonIntersections;
}

// Returns all street ids corresponding to street names that start with the
// given prefix.
// The function should be case-insensitive to the street prefix.
// The function should ignore spaces.
//  For example, both "bloor " and "BloOrst" are prefixes to 
//  "Bloor Street East".
// If no street names match the given prefix, this routine returns an empty
// (length 0) vector.
// You can choose what to return if the street prefix passed in is an empty
// (length 0) string, but your program must not crash if street_prefix is a
// length 0 string.
// Speed Requirement --> high
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){ //nithura
    string normalized_prefix = toLowercaseAndRemoveWhitespace(street_prefix);
    vector<StreetIdx> street_ids;
    unordered_set<StreetIdx> unique_ids;

    if (normalized_prefix.empty()) return street_ids;

    auto it = lower_bound(sorted_street_names.begin(), sorted_street_names.end(),
                          make_pair(normalized_prefix, 0));

    while (it != sorted_street_names.end() && it->first.find(normalized_prefix) == 0) {
        unique_ids.insert(it->second);
        ++it;
    }

    street_ids.assign(unique_ids.begin(), unique_ids.end());
    return street_ids;
}


// Returns the length of a given street in meters.
// Speed Requirement --> high
double findStreetLength(StreetIdx street_id){ //nithura
    unordered_set<StreetSegmentIdx> street_segments = street_segments_of_street[street_id];
    double length = 0;
    for (const auto& segment : street_segments) {
        length += findStreetSegmentLength(segment);
    }
    return length;
}

// Return the smallest axis-aligned rectangle that contains all the
// intersections and curve points of the given street (i.e. the min,max
// lattitude and longitude bounds that can just contain all points of the
// street).
// Speed Requirement --> none
LatLonBounds findStreetBoundingBox(StreetIdx street_id) {
    // Get the list of street segments for the given street ID
    unordered_set<StreetSegmentIdx> street_segments = street_segments_of_street[street_id];
    
    // If there are no segments, return an empty or invalid bounding box
    if (street_segments.empty()) {
        return {LatLon(), LatLon()}; // Return default LatLon objects, assuming they represent an invalid/empty state
    }

    // Initialize min/max latitude and longitude values
    double min_lat = std::numeric_limits<double>::max();
    double max_lat = std::numeric_limits<double>::lowest();
    double min_long = std::numeric_limits<double>::max();
    double max_long = std::numeric_limits<double>::lowest();

    // Iterate over all segments of the street
    for (StreetSegmentIdx segment : street_segments) {
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segment);

        // Get the start and end points of the segment
        LatLon start = getIntersectionPosition(segmentInfo.from);
        LatLon end = getIntersectionPosition(segmentInfo.to);

        // Update min/max latitude and longitude for the start and end points
        min_lat = std::min({min_lat, start.latitude(), end.latitude()});
        max_lat = std::max({max_lat, start.latitude(), end.latitude()});
        min_long = std::min({min_long, start.longitude(), end.longitude()});
        max_long = std::max({max_long, start.longitude(), end.longitude()});

        // Process curve points if they exist
        if (segmentInfo.numCurvePoints > 0) {  // Check if there are curve points
            for (int j = 0; j < segmentInfo.numCurvePoints; j++) {
                LatLon curve_pos = getStreetSegmentCurvePoint(segment, j);
                min_lat = std::min(min_lat, curve_pos.latitude());
                max_lat = std::max(max_lat, curve_pos.latitude());
                min_long = std::min(min_long, curve_pos.longitude());
                max_long = std::max(max_long, curve_pos.longitude());
            }
        }
    }

    // Return the calculated bounding box
    return {LatLon(min_lat, min_long), LatLon(max_lat, max_long)};
}
// Speed Requirement --> none 
POIIdx findClosestPOI(LatLon my_position, std::string poi_type){ //nithura
    int distance=1000000;
    int id=0;
    double d;
    //convert to x,y for current position
    for(int i=0;i<getNumPointsOfInterest();i++){
        if (getPOIType(i)==poi_type){
            LatLon second = getPOIPosition(i);
            d= findDistanceBetweenTwoPoints(my_position,second);
            if (d<distance){
                distance =d;
                id = i;
            }
        }
    }return id;
}

// Returns the area of the given closed feature in square meters.
// Assume a non self-intersecting polygon (i.e. no holes).
// Return 0 if this feature is not a closed polygon.
// Speed Requirement --> moderate

double calculateEnclosedArea(const std::vector<std::vector<double>>& directionVectors) {
    double currentX = 0.0, currentY = 0.0; 
    double prevX = 0.0;      
    double area = 0.0;

    for (const auto& vec : directionVectors) {
        double nextX = currentX + vec[0]; 
        double nextY = currentY + vec[1]; 

        area += (currentX * nextY - currentY * nextX);

        currentX = nextX;
        currentY = nextY;
    }

    return std::abs(area) / 2.0;
}

double findFeatureArea(FeatureIdx feature_id) {
    int num_feature_point = getNumFeaturePoints(feature_id);
    if (num_feature_point < 3) {
        return 0;
    }

    LatLon firstPoint = getFeaturePoint(feature_id, 0);
    LatLon lastPoint = getFeaturePoint(feature_id, num_feature_point - 1);

    if (firstPoint.latitude() != lastPoint.latitude() || 
        firstPoint.longitude() != lastPoint.longitude()) {
        return 0; 
    }

    std::vector<std::vector<double>> dvectors;
    dvectors.reserve(num_feature_point); 

    LatLon prevPoint = firstPoint;
    for (int i = 1; i < num_feature_point; ++i) {
        LatLon currPoint = getFeaturePoint(feature_id, i);
        std::vector<double> dirVec = getDirectionVector(prevPoint, currPoint);
        dvectors.push_back(dirVec);
        prevPoint = currPoint;
    }

    std::vector<double> closingVec = getDirectionVector(prevPoint, firstPoint);
    dvectors.push_back(closingVec);

    return calculateEnclosedArea(dvectors);
}

// Returns the length of the OSMWay that has the given OSMID, in meters.
// To implement this function you will have to access the OSMDatabaseAPI.h
// functions.
// Speed Requirement --> high
double findWayLength(OSMID way_id) {
    auto it = way_length_cache.find(way_id);
    if (it != way_length_cache.end()) {
        return it->second;
    }
    return -1; // Way not found
}
std::string getOSMNodeTagValue(OSMID osm_id, std::string key) {
    auto it = osm_node_tag_cache.find(osm_id);
    if (it != osm_node_tag_cache.end()) {
        auto tag_it = it->second.find(key);
        if (tag_it != it->second.end()) {
            return tag_it->second;
        }
    }
    return ""; // Tag not found
}