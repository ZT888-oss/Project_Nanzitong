
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

// Include necessary headers
#include "m2.h"

#include <m3.h>
#include <m4.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.h"
using namespace std;

using namespace std;

// Struct representing a clickable object on the map
struct ClickableObject {
  enum Type {
    STREET_SEGMENT,
    INTERSECTION,
    POI,
    FEATURE
  };  // Types of clickable objects
  int id;
  Type type;
  std::vector<std::pair<double, double>>
      geometry;                      // Geometry for lines or polygons
  std::pair<double, double> center;  // Center point for points
};
// Global data structures to store preprocessed data
struct FeatureData {
  FeatureType type;
  std::vector<ezgl::point2d> points;  // Precomputed coordinates in meters
  double area;                        // Area for lakes
};

std::vector<FeatureData> features_data;

// External declarations
extern vector<StreetSegmentInfo> street_segment_info;
extern vector<LatLon> intersection_positions;
extern vector<pair<string, StreetIdx>> sorted_street_names;
extern unordered_map<OSMID, string> osm_id_to_classification;
extern vector<vector<OSMID>> bike_lane_ways;
extern vector<double> street_segment_travel_time;
extern vector<vector<StreetSegmentIdx>> intersection_street_segments;
extern vector<unordered_set<IntersectionIdx>> street_intersections;
extern int max_speed;
std::unordered_map<OSMID, int> osm_id_to_node_id;

// Global variables for map features
vector<double> top_right;
vector<vector<double>> POI_positions;  // Positions of Points of Interest (POIs)
vector<IntersectionIdx> highlighted_intersections;  // Highlighted intersections
bool highlight = false;
int intersection_index = -1;

// Global variables for the label
double lat_avg = 0;  // Average latitude for coordinate conversion
std::string currentLabel;
std::pair<double, double> labelPosition;  // Position of the label in meters
bool showLabel = false;
// Track the last clicked object
int lastClickedID =
    -1;  // ID of the last clicked object (-1 indicates no object)
int lastClickedType = 0;  // Type of the last clicked object (default is 0)
bool night_mode = false;
bool find_route = false;
bool showIntersectionLabels = true;
vector<int> find_route_intersections;

/**
 * @file
 *
 *
 */

LatLon maximum;
LatLon mininimum;
extern vector<pair<string, StreetIdx>> sorted_street_names;

// Function declarations
void draw_main_canvas(ezgl::renderer* g);  // Main drawing function
void load_button_click_help(GtkWidget* widget, ezgl::application* application);
void initial_setup(ezgl::application* application,
                   bool new_window);  // Initial setup for the application
void draw_map_contents(ezgl::renderer* g);
void load_button_click(
    GtkWidget* widget,
    ezgl::application* application);  // Callback for loading a map
void search_bar_click(const gchar* text, ezgl::application* application);
void test_button_cbk(
    GtkWidget* widget,
    ezgl::application* application);  // Callback for test button
void setCoordinateSystem();           // Sets the coordinate system for the map
void act_on_mouse_press(ezgl::application* application, GdkEventButton* event,
                        double x, double y);  // Handles mouse press events
void act_on_mouse_move(ezgl::application* application, GdkEventButton* event,
                       double x, double y);  // Handles mouse move events
void act_on_key_press(ezgl::application* application, GdkEventKey* event,
                      char* key_name);
std::string toLowerCase(const std::string& input);
int getPriority(ClickableObject::Type type);
std::pair<int, ClickableObject::Type> clicked(double x, double y);
void preprocessFeatures();
void find_route_button_click(GtkWidget* /*widget*/,
                             ezgl::application* application);
double heuristic(IntersectionIdx nodeID, IntersectionIdx destID,
                 double maxSpeedMPS);
vector<double> getCoordinateMeters(LatLon end);
double pointToLineDistance(double px, double py, double x1, double y1,
                           double x2, double y2);
std::string getClassificationForOSMID(OSMID id);
void drawArrowOnLine(ezgl::renderer* g, const std::vector<double>& from,
                     const std::vector<double>& to, double spacing);
std::vector<StreetIdx> getUniqueElements(const std::vector<StreetIdx>& input);
void draw_nice_label(ezgl::renderer* g, const std::string& text, double x,
                     double y);
bool labelsOverlap(double x1, double y1, double width1, double height1,
                   double x2, double y2, double width2, double height2);
double euclideanDistance(LatLon pos1, LatLon pos2);
gboolean toggle_night_mode(GtkSwitch* /*darkModeSwitch*/, gboolean switch_state,
                           ezgl::application* application);
void textEntry_enter(GtkEntry* textEntryWidget, ezgl::application* application);
void draw_one_way_arrows(ezgl::renderer* g,
                         const StreetSegmentInfo& segmentInfo, int id);
void draw_filled_feature(ezgl::renderer* g,
                         const std::vector<ezgl::point2d>& polygon,
                         ezgl::color fillColor);
void draw_styled_label(ezgl::renderer* g, const std::string& text, double x,
                       double y);
// M3 function: Computes the travel time for an entire path from Point A to B
// given it's segments and turn penalties for each turn in the streets

double computePathTravelTime(const double turn_penalty,
                             const std::vector<StreetSegmentIdx>& path) {
  if (path.empty()) {  // No streets
    return 0;
  }

  double time = 0;

  // Get initial street information
  StreetSegmentInfo street = getStreetSegmentInfo(path[0]);
  StreetIdx segment_p = street.streetID;

  // Add the first segmentCompute's travel time
  time += findStreetSegmentTravelTime(path[0]);

  // Loop through the rest of the path
  for (size_t i = 1; i < path.size(); i++) {
    StreetSegmentInfo street_belongs = getStreetSegmentInfo(path[i]);
    StreetIdx segment_c = street_belongs.streetID;

    // If street changes, add turn penalty
    if (segment_c != segment_p) {
      time += turn_penalty;
    }

    // Add travel time for current segment
    time += findStreetSegmentTravelTime(path[i]);

    // Update previous street ID
    segment_p = segment_c;
  }

  return time;
}

struct WaveElem {
  IntersectionIdx nodeID;
  StreetSegmentIdx edgeID;
  double travelTime;
  double estimatedTotalTime;

  WaveElem(IntersectionIdx n, StreetSegmentIdx e, double time, double estTime)
      : nodeID(n), edgeID(e), travelTime(time), estimatedTotalTime(estTime) {}
};

struct Node {
  vector<StreetSegmentIdx> outgoingEdges;
  StreetSegmentIdx reachingEdge;
  double bestTime;
  bool visited;

  Node()
      : reachingEdge(-1),
        bestTime(numeric_limits<double>::max()),
        visited(false) {}
};

// Custom comparator for the min-heap based on estimatedTotalTime
struct CompareWaveElem {
  bool operator()(const WaveElem& a, const WaveElem& b) {
    return a.estimatedTotalTime > b.estimatedTotalTime;
  }
};

// Function to calculate the Euclidean distance in meters between two points
double euclideanDistance(LatLon pos1, LatLon pos2) {
  double R = kEarthRadiusInMeters;  // Earth radius in meters
  double lat1 = pos1.latitude() * M_PI / 180;
  double lon1 = pos1.longitude() * M_PI / 180;
  double lat2 = pos2.latitude() * M_PI / 180;
  double lon2 = pos2.longitude() * M_PI / 180;

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;
}

// Function to calculate the heuristic estimate from node to destination
double heuristic(IntersectionIdx nodeID, IntersectionIdx destID,
                 double maxSpeedMPS) {
  LatLon nodePos = getIntersectionPosition(nodeID);
  LatLon destPos = getIntersectionPosition(destID);
  double distance = euclideanDistance(nodePos, destPos);
  return distance / maxSpeedMPS;
}

std::vector<StreetSegmentIdx> findPathBetweenIntersections(
    const double turn_penalty,
    const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {
  // extract source and destination from passed vector of intersections
  IntersectionIdx srcID = intersect_ids.first;
  IntersectionIdx destID = intersect_ids.second;

  int numIntersections = getNumIntersections();
  vector<Node> nodes(numIntersections);

  // Populate the outgoing edges for each node using
  // findStreetSegmentsOfIntersection
  for (IntersectionIdx nodeID = 0; nodeID < numIntersections; ++nodeID) {
    vector<StreetSegmentIdx> originalSegments =
        findStreetSegmentsOfIntersection(nodeID);
    for (const auto& segID : originalSegments) {
      StreetSegmentInfo segInfo = street_segment_info[segID];
      // Check if the segment is one-way and if we are at the 'from' node
      if (!segInfo.oneWay || segInfo.from == nodeID) {
        nodes[nodeID].outgoingEdges.push_back(segID);
      }
    }
  }

  // Assign max speed of given city calculated in LoadMap (preprocessed)
  double maxSpeedMPS = max_speed;
  // priority queue that holds next best paths using heuristic/cost function
  priority_queue<WaveElem, vector<WaveElem>, CompareWaveElem> wavefront;

  // Initialize the start node
  nodes[srcID].bestTime = 0.0;
  double startHeuristic = heuristic(srcID, destID, maxSpeedMPS);
  wavefront.push(WaveElem(srcID, -1, 0.0, startHeuristic));

  bool pathFound = false;
  while (!wavefront.empty()) {
    WaveElem wave = wavefront.top();
    wavefront.pop();
    IntersectionIdx currNodeID = wave.nodeID;

    if (wave.travelTime >
        nodes[currNodeID].bestTime)  // if it takes more time, skip
    {
      continue;
    }

    nodes[currNodeID].visited =
        true;  // mark as visited once gone to, avoids infinite loop case
    if (currNodeID == destID)  // reached end of path
    {
      pathFound = true;
      break;
    }

    for (const auto& outEdge : nodes[currNodeID].outgoingEdges) {
      StreetSegmentInfo outEdgeInfo = street_segment_info[outEdge];
      IntersectionIdx toNodeID =
          (outEdgeInfo.from == currNodeID) ? outEdgeInfo.to : outEdgeInfo.from;
      double newTravelTime =
          nodes[currNodeID].bestTime + street_segment_travel_time[outEdge];

      // Apply turn penalty if a turn occurs
      if (nodes[currNodeID].reachingEdge !=
          -1) {  // Check if there was a previous segment
        StreetSegmentInfo prevSegInfo =
            street_segment_info[nodes[currNodeID].reachingEdge];
        if (prevSegInfo.streetID != outEdgeInfo.streetID) {
          newTravelTime += turn_penalty;
        }
      }

      if (newTravelTime <
          nodes[toNodeID]
              .bestTime)  // check if the new travel time of the path from the
                          // segments is better, if it is, update the segment in
                          // the priority queue order
      {
        nodes[toNodeID].bestTime = newTravelTime;
        nodes[toNodeID].reachingEdge = outEdge;
        double newHeuristic = heuristic(toNodeID, destID, maxSpeedMPS);
        double newEstimatedTotalTime = newTravelTime + newHeuristic;
        wavefront.push(
            WaveElem(toNodeID, outEdge, newTravelTime, newEstimatedTotalTime));
      }
    }
  }

  // If no path was found, return an empty vector
  if (!pathFound) {
    return {};
  }

  // Reconstruct the path
  vector<StreetSegmentIdx> path;
  int currNodeID = destID;
  while (currNodeID != srcID) {
    path.push_back(nodes[currNodeID].reachingEdge);
    StreetSegmentInfo segInfo =
        getStreetSegmentInfo(nodes[currNodeID].reachingEdge);
    currNodeID = (segInfo.from == currNodeID) ? segInfo.to : segInfo.from;
  }

  // Reverse the path to get it from src to dest
  reverse(path.begin(), path.end());

  return path;
}

// Helper function to convert LatLon to Cartesian coordinates
vector<double> getCoordinateMeters(LatLon end) {
  vector<double> direction;

  // Convert lat/lon from degrees to radians
  double lat1 = mininimum.latitude() * kDegreeToRadian;
  double lon1 = mininimum.longitude() * kDegreeToRadian;
  double lat2 = end.latitude() * kDegreeToRadian;
  double lon2 = end.longitude() * kDegreeToRadian;

  double lat_avg_radian = lat_avg * kDegreeToRadian;

  // Convert lat/lon to Cartesian coordinates
  double x1 = kEarthRadiusInMeters * lon1 * cos(lat_avg_radian);
  double y1 = kEarthRadiusInMeters * lat1;
  double x2 = kEarthRadiusInMeters * lon2 * cos(lat_avg_radian);
  double y2 = kEarthRadiusInMeters * lat2;

  // Compute displacement
  double dx = x2 - x1;
  double dy = y2 - y1;

  direction.push_back(dx);
  direction.push_back(dy);
  return direction;
}
// Path beginning ---------------------------
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>  // For cerr/cout
#include <limits>
#include <list>  // For faster insertion/deletion in initial solution generation
#include <map>  // Added for ordered map if needed, though unordered_map is generally faster
#include <mutex>
#include <queue>
#include <random>
#include <set>  // For available stops in nearest neighbor
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>  // For std::pair
#include <vector>

#include "StreetsDatabaseAPI.h"  // Assuming this provides necessary declarations
#include "m3.h"  // Assuming this provides findPathBetweenIntersections and DeliveryInf
#include "m4.h"  // Include m4.h for CourierSubPath definition (Uncommented)

using namespace std;

// CourierSubPath struct definition removed - assuming it's defined in m4.h

// --- Tunable Parameters ---
// These parameters control the behavior of the optimization algorithms.
// They might need further tuning based on specific map characteristics and
// problem sizes.

// *** MODIFIED TIME LIMITS ***
const float SOFT_TIME_LIMIT =
    45.0f;  // Target time limit for optimization phases
const float HARD_TIME_LIMIT =
    50.0f;  // Absolute cutoff time limit for the entire function

int NUM_THREADS =
    std::thread::hardware_concurrency();  // Use available hardware threads,
                                          // capped later if needed

// Simulated Annealing Parameters
double INITIAL_TEMPERATURE = 10000.0;
double COOLING_RATE = 0.9995;
int PERTURBATIONS_PER_TEMP =
    100;  // Number of perturbations attempted at each temperature level
const double SA_REHEAT_THRESHOLD_FACTOR =
    0.5;  // Reheat if no improvement for 50% of max streak
const double SA_REHEAT_MULTIPLIER =
    1.5;  // Factor to multiply temperature by on reheat
const double SA_MIN_TEMP_REHEAT_FACTOR =
    0.1;  // Don't reheat if temp is still high (> 10% of initial)

const float MIN_REMAINING_TIME_FACTOR_FOR_3OPT =
    0.20f;  // e.g., Run 3-Opt only if >15% of SOFT_TIME_LIMIT remains (~7s if
            // SOFT=45s)

// Time Allocation Factors (fractions of SOFT_TIME_LIMIT)
// These determine how much time is spent in each optimization phase relative to
// the 45s target.
float PRECOMPUTATION_TIME_FACTOR = 0.20f;  // Time for distance precomputation
float INITIAL_SOLUTION_TIME_FACTOR =
    0.30f;  // Combined time limit factor for generating *all* initial solutions
float HC_TIME_FACTOR = 0.60f;  // Time for Hill Climbing phase (runs within each
                               // multi-start thread) (Adjusted)
float SA_TIME_FACTOR = 0.88f;  // Time for Simulated Annealing phase (runs
                               // within each multi-start thread) (Adjusted)
float THREE_OPT_TIME_FACTOR =
    0.97f;  // Time for 2-Opt refinement phase (runs within each multi-start
            // thread) (Adjusted)
// Note: Factors don't need to sum to 1, they represent deadlines relative to
// the start time based on SOFT_TIME_LIMIT.
float TWO_OPT_TIME_FACTOR = 0.90f;  // *** NEW *** Deadline = SA + 0.15

// Perturbation Probabilities (used in simulatedAnnealing and hillClimbing via
// perturbSolution) These control the mix of different neighborhood moves.
double SWAP_PROBABILITY = 0.25;               // Adjusted
double REVERSE_PROBABILITY = 0.25;            // Adjusted
double MOVE_PROBABILITY = 0.15;               // Adjusted
double RELOCATE_PAIR_PROBABILITY = 0.20;      // Adjusted
double RELOCATE_SEQUENCE_PROBABILITY = 0.15;  // *** NEW ***

// Iteration-based Termination Constants
const int HC_MAX_ITER_NO_IMPROVEMENT = 50;
const int SA_MAX_ITER_NO_IMPROVEMENT = 50;
// Replace TWO_OPT constants with THREE_OPT constants
const int TWO_OPT_MAX_NO_IMPROVEMENT_PASSES =
    7;  // *** NEW *** Max consecutive passes for 2-Opt
const int THREE_OPT_MAX_NO_IMPROVEMENT_PASSES =
    2;  // *** ADJUSTED *** Max *consecutive passes* for 3-Opt (very low)

// --- Data Structures ---

// Represents a potential solution (delivery order) and its calculated travel
// time.
struct Solution {
  vector<int> deliveryOrder;  // Encodes the order: 2*i for pickup i, 2*i+1 for
                              // dropoff i
  double travelTime =
      numeric_limits<double>::max();  // Initialize with a high value

  // Comparison operator for sorting/finding the minimum solution.
  bool operator<(const Solution& other) const {
    return travelTime < other.travelTime;
  }
};

// Thread-safe cache for storing precomputed travel times between intersections.
struct DistanceCache {
  unordered_map<IntersectionIdx, unordered_map<IntersectionIdx, double>> data;
  mutable mutex mtx;  // Mutex to protect concurrent access to the cache data

  // Delete copy constructor and assignment operator to prevent accidental
  // copying.
  DistanceCache(const DistanceCache&) = delete;
  DistanceCache& operator=(const DistanceCache&) = delete;

  // Default constructor is sufficient.
  DistanceCache() = default;

  // Move constructor for efficient transfer of ownership (e.g., when returning
  // from a function).
  DistanceCache(DistanceCache&& other) noexcept {
    lock_guard<mutex> lock(other.mtx);  // Lock the source cache during move
    data = move(other.data);
  }

  // Move assignment operator.
  DistanceCache& operator=(DistanceCache&& other) noexcept {
    if (this != &other) {
      // Lock both mutexes before moving data to prevent deadlocks
      // Use std::lock for deadlock avoidance
      std::lock(mtx, other.mtx);
      // Ensure locks are automatically released
      lock_guard<mutex> lock1(mtx, adopt_lock);
      lock_guard<mutex> lock2(other.mtx, adopt_lock);
      data = move(other.data);
    }
    return *this;
  }

  // Inserts a distance into the cache. Thread-safe.
  void insert(IntersectionIdx from, IntersectionIdx to, double dist) {
    lock_guard<mutex> lock(mtx);  // Lock before modifying data
    data[from][to] = dist;
  }

  // Retrieves a distance from the cache. Returns max() if not found.
  // Thread-safe.
  double get(IntersectionIdx from, IntersectionIdx to) const {
    // Handle trivial case
    if (from == to) return 0.0;

    lock_guard<mutex> lock(mtx);  // Lock before accessing data
    auto from_it = data.find(from);
    if (from_it != data.end()) {
      auto to_it = from_it->second.find(to);
      if (to_it != from_it->second.end()) {
        return to_it->second;  // Return cached distance
      }
    }
    // Optionally print a warning if a needed distance is missing
    // cerr << "Warning: Distance cache miss for " << from << " -> " << to <<
    // endl;
    return numeric_limits<double>::max();  // Indicate distance not found
  }
};
// --- MODIFIED adjustParametersForProblemSize ---
// Adjusts optimization parameters based on the number of deliveries.
void adjustParametersForProblemSize(size_t deliveryCount) {
  // Cap the number of threads to avoid excessive overhead on machines with many
  // cores
  NUM_THREADS =
      min((int)thread::hardware_concurrency(), 8);  // Cap at 8 threads

  if (deliveryCount >= 150) {  // Very Large Problems (>150 deliveries)
    PRECOMPUTATION_TIME_FACTOR = 0.20f;
    INITIAL_SOLUTION_TIME_FACTOR = 0.30f;  // Deadline = 0.20 + 0.10
    HC_TIME_FACTOR = 0.45f;                // Deadline = 0.30 + 0.20
    SA_TIME_FACTOR = 0.80f;                // Deadline = 0.50 + 0.30
    TWO_OPT_TIME_FACTOR = 0.92f;           // Deadline = 0.80 + 0.12
    THREE_OPT_TIME_FACTOR = 0.96f;         // Deadline = 0.92 + 0.05

    INITIAL_TEMPERATURE = 50.0;
    COOLING_RATE = 0.995;
    PERTURBATIONS_PER_TEMP = 500;

    // Give larger moves higher probability
    RELOCATE_PAIR_PROBABILITY = 0.25;
    RELOCATE_SEQUENCE_PROBABILITY = 0.25;
    REVERSE_PROBABILITY = 0.25;  // 2-Opt handles reverse, maybe lower this?
                                 // Keep for SA diversity.
    SWAP_PROBABILITY = 0.15;
    MOVE_PROBABILITY = 0.10;
    NUM_THREADS = min((int)thread::hardware_concurrency(), 4);

  } else if (deliveryCount >= 65) {  // Large Problems (65-149 deliveries)
    PRECOMPUTATION_TIME_FACTOR = 0.20f;
    INITIAL_SOLUTION_TIME_FACTOR = 0.30f;  // Deadline = 0.20 + 0.10
    HC_TIME_FACTOR = 0.45f;                // Deadline = 0.30 + 0.15
    SA_TIME_FACTOR = 0.75f;                // Deadline = 0.45 + 0.30
    TWO_OPT_TIME_FACTOR = 0.90f;           // Deadline = 0.75 + 0.15
    THREE_OPT_TIME_FACTOR = 0.96f;         // Deadline = 0.90 + 0.07

    INITIAL_TEMPERATURE = 30000.0;
    COOLING_RATE = 0.999999;
    PERTURBATIONS_PER_TEMP = 200;

    RELOCATE_PAIR_PROBABILITY = 0.25;
    RELOCATE_SEQUENCE_PROBABILITY = 0.20;
    REVERSE_PROBABILITY = 0.25;
    SWAP_PROBABILITY = 0.15;
    MOVE_PROBABILITY = 0.15;
    NUM_THREADS = min((int)thread::hardware_concurrency(), 4);

  } else if (deliveryCount >= 20) {  // Medium Problems (20-64 deliveries)
    PRECOMPUTATION_TIME_FACTOR = 0.15f;
    INITIAL_SOLUTION_TIME_FACTOR = 0.25f;  // Deadline = 0.15 + 0.10
    HC_TIME_FACTOR = 0.40f;                // Deadline = 0.25 + 0.15
    SA_TIME_FACTOR = 0.70f;                // Deadline = 0.40 + 0.30
    TWO_OPT_TIME_FACTOR = 0.92f;           // Deadline = 0.70 + 0.20
    THREE_OPT_TIME_FACTOR = 0.0;           // Deadline = 0.90 + 0.07

    INITIAL_TEMPERATURE = 15000.0;
    COOLING_RATE = 0.999;
    PERTURBATIONS_PER_TEMP = 150;

    RELOCATE_PAIR_PROBABILITY = 0.20;
    RELOCATE_SEQUENCE_PROBABILITY = 0.15;
    REVERSE_PROBABILITY = 0.25;
    SWAP_PROBABILITY = 0.20;
    MOVE_PROBABILITY = 0.20;
    NUM_THREADS = min((int)thread::hardware_concurrency(), 4);

  } else {  // Small Problems (<20 deliveries)
    PRECOMPUTATION_TIME_FACTOR = 0.10f;
    INITIAL_SOLUTION_TIME_FACTOR = 0.15f;  // Deadline = 0.10 + 0.05
    HC_TIME_FACTOR = 0.45f;                // Deadline = 0.15 + 0.30
    SA_TIME_FACTOR = 0.0f;                 // Disable SA
    TWO_OPT_TIME_FACTOR =
        0.90f;  // Deadline = 0.45 + 0.45 (Give 2-Opt plenty of time)
    THREE_OPT_TIME_FACTOR = 0.0f;  // Disable 3-Opt

    INITIAL_TEMPERATURE = 10000000000.0;
    COOLING_RATE = 0.995;
    PERTURBATIONS_PER_TEMP = 100;

    RELOCATE_PAIR_PROBABILITY = 0.15;
    RELOCATE_SEQUENCE_PROBABILITY = 0.10;
    REVERSE_PROBABILITY = 0.30;
    SWAP_PROBABILITY = 0.30;
    MOVE_PROBABILITY = 0.15;
    NUM_THREADS = min((int)thread::hardware_concurrency(), 2);
  }

  // Ensure perturbation probabilities sum to 1 (or close enough) after
  // adjustments
  double total_prob = SWAP_PROBABILITY + REVERSE_PROBABILITY +
                      MOVE_PROBABILITY + RELOCATE_PAIR_PROBABILITY +
                      RELOCATE_SEQUENCE_PROBABILITY;
  if (abs(total_prob) > 1e-9 && abs(total_prob - 1.0) > 1e-6) {
    SWAP_PROBABILITY /= total_prob;
    REVERSE_PROBABILITY /= total_prob;
    MOVE_PROBABILITY /= total_prob;
    RELOCATE_PAIR_PROBABILITY /= total_prob;
    RELOCATE_SEQUENCE_PROBABILITY /= total_prob;
  } else if (abs(total_prob) < 1e-9) {
    // Fallback if all probabilities are zero
    SWAP_PROBABILITY = 0.20;
    REVERSE_PROBABILITY = 0.20;
    MOVE_PROBABILITY = 0.20;
    RELOCATE_PAIR_PROBABILITY = 0.20;
    RELOCATE_SEQUENCE_PROBABILITY = 0.20;
  }
}

// --- Pathfinding (Dijkstra for Precomputation) ---

// Structure used in the Dijkstra priority queue.
struct DijkstraWaveElem {
  IntersectionIdx nodeID;
  StreetSegmentIdx reachingEdge;  // The edge used to reach this node
  double travelTime;  // The time taken to reach this node from the source

  // Constructor
  DijkstraWaveElem(IntersectionIdx node, StreetSegmentIdx edge, double time)
      : nodeID(node), reachingEdge(edge), travelTime(time) {}

  // Comparison operator for the priority queue (min-heap based on travelTime).
  bool operator>(const DijkstraWaveElem& other) const {
    return travelTime > other.travelTime;
  }
};

// Structure to store node information during Dijkstra's algorithm.
struct DijkstraNodeInfo {
  StreetSegmentIdx reachingEdge =
      -1;  // Invalid index signifies no reaching edge yet
  double bestTime = numeric_limits<double>::max();  // Initialize with infinity
};

// Performs Dijkstra's algorithm from a source intersection to find shortest
// paths to all specified destination intersections. Stores results in the
// DistanceCache.
void parallelDijkstraWorker(
    IntersectionIdx source, const unordered_set<IntersectionIdx>& destinations,
    float turn_penalty,
    DistanceCache& distance_cache) {  // Pass cache by reference

  vector<DijkstraNodeInfo> nodes_info(getNumIntersections());
  // Priority queue stores {travelTime, nodeID, reachingEdge}
  priority_queue<DijkstraWaveElem, vector<DijkstraWaveElem>,
                 greater<DijkstraWaveElem>>
      waveFront;

  nodes_info[source].bestTime = 0.0;
  waveFront.push(
      DijkstraWaveElem(source, -1, 0.0));  // Start at source with 0 time

  size_t destinations_found = 0;
  size_t destinations_to_find = 0;
  // Count how many destinations we actually need to find *from this source*
  for (IntersectionIdx dest : destinations) {
    if (dest != source) {
      destinations_to_find++;
    }
  }
  // Add self-distance immediately
  distance_cache.insert(source, source, 0.0);

  while (!waveFront.empty() && destinations_found < destinations_to_find) {
    DijkstraWaveElem current = waveFront.top();
    waveFront.pop();

    IntersectionIdx currentNodeID = current.nodeID;

    // If we found a shorter path already, skip this element
    // Also skip if the node is already marked as found (-2)
    if (current.travelTime > nodes_info[currentNodeID].bestTime ||
        nodes_info[currentNodeID].reachingEdge == -2) {
      continue;
    }

    // If this node is one of the target destinations (and not the source
    // itself), record the distance. Use reachingEdge == -2 to mark as found to
    // avoid double counting if reached via multiple paths in PQ.
    if (currentNodeID != source && destinations.count(currentNodeID)) {
      // Check if bestTime is valid before inserting
      if (nodes_info[currentNodeID].bestTime != numeric_limits<double>::max()) {
        distance_cache.insert(source, currentNodeID, current.travelTime);
        destinations_found++;
        nodes_info[currentNodeID].reachingEdge =
            -2;  // Mark as found and processed
      } else {
        // If bestTime is still max, it means we popped an invalid path,
        // continue searching This case should be rare if Dijkstra is
        // implemented correctly but handles potential edge cases.
      }
    }

    // Optimization: If the current node is already found, we don't need to
    // explore its neighbors further for finding paths *to other destinations*,
    // as we have the shortest path *to this node*. However, this node might be
    // on the shortest path *to another destination*, so we MUST continue
    // exploring. The check `nodes_info[currentNodeID].reachingEdge == -2` at
    // the top handles skipping already finalized nodes.

    // Explore neighbors
    for (const StreetSegmentIdx& outgoing_segment_id :
         findStreetSegmentsOfIntersection(currentNodeID)) {
      StreetSegmentInfo segInfo = getStreetSegmentInfo(outgoing_segment_id);

      // Determine the neighbor intersection ID
      IntersectionIdx neighborNodeID =
          (segInfo.from == currentNodeID) ? segInfo.to : segInfo.from;

      // Skip if it's a one-way street going the wrong way
      if (segInfo.oneWay && segInfo.to == currentNodeID) {
        continue;
      }

      // Calculate the cost to reach the neighbor
      double segment_travel_time = findStreetSegmentTravelTime(
          outgoing_segment_id);  // Assuming m1 function
      double turn_cost = 0.0;

      // Add turn penalty if turning from a previous street
      if (current.reachingEdge != -1) {  // Check if not the starting node
        StreetSegmentInfo prevSegInfo =
            getStreetSegmentInfo(current.reachingEdge);
        if (prevSegInfo.streetID != segInfo.streetID) {
          turn_cost = turn_penalty;
        }
      }

      double newTime = current.travelTime + segment_travel_time + turn_cost;

      // If this path is shorter than the previously known shortest path to the
      // neighbor
      if (newTime < nodes_info[neighborNodeID].bestTime) {
        nodes_info[neighborNodeID].bestTime = newTime;
        nodes_info[neighborNodeID].reachingEdge =
            outgoing_segment_id;  // Store the edge used
        waveFront.push(
            DijkstraWaveElem(neighborNodeID, outgoing_segment_id, newTime));
      }
    }
  }
  // After Dijkstra finishes, any destination not found remains at max distance
  // in the cache implicitly.
}

// Precomputes travel times between all relevant intersections (depots, pickups,
// dropoffs) using parallel Dijkstra runs.
DistanceCache precomputeDistancesParallel(
    const vector<IntersectionIdx>& depots,
    const vector<DeliveryInf>& deliveries, float turn_penalty,
    const chrono::high_resolution_clock::time_point& start_time) {
  DistanceCache distance_cache;
  unordered_set<IntersectionIdx>
      all_locations;  // Use a set to automatically handle duplicates

  // Add all depot locations
  for (const auto& depot : depots) {
    all_locations.insert(depot);
  }
  // Add all pickup and dropoff locations
  for (const auto& d : deliveries) {
    all_locations.insert(d.pickUp);
    all_locations.insert(d.dropOff);
  }

  vector<IntersectionIdx> sources(all_locations.begin(), all_locations.end());
  vector<thread> threads;
  atomic<size_t> next_source_idx(
      0);  // Atomic index to safely get the next source for threads
  atomic<bool> timed_out(false);  // Flag to signal timeout to worker threads

  // Worker function for each thread
  auto worker_task = [&]() {
    while (true) {
      // Check for timeout signal
      if (timed_out.load()) {
        break;
      }
      // Check time limit directly as well - Use SOFT limit for phases
      if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                   start_time)
              .count() > SOFT_TIME_LIMIT * PRECOMPUTATION_TIME_FACTOR) {
        timed_out.store(true);  // Signal other threads
        break;                  // Stop precomputation if time limit exceeded
      }

      size_t current_idx =
          next_source_idx.fetch_add(1);  // Atomically get and increment index
      if (current_idx >= sources.size()) {
        break;  // No more sources left
      }
      // Run Dijkstra for the assigned source intersection
      parallelDijkstraWorker(sources[current_idx], all_locations, turn_penalty,
                             distance_cache);
    }
  };

  // Launch worker threads
  int num_actual_threads =
      min((int)sources.size(),
          NUM_THREADS);  // Don't launch more threads than sources
  for (int i = 0; i < num_actual_threads; ++i) {
    threads.emplace_back(worker_task);
  }

  // Wait for all threads to complete
  for (auto& t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }

  return distance_cache;  // Return the populated cache
}

// --- Solution Evaluation and Legality ---

// Calculates the total travel time for a given solution (delivery order).
// Returns numeric_limits<double>::max() if the solution is invalid (e.g.,
// dropoff before pickup) or if any required path segment distance is missing
// from the cache.
double evaluateSolution(
    const vector<int>& deliveryOrder,  // Pass only the order vector
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot,
    const DistanceCache& distance_cache) {  // Pass cache by const reference

  unordered_set<int> pickedUpItems;  // Tracks which delivery items (by index)
                                     // have been picked up
  IntersectionIdx currentLocation = startDepot;
  double total_time = 0.0;

  // Iterate through the delivery order (pickups and dropoffs)
  for (int stop_code : deliveryOrder) {
    int deliveryIndex =
        stop_code / 2;  // Index of the delivery in the 'deliveries' vector
    bool isPickup = (stop_code % 2 == 0);
    IntersectionIdx nextLocation;

    if (isPickup) {
      nextLocation = deliveries[deliveryIndex].pickUp;
    } else {
      // Check legality: Ensure the item was picked up before dropoff
      if (pickedUpItems.find(deliveryIndex) == pickedUpItems.end()) {
        // cerr << "Debug: evaluateSolution found illegal dropoff for item " <<
        // deliveryIndex << endl;
        return numeric_limits<double>::max();  // Invalid order: dropoff before
                                               // pickup
      }
      nextLocation = deliveries[deliveryIndex].dropOff;
    }

    // Get travel time from cache
    double leg_time = distance_cache.get(currentLocation, nextLocation);
    if (leg_time == numeric_limits<double>::max()) {
      // If a path segment is unknown/infinite, the entire solution is
      // invalid/infinite time. cerr << "Debug: evaluateSolution cache miss or
      // infinite distance from " << currentLocation << " to " << nextLocation
      // << endl;
      return numeric_limits<double>::max();
    }
    total_time += leg_time;

    // Update state
    currentLocation = nextLocation;
    if (isPickup) {
      pickedUpItems.insert(deliveryIndex);  // Mark item as picked up
    }
  }

  // Add travel time from the last delivery stop to the end depot
  double final_leg_time = distance_cache.get(currentLocation, endDepot);
  if (final_leg_time == numeric_limits<double>::max()) {
    // cerr << "Debug: evaluateSolution cache miss or infinite distance from
    // last stop " << currentLocation << " to end depot " << endDepot << endl;
    return numeric_limits<double>::max();  // Cannot reach end depot
  }
  total_time += final_leg_time;

  return total_time;
}

// Checks if a given delivery order is valid (all dropoffs occur after their
// corresponding pickups).
bool isSolutionLegal(const vector<int>& deliveryOrder) {
  unordered_set<int> pickedUpItems;
  for (int stop_code : deliveryOrder) {
    int deliveryIndex = stop_code / 2;
    bool isPickup = (stop_code % 2 == 0);

    if (isPickup) {
      pickedUpItems.insert(deliveryIndex);
    } else {
      // If it's a dropoff, check if the corresponding pickup has occurred.
      if (pickedUpItems.find(deliveryIndex) == pickedUpItems.end()) {
        return false;  // Illegal: Dropoff before pickup
      }
    }
  }
  return true;  // All checks passed, the order is legal.
}

// --- Initial Solution Generation ---

// Helper function to get the IntersectionIdx for a stop code
IntersectionIdx getStopLocation(int stop_code,
                                const vector<DeliveryInf>& deliveries) {
  int deliveryIndex = stop_code / 2;
  bool isPickup = (stop_code % 2 == 0);
  // Basic bounds check for safety
  if (deliveryIndex < 0 || deliveryIndex >= deliveries.size()) {
    cerr << "Error: Invalid delivery index " << deliveryIndex
         << " from stop code " << stop_code << endl;
    return -1;  // Invalid intersection ID
  }
  return isPickup ? deliveries[deliveryIndex].pickUp
                  : deliveries[deliveryIndex].dropOff;
}

// Generates an initial feasible solution using a fast Nearest Neighbor
// heuristic.
Solution generateInitialSolution_NearestNeighbor(
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot,  // Needed for final evaluation
    const DistanceCache& distance_cache,
    const chrono::high_resolution_clock::time_point& start_time) {
  Solution solution;
  int numDeliveries = deliveries.size();
  solution.deliveryOrder.reserve(2 * numDeliveries);

  set<int> available_stops;            // Stores stop codes (2i or 2i+1)
  unordered_set<int> picked_up_items;  // Stores delivery indices (i)

  // Initially, only pickups are available
  for (int i = 0; i < numDeliveries; ++i) {
    available_stops.insert(2 * i);  // Add pickup code
  }

  IntersectionIdx currentLocation = startDepot;
  double currentTotalTime = 0.0;  // Keep track of time for NN heuristic
                                  // (optional, but can inform choices)

  // Use SOFT limit for phase deadline
  double initial_solution_deadline =
      SOFT_TIME_LIMIT *
      (PRECOMPUTATION_TIME_FACTOR + INITIAL_SOLUTION_TIME_FACTOR);

  while (solution.deliveryOrder.size() < 2 * numDeliveries) {
    // Check for overall HARD timeout - important to prevent exceeding total
    // limit
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 start_time)
            .count() >
        HARD_TIME_LIMIT * 0.98) {  // Check against overall limit (e.g., 98%)
      cerr << "Warning: HARD time limit approaching during initial solution "
              "generation (NN)."
           << endl;
      solution.travelTime =
          numeric_limits<double>::max();  // Mark as invalid due to timeout
      return solution;
    }

    // --- Time check removed as requested by user previously ---
    // --- It's generally safer to keep a check against the SOFT limit here ---
    // if (chrono::duration<double>(chrono::high_resolution_clock::now() -
    // start_time).count() > initial_solution_deadline) {
    //      cerr << "Warning: Initial solution generation (NN) timed out based
    //      on SOFT limit." << endl; solution.travelTime =
    //      numeric_limits<double>::max(); // Mark as invalid due to timeout
    //      return solution;
    // }
    // --- End of removed time check ---

    int best_next_stop = -1;
    double min_time_to_next = numeric_limits<double>::max();

    // Find the closest valid stop among available stops
    for (int stop_code : available_stops) {
      IntersectionIdx stopLocation = getStopLocation(stop_code, deliveries);
      if (stopLocation == -1) continue;  // Skip if location is invalid

      double time_to_stop = distance_cache.get(currentLocation, stopLocation);

      if (time_to_stop < min_time_to_next) {
        min_time_to_next = time_to_stop;
        best_next_stop = stop_code;
      }
    }

    // Check if a reachable stop was found
    if (best_next_stop == -1 ||
        min_time_to_next == numeric_limits<double>::max()) {
      cerr << "Error: Could not find reachable stop in Nearest Neighbor. "
              "Problem might be unsolvable or cache incomplete."
           << endl;
      solution.travelTime = numeric_limits<double>::max();  // Mark as invalid
      return solution;
    }

    // Add the best stop to the order
    solution.deliveryOrder.push_back(best_next_stop);
    currentTotalTime += min_time_to_next;  // Update time estimate (optional)
    currentLocation =
        getStopLocation(best_next_stop, deliveries);  // Update current location
    available_stops.erase(best_next_stop);            // Remove from available

    // If it was a pickup, mark item as picked up and make dropoff available
    if (best_next_stop % 2 == 0) {
      int deliveryIndex = best_next_stop / 2;
      picked_up_items.insert(deliveryIndex);
      available_stops.insert(2 * deliveryIndex + 1);  // Add dropoff code
    }
  }

  // Calculate the final accurate travel time using evaluateSolution
  solution.travelTime = evaluateSolution(solution.deliveryOrder, deliveries,
                                         startDepot, endDepot, distance_cache);

  // Final legality check (should pass if logic is correct)
  if (!isSolutionLegal(solution.deliveryOrder)) {
    cerr << "Error: Nearest Neighbor generated an illegal solution!" << endl;
    solution.travelTime = numeric_limits<double>::max();
  }
  if (solution.travelTime == numeric_limits<double>::max()) {
    cerr << "Warning: Nearest Neighbor solution evaluation resulted in max "
            "time (likely cache miss or disconnect)."
         << endl;
  }

  return solution;
}

// --- Neighborhood Search Operators (Perturbations) ---

// Generates a neighboring solution by applying a random perturbation
// (swap, reverse segment, move single stop, relocate pair, relocate sequence).
// Ensures the resulting neighbor is legal (pickup before dropoff).
Solution perturbSolution(
    const Solution& current,
    const vector<DeliveryInf>&
        deliveries,       // Still potentially needed for legality checks if
                          // isSolutionLegal uses it
    std::mt19937& rng) {  // Pass random number generator by reference

  Solution neighbor = current;  // Start with a copy
  size_t order_size = neighbor.deliveryOrder.size();

  // Need at least 2 stops for any of these perturbations
  if (order_size < 2) {
    return neighbor;
  }

  // Define distribution based on probabilities for lecture-based operators
  // 0: Swap, 1: Reverse, 2: Move
  std::discrete_distribution<int> op_dist({
      SWAP_PROBABILITY,
      REVERSE_PROBABILITY,
      MOVE_PROBABILITY,
      // Removed Relocate Pair and Relocate Sequence probabilities
  });

  // Distribution for selecting indices within the order
  std::uniform_int_distribution<size_t> index_dist(0, order_size - 1);

  // Try multiple times (e.g., 5) to find a legal perturbation
  // This handles cases where a randomly chosen move is illegal
  const int MAX_ATTEMPTS = 5;
  for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
    Solution temp_neighbor = current;  // Work on a temporary copy each attempt
    int operation =
        op_dist(rng);  // Choose operation: 0=Swap, 1=Reverse, 2=Move

    // Generate random indices i and j
    size_t i = index_dist(rng);
    size_t j = index_dist(rng);

    // Ensure i <= j for operations that need ordered indices (Reverse)
    // For Swap and Move, the order of i and j might not strictly matter,
    // but ensuring i <= j simplifies the Move logic slightly.
    if (i > j) std::swap(i, j);

    // --- Apply the chosen operation ---
    bool perturbation_applied =
        false;  // Flag to check if a valid operation was performed

    if (operation == 0) {    // Swap (Lecture Page 11)
      if (i == j) continue;  // Don't swap with self, try another attempt
      std::swap(temp_neighbor.deliveryOrder[i], temp_neighbor.deliveryOrder[j]);
      perturbation_applied = true;

    } else if (operation == 1) {  // Reverse (2-Opt) (Lecture Page 12-13)
      // Need at least two elements to reverse a segment (j > i)
      if (i == j)
        continue;  // Cannot reverse single element, try another attempt
      std::reverse(temp_neighbor.deliveryOrder.begin() + i,
                   temp_neighbor.deliveryOrder.begin() + j + 1);
      perturbation_applied = true;

    } else if (operation == 2) {  // Move single stop (Lecture Page 8-10)
      // We need to move element at index `i` to be inserted *before* index `j`.
      // Re-select `j` to be the insertion point (0 to order_size).
      // Ensure i and j are chosen such that the move is meaningful.

      // Choose original position `i_move` (0 to size-1)
      std::uniform_int_distribution<size_t> move_idx_dist(0, order_size - 1);
      size_t i_move = move_idx_dist(rng);

      // Choose insertion position `j_insert` (0 to size)
      // This is the index the element will have *after* insertion.
      std::uniform_int_distribution<size_t> insert_idx_dist(0, order_size);
      size_t j_insert = insert_idx_dist(rng);

      // If inserting at the original position or just after, it's a no-op. Try
      // again.
      if (j_insert == i_move || j_insert == i_move + 1) {
        continue;  // Try another attempt
      }

      int stop_to_move = temp_neighbor.deliveryOrder[i_move];

      // Erase from original position
      temp_neighbor.deliveryOrder.erase(temp_neighbor.deliveryOrder.begin() +
                                        i_move);

      // Calculate actual insertion index after erase
      size_t actual_insert_idx = j_insert;
      if (j_insert > i_move) {
        actual_insert_idx--;  // Adjust index due to erase
      }
      // Ensure index is within bounds [0, current_size]
      actual_insert_idx =
          std::min(actual_insert_idx, temp_neighbor.deliveryOrder.size());

      // Insert at the new position
      temp_neighbor.deliveryOrder.insert(
          temp_neighbor.deliveryOrder.begin() + actual_insert_idx,
          stop_to_move);
      perturbation_applied = true;
    }

    // --- Check Legality ---
    // Only return if a valid perturbation was actually applied and it's legal
    if (perturbation_applied && isSolutionLegal(temp_neighbor.deliveryOrder)) {
      // Reset travel time as the order changed. Will be re-evaluated by caller
      // (e.g., SA).
      temp_neighbor.travelTime = std::numeric_limits<double>::max();
      return temp_neighbor;  // Return the first legal neighbor found
    }
    // If illegal or no-op, loop and try another perturbation attempt
  }

  // Return original solution if no legal neighbor found after MAX_ATTEMPTS
  // Reset time just in case, although it should be the same as
  // current.travelTime
  Solution original_copy = current;
  original_copy.travelTime =
      std::numeric_limits<double>::max();  // Signal evaluation needed if
                                           // returned
  return original_copy;
}

// --- Optimization Algorithms ---

IntersectionIdx getLocationFromOrderIndex(int orderIndex,
                                          const vector<int>& deliveryOrder,
                                          const vector<DeliveryInf>& deliveries,
                                          IntersectionIdx startDepot,
                                          IntersectionIdx endDepot) {
  if (orderIndex < 0) {  // Start depot
    return startDepot;
  } else if (static_cast<size_t>(orderIndex) >=
             deliveryOrder.size()) {  // End depot
    return endDepot;
  } else {  // A delivery stop
    return getStopLocation(deliveryOrder[orderIndex], deliveries);
  }
}

// Hill Climbing algorithm: Iteratively moves to the best neighboring solution.
// Stops when no better neighbor is found for a number of iterations or time
// runs out.
Solution hillClimbing(
    Solution initialSolution, float turn_penalty,
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot, const DistanceCache& distance_cache,
    const chrono::high_resolution_clock::time_point& startTime) {
  Solution current = initialSolution;
  // Ensure initial travel time is calculated if not already done
  if (current.travelTime == numeric_limits<double>::max()) {
    current.travelTime = evaluateSolution(current.deliveryOrder, deliveries,
                                          startDepot, endDepot, distance_cache);
    if (current.travelTime == numeric_limits<double>::max()) {
      // If still invalid after evaluation, cannot proceed with HC
      return current;
    }
  }

  Solution best_overall = current;  // Track the best solution found so far
  int no_improvement_streak = 0;  // Counter for iterations without improvement

  // Use thread-local random number generator for better parallel performance
  // Seed with a combination of random_device and thread ID for uniqueness
  random_device rd;
  mt19937 rng(rd() ^ (uint64_t)hash<thread::id>{}(this_thread::get_id()));

  while (
      chrono::duration<double>(chrono::high_resolution_clock::now() - startTime)
              .count() < SOFT_TIME_LIMIT * HC_TIME_FACTOR  // Use SOFT limit
      && no_improvement_streak <
             HC_MAX_ITER_NO_IMPROVEMENT) {  // Added iteration check

    double time_before_perturb = best_overall.travelTime;

    // Generate a neighbor
    Solution neighbor = perturbSolution(current, deliveries, rng);

    // Evaluate the neighbor
    neighbor.travelTime =
        evaluateSolution(neighbor.deliveryOrder, deliveries, startDepot,
                         endDepot, distance_cache);

    // If the neighbor is better and valid, move to it
    if (neighbor.travelTime != numeric_limits<double>::max() &&
        neighbor.travelTime < current.travelTime) {
      current = neighbor;
      // Update the overall best if this is the best found yet
      if (current.travelTime < best_overall.travelTime) {
        best_overall = current;
        // Improvement found, reset streak
        // no_improvement_streak = 0; // Resetting here makes it consecutive
      }
    }
    // If neighbor is not better or invalid, stay at current solution for the
    // next iteration.

    // Check if best_overall improved in this iteration
    if (best_overall.travelTime >= time_before_perturb) {
      no_improvement_streak++;  // Increment if no improvement in best_overall
    } else {
      no_improvement_streak = 0;  // Reset if best_overall improved
    }

  }  // End while loop
  return best_overall;  // Return the best solution found during the HC phase
}

// --- Constants (ensure these are defined appropriately and tuned) ---
// double INITIAL_TEMPERATURE = 10000.0; // Lecture Q: How high? Needs tuning.
// double COOLING_RATE = 0.9995; // Lecture Q: How quickly to reduce T?
// Geometric cooling rate. int PERTURBATIONS_PER_TEMP = 100; // Lecture: "many
// perturbations" - Number of attempts at each temp level. const double
// SA_REHEAT_THRESHOLD_FACTOR = 0.5; // Factor of max streak before considering
// reheat const double SA_REHEAT_MULTIPLIER = 1.5;       // Temperature
// multiplier on reheat const double SA_MIN_TEMP_REHEAT_FACTOR = 0.1;  // Don't
// reheat if temp hasn't dropped much (e.g., >10% of initial) const int
// SA_MAX_ITER_NO_IMPROVEMENT = 100; // Max iterations without improving BEST
// solution const float SOFT_TIME_LIMIT = 45.0f; // Target time limit const
// float SA_TIME_FACTOR = 0.75f;  // Deadline factor for SA phase

// Simulated Annealing algorithm implementation based on lecture slides
Solution simulatedAnnealing(
    Solution initialSolution,
    float turn_penalty,  // Keep parameters consistent
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot, const DistanceCache& distance_cache,
    const chrono::high_resolution_clock::time_point& startTime) {
  // --- Initialization (Matches Lecture) ---
  // S = InitialSolution;
  Solution current = initialSolution;

  // C = Cost(s);
  // Ensure initial travel time is calculated
  if (current.travelTime == std::numeric_limits<double>::max()) {
    current.travelTime = evaluateSolution(current.deliveryOrder, deliveries,
                                          startDepot, endDepot, distance_cache);
    // If initial solution is invalid, return immediately
    if (current.travelTime == std::numeric_limits<double>::max()) {
      std::cerr << "Error: Initial solution for SA is invalid." << std::endl;
      return current;
    }
  }

  // Track the best solution found during the entire SA process
  Solution best_overall = current;
  int no_improvement_streak =
      0;  // Counter for steps without improvement in best_overall

  // T = high temperature;
  double temperature = INITIAL_TEMPERATURE;
  double initial_temp_for_reheat_check =
      INITIAL_TEMPERATURE;  // Store initial T for reheat check

  // Random number generation for acceptance probability
  std::random_device rd;
  std::mt19937 rng(rd() ^ (uint64_t)std::hash<std::thread::id>{}(
                              std::this_thread::get_id()));
  std::uniform_real_distribution<double> accept_prob_dist(0.0, 1.0);

  // Calculate deadline time point for this phase
  auto sa_deadline = startTime + std::chrono::duration<double>(SOFT_TIME_LIMIT *
                                                               SA_TIME_FACTOR);

  // --- Outer Loop (Matches Lecture: `while (solution changing)`) ---
  // Implemented with practical termination conditions: time, temperature, lack
  // of improvement
  while (std::chrono::high_resolution_clock::now() < sa_deadline &&
         temperature > 1e-3  // Stop if temperature gets very low
  ) {
    bool best_improved_this_temp_step =
        false;  // Track if best_overall improved in this temp step

    // --- Inner Loop (Matches Lecture: `for (many perturbations)`) ---
    for (int iter = 0; iter < PERTURBATIONS_PER_TEMP; ++iter) {
      // Check time limit frequently within the inner loop
      if (std::chrono::high_resolution_clock::now() >= sa_deadline) {
        break;  // Exit inner loop if deadline reached
      }

      // --- Perturbation (Matches Lecture: `S_new = perturb(s);`) ---
      // Using the original multi-operator perturbation function for exploration
      Solution neighbor = perturbSolution(current, deliveries, rng);

      // --- Cost Calculation (Matches Lecture: `C_new = Cost(S_new);`) ---
      // Evaluate the neighbor only if perturbation actually changed the order
      // or if the current solution is somehow invalid (shouldn't happen here).
      // perturbSolution tries to return legal neighbors.
      if (neighbor.deliveryOrder != current.deliveryOrder ||
          current.travelTime == std::numeric_limits<double>::max()) {
        neighbor.travelTime =
            evaluateSolution(neighbor.deliveryOrder, deliveries, startDepot,
                             endDepot, distance_cache);
      } else {
        neighbor.travelTime =
            current.travelTime;  // No change if perturbation failed
      }

      // Skip if evaluation failed (e.g., cache miss, disconnected graph)
      if (neighbor.travelTime == std::numeric_limits<double>::max()) {
        continue;
      }

      // --- Delta Calculation (Matches Lecture: `deltaC = C_new - C;`) ---
      double delta_time = neighbor.travelTime - current.travelTime;
      // --- Acceptance Criterion (Matches Lecture: `if (deltaC < 0 ||
      // random(0,1) < exp(-deltaC / T))`) ---
      if (delta_time < 0 ||
          accept_prob_dist(rng) < std::exp(-delta_time / temperature)) {
        // --- Update Current Solution (Matches Lecture: `S = S_new; C =
        // C_new;`) ---
        current = neighbor;  // Move to the accepted neighbor state

        // Update the overall best solution if the *accepted* current one is
        // better
        if (current.travelTime < best_overall.travelTime) {
          cout << "here" << endl;
          best_overall = current;
          best_improved_this_temp_step =
              true;  // Mark improvement in best_overall
                     // Reset streak only when the *best overall* is improved
                     // no_improvement_streak = 0; // Moved logic below
        }
      }
    }  // --- End Inner Loop ---

    // Check time limit again before cooling
    if (std::chrono::high_resolution_clock::now() >= sa_deadline) {
      break;  // Exit outer loop if deadline reached
    }

    // Update improvement streak & check for reheat based on *best_overall*
    // improvement
    if (best_improved_this_temp_step) {
      no_improvement_streak = 0;  // Reset streak if best improved
    } else {
      no_improvement_streak++;  // Increment streak if no improvement in
                                // best_overall this step
      // Check for reheating condition (practical enhancement)
      if (no_improvement_streak >
              SA_MAX_ITER_NO_IMPROVEMENT * SA_REHEAT_THRESHOLD_FACTOR &&
          temperature > 1e-3 &&  // Avoid reheating if already very cold
          temperature <
              initial_temp_for_reheat_check *
                  (1.0 - SA_MIN_TEMP_REHEAT_FACTOR)) {  // Check if temp dropped
                                                        // enough
        temperature *= SA_REHEAT_MULTIPLIER;            // Reheat
        no_improvement_streak = 0;  // Reset streak after reheat
        // std::cout << "Info: SA Reheating to T=" << temperature << std::endl;
        // // Optional debug
      }
    }

    // --- Cooling (Matches Lecture: `T = reduceTemp(T);`) ---
    temperature *= COOLING_RATE;  // Geometric cooling

  }  // --- End Outer Loop ---

  // Return the best solution found during the entire SA process
  return best_overall;
}

// --- Parallel Multi-Start Execution ---

// Finds the best depot to start from, considering average distance to all
// pickup locations. Uses parallel processing to speed up the calculation.
IntersectionIdx findBestStartDepotParallel(
    const vector<IntersectionIdx>& depots,
    const vector<DeliveryInf>& deliveries,
    const DistanceCache& distance_cache) {
  if (depots.empty()) return -1;             // Handle empty depot list
  if (depots.size() == 1) return depots[0];  // Only one choice

  vector<pair<double, IntersectionIdx>> depotScores(
      depots.size());  // Store {score, depot_id}
  vector<thread> threads;
  atomic<size_t> next_depot_idx(0);

  auto worker = [&]() {
    while (true) {
      size_t i = next_depot_idx.fetch_add(1);
      if (i >= depots.size()) break;

      IntersectionIdx currentDepot = depots[i];
      double totalDist = 0;
      int reachable_pickups = 0;
      bool possible = true;

      for (const auto& delivery : deliveries) {
        double dist = distance_cache.get(currentDepot, delivery.pickUp);
        if (dist != numeric_limits<double>::max()) {
          totalDist += dist;
          reachable_pickups++;
        } else {
          // If any pickup is unreachable, this depot is invalid
          possible = false;
          break;
        }
      }
      // Score is average distance if possible, otherwise max
      depotScores[i] = {(!possible || reachable_pickups == 0)
                            ? numeric_limits<double>::max()
                            : (totalDist / reachable_pickups),
                        currentDepot};
    }
  };

  int num_actual_threads = min((int)depots.size(), NUM_THREADS);
  for (int i = 0; i < num_actual_threads; ++i) {
    threads.emplace_back(worker);
  }
  for (auto& t : threads) {
    if (t.joinable()) t.join();
  }

  // Find the depot with the minimum score (average distance)
  auto best_depot_it = min_element(depotScores.begin(), depotScores.end());

  // Check if any depot was valid (score is not max)
  if (best_depot_it == depotScores.end() ||
      best_depot_it->first == numeric_limits<double>::max()) {
    cerr << "Warning: No start depot can reach all pickup locations." << endl;
    return depots[0];  // Fallback to the first depot
  }

  return best_depot_it->second;  // Return the ID of the best depot
}

// Finds the best depot to end at, considering average distance from all dropoff
// locations. Uses parallel processing.
IntersectionIdx findBestEndDepotParallel(const vector<IntersectionIdx>& depots,
                                         const vector<DeliveryInf>& deliveries,
                                         const DistanceCache& distance_cache) {
  if (depots.empty()) return -1;
  if (depots.size() == 1) return depots[0];

  vector<pair<double, IntersectionIdx>> depotScores(
      depots.size());  // {score, depot_id}
  vector<thread> threads;
  atomic<size_t> next_depot_idx(0);

  auto worker = [&]() {
    while (true) {
      size_t i = next_depot_idx.fetch_add(1);
      if (i >= depots.size()) break;

      IntersectionIdx currentDepot = depots[i];
      double totalDist = 0;
      int reachable_dropoffs = 0;
      bool possible = true;

      for (const auto& delivery : deliveries) {
        double dist = distance_cache.get(delivery.dropOff, currentDepot);
        if (dist != numeric_limits<double>::max()) {
          totalDist += dist;
          reachable_dropoffs++;
        } else {
          // If any dropoff cannot reach this depot, it's invalid
          possible = false;
          break;
        }
      }
      // Score is average distance if possible, otherwise max
      depotScores[i] = {(!possible || reachable_dropoffs == 0)
                            ? numeric_limits<double>::max()
                            : (totalDist / reachable_dropoffs),
                        currentDepot};
    }
  };

  int num_actual_threads = min((int)depots.size(), NUM_THREADS);
  for (int i = 0; i < num_actual_threads; ++i) {
    threads.emplace_back(worker);
  }
  for (auto& t : threads) {
    if (t.joinable()) t.join();
  }

  // Find the depot with the minimum score
  auto best_depot_it = min_element(depotScores.begin(), depotScores.end());

  // Check if any depot was valid
  if (best_depot_it == depotScores.end() ||
      best_depot_it->first == numeric_limits<double>::max()) {
    cerr << "Warning: No end depot is reachable from all dropoff locations."
         << endl;
    return depots[0];  // Fallback
  }

  return best_depot_it->second;
}
#include <algorithm>  // For std::reverse, std::min
#include <chrono>
#include <iostream>  // For cerr
#include <limits>
#include <vector>

// Assuming these are defined elsewhere:
// struct Solution { ... };
// struct DeliveryInf { ... };
// struct DistanceCache { ... double get(...) const; };
// bool isSolutionLegal(const vector<int>& deliveryOrder);
// double evaluateSolution(const vector<int>& deliveryOrder, const
// vector<DeliveryInf>& deliveries, IntersectionIdx startDepot, IntersectionIdx
// endDepot, const DistanceCache& distance_cache); IntersectionIdx
// getStopLocation(int stop_code, const vector<DeliveryInf>& deliveries);

// Make sure these constants are defined appropriately in your global scope or
// adjustParametersForProblemSize Example values: const float SOFT_TIME_LIMIT
// = 45.0f; const float TWO_OPT_TIME_FACTOR = 0.95f; const int
// TWO_OPT_MAX_NO_IMPROVEMENT_PASSES = 10;

// *** Ensure HELPER FUNCTION is defined before this function ***
// Helper function to get the IntersectionIdx for a stop code or depot based on
// position in the order Returns -1 if index is invalid or lookup fails
IntersectionIdx getLocationForOrderIndex(
    int order_index,           // -1 for start depot, size() for end depot
    const vector<int>& order,  // Use const&
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot) {
  if (order_index == -1) return startDepot;
  if (order_index == (int)order.size()) return endDepot;
  if (order_index < 0 ||
      order_index >= (int)order.size()) {  // Check bounds correctly
    // cerr << "Error: Invalid index in getLocationForOrderIndex: " <<
    // order_index << " size: " << order.size() << endl;
    return -1;  // Indicate error
  }

  int stop_code = order[order_index];
  // getStopLocation already handles bounds check for deliveries vector
  return getStopLocation(
      stop_code, deliveries);  // Assuming getStopLocation is defined elsewhere
}

// 2-Opt Optimization with Delta Evaluation and Corrected Time Update
void twoOptOptimization(
    Solution& solution,  // Pass by reference
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot, const DistanceCache& distance_cache,
    const chrono::high_resolution_clock::time_point& startTime) {
  // Skip if factor is 0 or solution is too small for 2-opt
  // *** Ensure TWO_OPT_TIME_FACTOR is defined ***
  if (TWO_OPT_TIME_FACTOR <= 0.0f || solution.deliveryOrder.size() < 2) {
    return;
  }

  // Ensure initial travel time is calculated (should be done by previous
  // phases)
  if (solution.travelTime == numeric_limits<double>::max()) {
    solution.travelTime =
        evaluateSolution(solution.deliveryOrder, deliveries, startDepot,
                         endDepot, distance_cache);
    if (solution.travelTime == numeric_limits<double>::max())
      return;  // Cannot optimize invalid solution
  }

  int n = solution.deliveryOrder.size();
  bool improvement_found_in_pass = true;
  int consecutive_passes_without_improvement = 0;

  // Temporary vector for legality check after reversal
  vector<int> temp_segment;
  const double EPSILON = 1e-9;  // Tolerance for float comparison

  // *** Ensure TWO_OPT_MAX_NO_IMPROVEMENT_PASSES is defined ***
  while (improvement_found_in_pass && consecutive_passes_without_improvement <
                                          TWO_OPT_MAX_NO_IMPROVEMENT_PASSES) {
    // Check time limit at the start of each full pass - Use SOFT limit deadline
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count() > SOFT_TIME_LIMIT * TWO_OPT_TIME_FACTOR) {
      return;  // Stop if time limit exceeded
    }

    improvement_found_in_pass = false;
    double best_delta_this_pass = 0.0;  // Track best improvement found *in this
                                        // pass* (most negative delta)
    int best_i = -1, best_j = -1;

    // Iterate through all possible pairs of edges to swap
    // Edge 1: (i-1) -> i
    // Edge 2: (j-1) -> j (or (n-1) -> endDepot if j==n)
    for (int i = 0; i < n; ++i) {  // Index of the node *after* the first break
      for (int j = i + 1; j <= n;
           ++j) {  // Index of the node *after* the second break (j=n means
                   // break before end depot)

        // Check time limit frequently within inner loops
        if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                     startTime)
                .count() > SOFT_TIME_LIMIT * TWO_OPT_TIME_FACTOR) {
          return;
        }

        // --- Calculate Delta Cost ---
        // *** FIX: Pass solution.deliveryOrder instead of undefined 'order' ***
        IntersectionIdx node_im1 = getLocationForOrderIndex(
            i - 1, solution.deliveryOrder, deliveries, startDepot, endDepot);
        IntersectionIdx node_i = getLocationForOrderIndex(
            i, solution.deliveryOrder, deliveries, startDepot, endDepot);
        IntersectionIdx node_jm1 = getLocationForOrderIndex(
            j - 1, solution.deliveryOrder, deliveries, startDepot, endDepot);
        IntersectionIdx node_j = getLocationForOrderIndex(
            j, solution.deliveryOrder, deliveries, startDepot, endDepot);

        // Check for errors from getLocationForOrderIndex
        if (node_im1 == -1 || node_i == -1 || node_jm1 == -1 || node_j == -1)
          continue;

        // Original edge costs
        double cost_im1_i = distance_cache.get(node_im1, node_i);
        double cost_jm1_j = distance_cache.get(node_jm1, node_j);

        // New edge costs after reversing segment i to j-1
        double cost_im1_jm1 = distance_cache.get(node_im1, node_jm1);
        double cost_i_j = distance_cache.get(node_i, node_j);

        // Check if any path segment is invalid/infinite
        if (cost_im1_i == numeric_limits<double>::max() ||
            cost_jm1_j == numeric_limits<double>::max() ||
            cost_im1_jm1 == numeric_limits<double>::max() ||
            cost_i_j == numeric_limits<double>::max()) {
          continue;  // Cannot evaluate this move if any segment is disconnected
        }

        // Calculate delta cost
        double delta_time =
            (cost_im1_jm1 + cost_i_j) - (cost_im1_i + cost_jm1_j);

        // --- Check for Improvement and Legality (Best Improvement Strategy
        // within Pass) ---
        if (delta_time <
            best_delta_this_pass -
                EPSILON) {  // Found a potentially better move (use tolerance)

          // *** Check Legality ***
          // Only construct the temporary vector if needed for check
          vector<int> temp_full_order;
          temp_full_order.reserve(n);
          temp_full_order.insert(temp_full_order.end(),
                                 solution.deliveryOrder.begin(),
                                 solution.deliveryOrder.begin() + i);
          // Add the reversed segment i to j-1
          if (j > i) {  // Check if segment is non-empty
            temp_segment.assign(solution.deliveryOrder.begin() + i,
                                solution.deliveryOrder.begin() + j);
            reverse(temp_segment.begin(), temp_segment.end());
            temp_full_order.insert(temp_full_order.end(), temp_segment.begin(),
                                   temp_segment.end());
          }
          // Add the remaining part from j onwards
          temp_full_order.insert(temp_full_order.end(),
                                 solution.deliveryOrder.begin() + j,
                                 solution.deliveryOrder.end());

          if (isSolutionLegal(temp_full_order)) {
            // If legal and better than the best found *so far in this pass*
            best_delta_this_pass = delta_time;
            best_i = i;
            best_j = j;
            improvement_found_in_pass =
                true;  // Mark that *an* improvement was found this pass
          }
        }
      }  // end j loop
    }  // end i loop

    // --- Apply the Best Move Found in the Pass ---
    if (improvement_found_in_pass) {  // Check if best_i != -1 is redundant if
                                      // best_delta_this_pass is initialized to
                                      // 0
      // Apply the reverse in-place
      reverse(solution.deliveryOrder.begin() + best_i,
              solution.deliveryOrder.begin() + best_j);

      // *** CORRECTION: Recalculate total time instead of adding delta ***
      solution.travelTime =
          evaluateSolution(solution.deliveryOrder, deliveries, startDepot,
                           endDepot, distance_cache);

      // Optional: Check if recalculation failed (shouldn't if legality check
      // passed and segments were valid)
      if (solution.travelTime == numeric_limits<double>::max()) {
        cerr
            << "Error: 2-Opt move resulted in invalid time after recalculation!"
            << endl;
        // Revert the move for safety
        reverse(solution.deliveryOrder.begin() + best_i,
                solution.deliveryOrder.begin() + best_j);
        // Recalculate time again after reverting
        solution.travelTime =
            evaluateSolution(solution.deliveryOrder, deliveries, startDepot,
                             endDepot, distance_cache);
        improvement_found_in_pass =
            false;  // Treat as no improvement found this pass
        consecutive_passes_without_improvement++;  // Increment counter as if no
                                                   // improvement
      } else {
        consecutive_passes_without_improvement =
            0;  // Reset counter only if successful
                // Optional: cout << "2-Opt Improvement Applied. New Time: " <<
                // solution.travelTime << endl;
      }
    } else {
      consecutive_passes_without_improvement++;  // Increment if no improvement
                                                 // found
    }

    // Check if loop should terminate based on passes without improvement
    if (consecutive_passes_without_improvement >=
        TWO_OPT_MAX_NO_IMPROVEMENT_PASSES) {
      improvement_found_in_pass = false;  // Signal loop to stop
    }

  }  // End of while loop (passes)
}

// 3-Opt optimization: Iteratively improves a solution by considering breaks at
// 3 points. This version is adapted from the provided PDF code. WARNING: This
// implementation generates many temporary vectors and calls evaluateSolution
// frequently within O(N^3) loops, which can be very slow.
void threeOptOptimization(
    Solution& solution,  // Pass by reference
    float turn_penalty, const vector<DeliveryInf>& deliveries,
    IntersectionIdx startDepot, IntersectionIdx endDepot,
    const DistanceCache& distance_cache,
    const chrono::high_resolution_clock::time_point& startTime) {
  // Skip if factor is 0 or solution is too small
  if (THREE_OPT_TIME_FACTOR <= 0.0f ||
      solution.deliveryOrder.size() < 3) {  // Need at least 3 edges for 3-opt
    return;
  }

  // Ensure initial travel time is calculated
  if (solution.travelTime == numeric_limits<double>::max()) {
    solution.travelTime =
        evaluateSolution(solution.deliveryOrder, deliveries, startDepot,
                         endDepot, distance_cache);
    if (solution.travelTime == numeric_limits<double>::max())
      return;  // Cannot optimize invalid solution
  }

  bool improvement_found_in_pass = true;           // Control the outer loop
  int consecutive_passes_without_improvement = 0;  // Counter

  // Use the global legality check function from the Canvas code
  // auto isLegal = [](const vector<int>& order) { ... }; // Remove local lambda

  while (improvement_found_in_pass) {
    // Check time limit at the start of each full pass - Use SOFT limit
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count() > SOFT_TIME_LIMIT * THREE_OPT_TIME_FACTOR) {
      return;  // Stop if time limit exceeded
    }

    improvement_found_in_pass = false;  // Assume no improvement in this pass

    // O(N^3) loops for 3-Opt
    for (size_t i = 0; i < solution.deliveryOrder.size(); ++i) {
      for (size_t j = i + 1; j < solution.deliveryOrder.size(); ++j) {
        for (size_t k = j + 1; k < solution.deliveryOrder.size(); ++k) {
          // Check time limit frequently within the loops - Use SOFT limit
          if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                       startTime)
                  .count() > SOFT_TIME_LIMIT * THREE_OPT_TIME_FACTOR) {
            return;
          }

          // --- Generate 3-Opt Candidates (as per PDF logic) ---
          // WARNING: Creating these vectors repeatedly is inefficient!
          const vector<int>& original = solution.deliveryOrder;
          vector<vector<int>> candidates;
          candidates.reserve(7);  // Reserve space

          // 1. Reverse segment i to j (Equivalent to a 2-Opt move)
          vector<int> m1 = original;
          reverse(m1.begin() + i, m1.begin() + j);
          candidates.push_back(m1);

          // 2. Reverse segment j to k (Equivalent to a 2-Opt move)
          vector<int> m2 = original;
          reverse(m2.begin() + j, m2.begin() + k);
          candidates.push_back(m2);

          // 3. Reverse segment i to k (Equivalent to a 2-Opt move)
          vector<int> m3 = original;
          reverse(m3.begin() + i, m3.begin() + k);
          candidates.push_back(m3);

          // 4. Reconnect: (..i-1)->(j..k-1)->(i..j-1)->(k..) (Relocate segment
          // i..j-1)
          vector<int> m4;
          m4.reserve(original.size());
          m4.insert(m4.end(), original.begin(), original.begin() + i);
          m4.insert(m4.end(), original.begin() + j, original.begin() + k);
          m4.insert(m4.end(), original.begin() + i, original.begin() + j);
          m4.insert(m4.end(), original.begin() + k, original.end());
          candidates.push_back(m4);

          // 5. Reconnect: Reverse both i-j and j-k
          vector<int> m5 = original;
          reverse(m5.begin() + i, m5.begin() + j);
          reverse(m5.begin() + j, m5.begin() + k);
          candidates.push_back(m5);

          // 6. Reconnect: (..i-1)->(j..k-1)->rev(i..j-1)->(k..)
          vector<int> m6;
          m6.reserve(original.size());
          vector<int> seg1(original.begin() + i, original.begin() + j);
          reverse(seg1.begin(), seg1.end());
          m6.insert(m6.end(), original.begin(), original.begin() + i);
          m6.insert(m6.end(), original.begin() + j, original.begin() + k);
          m6.insert(m6.end(), seg1.begin(), seg1.end());
          m6.insert(m6.end(), original.begin() + k, original.end());
          candidates.push_back(m6);

          // 7. Reconnect: (..i-1)->rev(j..k-1)->(i..j-1)->(k..)
          // PDF logic for m7 seems incorrect/same as m6? Let's try the other
          // standard 3-opt: Reconnect: (..i-1)->(i..j-1)->rev(j..k-1)->(k..)
          // (This is just m2) Let's try:
          // (..i-1)->rev(j..k-1)->rev(i..j-1)->(k..)
          vector<int> m7;
          m7.reserve(original.size());
          vector<int> seg2_rev(original.begin() + i, original.begin() + j);
          reverse(seg2_rev.begin(), seg2_rev.end());
          vector<int> seg3_rev(original.begin() + j, original.begin() + k);
          reverse(seg3_rev.begin(), seg3_rev.end());
          m7.insert(m7.end(), original.begin(), original.begin() + i);
          m7.insert(m7.end(), seg3_rev.begin(), seg3_rev.end());
          m7.insert(m7.end(), seg2_rev.begin(), seg2_rev.end());
          m7.insert(m7.end(), original.begin() + k, original.end());
          candidates.push_back(m7);

          // --- Evaluate all candidates for this i, j, k ---
          double bestCandidateTime =
              solution.travelTime;  // Start with current time
          vector<int> bestCandidateOrder =
              original;  // Start with current order
          bool candidateFoundImprovement = false;

          for (const auto& candidate_order : candidates) {
            // Check legality using the global function
            if (!isSolutionLegal(candidate_order)) continue;

            Solution tempSol;
            tempSol.deliveryOrder = candidate_order;
            tempSol.travelTime =
                evaluateSolution(tempSol.deliveryOrder, deliveries, startDepot,
                                 endDepot, distance_cache);

            if (tempSol.travelTime != numeric_limits<double>::max() &&
                tempSol.travelTime < bestCandidateTime) {
              bestCandidateTime = tempSol.travelTime;
              bestCandidateOrder = candidate_order;
              candidateFoundImprovement = true;
            }
          }

          // If the best candidate for this i,j,k improved the overall solution
          if (candidateFoundImprovement) {
            solution.deliveryOrder = bestCandidateOrder;
            solution.travelTime = bestCandidateTime;
            improvement_found_in_pass = true;  // Mark improvement for this pass
            // Using best improvement strategy within the pass
          }
        }  // end k loop
      }  // end j loop
    }  // end i loop

    // Update consecutive passes counter
    if (!improvement_found_in_pass) {
      consecutive_passes_without_improvement++;
    } else {
      consecutive_passes_without_improvement =
          0;  // Reset if improvement was found
    }

    // Stop if no improvement was found for a set number of consecutive passes
    if (consecutive_passes_without_improvement >=
        THREE_OPT_MAX_NO_IMPROVEMENT_PASSES) {
      break;
    }

  }  // End of while loop
}

// --- MODIFIED parallelMultiStartOptimizer ---
vector<Solution> parallelMultiStartOptimizer(
    const vector<DeliveryInf>& deliveries, IntersectionIdx startDepot,
    IntersectionIdx endDepot, float turn_penalty,
    const DistanceCache& distance_cache,
    const chrono::high_resolution_clock::time_point& startTime,
    const Solution& commonInitialSolution) {
  vector<Solution> solutions(
      NUM_THREADS);  // Store the best solution from each thread
  vector<thread> threads;

  // Worker function for each optimization thread
  auto worker = [&](int thread_id) {
    // Use thread-local random number generator, seeded uniquely
    random_device rd;
    mt19937 rng(rd() ^
                ((uint64_t)hash<thread::id>{}(this_thread::get_id()) << 32 |
                 thread_id));

    // Start with a copy of the common initial solution
    Solution current_best = commonInitialSolution;

    // Check if initial solution is valid before proceeding
    if (current_best.travelTime == numeric_limits<double>::max()) {
      solutions[thread_id] = current_best;  // Store invalid solution
      return;  // Cannot optimize if initial is invalid
    }

    // Apply a few initial random perturbations to diversify starting points
    // (unless it's thread 0)
    if (thread_id > 0) {
      int initial_perturbs = 5;  // Number of initial perturbations
      for (int p = 0; p < initial_perturbs; ++p) {
        current_best = perturbSolution(current_best, deliveries,
                                       rng);  // Assumes perturbSolution exists
      }
      // Re-evaluate travel time after initial perturbations
      current_best.travelTime =
          evaluateSolution(current_best.deliveryOrder, deliveries, startDepot,
                           endDepot, distance_cache);
      if (current_best.travelTime == numeric_limits<double>::max()) {
        solutions[thread_id] = current_best;
        return;
      }
    }
    std::cout << "Before " << thread_id << ": travel time is now"
              << current_best.travelTime << "s" << std::endl;
    Solution old_best = current_best;
    //--- Hill Climbing Phase ---
    // Check time before starting HC
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count() < SOFT_TIME_LIMIT * HC_TIME_FACTOR) {
      current_best =
          hillClimbing(current_best, turn_penalty, deliveries, startDepot,
                       endDepot, distance_cache, startTime);
    }
    std::cout << "After Hill climbing " << thread_id << ": travel time is now"
              << old_best.travelTime - current_best.travelTime << "s"
              << std::endl;
    old_best = current_best;

    // --- Simulated Annealing Phase ---
    // Check time before starting SA
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count() < SOFT_TIME_LIMIT * SA_TIME_FACTOR) {
      current_best =
          simulatedAnnealing(current_best, turn_penalty, deliveries, startDepot,
                             endDepot, distance_cache, startTime);
    }
    std::cout << "After Simulated anealing " << thread_id
              << ": travel time is now"
              << old_best.travelTime - current_best.travelTime << "s"
              << std::endl;
    old_best = current_best;

    // --- 2-Opt Phase ---
    // Check time before starting 2-Opt
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count() < SOFT_TIME_LIMIT * TWO_OPT_TIME_FACTOR) {
      twoOptOptimization(current_best, deliveries, startDepot, endDepot,
                         distance_cache, startTime);
    }
    std::cout << "After Two Opt  " << thread_id << ": travel time is now"
              << old_best.travelTime - current_best.travelTime << "s"
              << std::endl;
    old_best = current_best;
    double elapsed_before_3opt =
        chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count();
    double remaining_time = SOFT_TIME_LIMIT - elapsed_before_3opt;
    double required_remaining_time =
        SOFT_TIME_LIMIT * MIN_REMAINING_TIME_FACTOR_FOR_3OPT;

    // Check time before starting 3-Opt - Use SOFT limit and the correct factor
    if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                                 startTime)
            .count() <
        SOFT_TIME_LIMIT * THREE_OPT_TIME_FACTOR) {  // Use THREE_OPT factor
      // Call the new 3-Opt function
      std::cout << "Thread " << thread_id << ": Enough time remaining ("
                << remaining_time << "s), running 3-Opt." << std::endl;

      threeOptOptimization(current_best, turn_penalty, deliveries, startDepot,
                           endDepot, distance_cache, startTime);
    }
    std::cout << "After Three Opt  " << thread_id << ": travel time is now"
              << old_best.travelTime - current_best.travelTime << "s"
              << std::endl;
    old_best = current_best;

    // Store the best solution found by this thread
    solutions[thread_id] = current_best;
  };

  // Launch optimization threads
  for (int i = 0; i < NUM_THREADS; ++i) {
    threads.emplace_back(worker, i);
  }

  // Wait for all threads to complete
  for (auto& t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }

  return solutions;  // Return the collection of best solutions from each thread
}
// --- Final Path Construction ---

// Constructs the final vector of CourierSubPath objects from the best found
// solution order. Uses parallel calls to findPathBetweenIntersections (m3
// function) for efficiency.
vector<CourierSubPath> buildFinalPath(const Solution& bestSolution,
                                      const vector<DeliveryInf>& deliveries,
                                      IntersectionIdx startDepot,
                                      IntersectionIdx endDepot,
                                      float turn_penalty) {
  vector<CourierSubPath> final_route;
  vector<IntersectionIdx>
      route_intersections;  // Sequence of intersections visited

  // Start at the chosen start depot
  route_intersections.push_back(startDepot);

  // Add intersections from the delivery order
  unordered_set<int> pickedUpItems;  // Keep track locally for path construction
  for (int stop_code : bestSolution.deliveryOrder) {
    int deliveryIndex = stop_code / 2;
    bool isPickup = (stop_code % 2 == 0);
    IntersectionIdx location;

    if (isPickup) {
      location = deliveries[deliveryIndex].pickUp;
      pickedUpItems.insert(
          deliveryIndex);  // Mark item as picked up for construction logic
    } else {
      // Legality should already be guaranteed by the optimization process,
      // but double-checking doesn't hurt.
      if (pickedUpItems.find(deliveryIndex) == pickedUpItems.end()) {
        // This indicates a bug in the optimization's legality checks
        cerr << "Error: Trying to build path for illegal dropoff!" << endl;
        return {};  // Return empty path if illegal state reached
      }
      location = deliveries[deliveryIndex].dropOff;
    }
    // Avoid adding duplicate consecutive intersections if a pickup/dropoff
    // happens at the same place
    if (route_intersections.empty() || route_intersections.back() != location) {
      route_intersections.push_back(location);
    }
  }

  // Add the end depot
  // Avoid adding duplicate if the last delivery was at the end depot
  if (route_intersections.empty() || route_intersections.back() != endDepot) {
    route_intersections.push_back(endDepot);
  }

  // --- Parallel Path Finding for Subpaths ---
  vector<future<vector<StreetSegmentIdx>>> path_futures;
  // Request paths between consecutive intersections in the route
  for (size_t i = 0; i < route_intersections.size() - 1; ++i) {
    IntersectionIdx from = route_intersections[i];
    IntersectionIdx to = route_intersections[i + 1];
    // Only find path if start and end are different
    if (from != to) {
      // Asynchronously call the m3 pathfinding function
      path_futures.push_back(async(launch::async, findPathBetweenIntersections,
                                   turn_penalty, make_pair(from, to)));
    } else {
      // If start and end are the same, push a future containing an empty path
      promise<vector<StreetSegmentIdx>> p;
      p.set_value({});  // Set an empty vector
      path_futures.push_back(p.get_future());
    }
  }

  // --- Assemble Final Route ---
  size_t future_idx = 0;
  for (size_t i = 0; i < route_intersections.size() - 1; ++i) {
    IntersectionIdx from = route_intersections[i];
    IntersectionIdx to = route_intersections[i + 1];

    // Retrieve the computed path from the future
    vector<StreetSegmentIdx> subpath_segments =
        path_futures[future_idx++].get();

    // Check if the pathfinding succeeded (returned non-empty) OR if it was a
    // zero-length segment (from == to)
    if (!subpath_segments.empty() || from == to) {
      // Only add non-empty paths to the final_route vector
      if (!subpath_segments.empty()) {
        final_route.emplace_back(CourierSubPath{{from, to}, subpath_segments});
      }
      // If from == to, we don't add a subpath object, as no travel occurred.
      // The sequence of intersections is correctly maintained by
      // route_intersections.
    } else {
      // If findPathBetweenIntersections returned empty for different 'from' and
      // 'to' intersections, it means no path exists between them, making the
      // whole route invalid.
      cerr << "Error: No path found between " << from << " and " << to
           << ". Route invalid." << endl;
      return {};  // Return empty vector indicating failure
    }
  }

  return final_route;
}

// --- Main Function ---

// The main entry point for the Traveling Courier problem.
vector<CourierSubPath> travelingCourier(float turn_penalty,
                                        const vector<DeliveryInf>& deliveries,
                                        const vector<IntersectionIdx>& depots) {
  auto overallStartTime = chrono::high_resolution_clock::now();

  // Handle edge cases: no deliveries or no depots
  if (deliveries.empty() || depots.empty()) {
    return {};  // Return an empty vector as no route is needed or possible
  }

  // Adjust parameters based on problem size
  adjustParametersForProblemSize(deliveries.size());

  // 1. Precompute distances between all relevant locations
  DistanceCache distance_cache = precomputeDistancesParallel(
      depots, deliveries, turn_penalty, overallStartTime);
  // Check if precomputation timed out significantly (based on SOFT limit)
  if (chrono::duration<double>(chrono::high_resolution_clock::now() -
                               overallStartTime)
          .count() > SOFT_TIME_LIMIT * (PRECOMPUTATION_TIME_FACTOR +
                                        0.05)) {  // Add small buffer
    cerr << "Warning: Precomputation took too long or timed out, results might "
            "be suboptimal."
         << endl;
    // Continue, but the cache might be incomplete.
  }

  // 2. Find best *potential* end depot (needed for evaluating initial
  // solutions)
  IntersectionIdx bestOverallEndDepot =
      findBestEndDepotParallel(depots, deliveries, distance_cache);
  if (bestOverallEndDepot == -1) {
    cerr << "Error: Could not determine valid end depot (potentially "
            "unreachable)."
         << endl;
    return {};  // Cannot proceed
  }

  // 3. Generate Initial Solutions in Parallel (starting from *each* depot)
  vector<pair<Solution, IntersectionIdx>> initial_solution_pairs(depots.size());
  vector<thread> initial_threads;
  atomic<size_t> next_depot_idx_atomic(0);
  atomic<bool> initial_timed_out(
      false);  // Timeout flag for initial generation phase

  // Worker task for generating initial solutions
  auto initial_worker_task = [&]() {
    while (true) {
      size_t idx = next_depot_idx_atomic.fetch_add(1);
      if (idx >= depots.size()) break;  // No more depots to process

      // Check time limit before starting generation for this depot - Use SOFT
      // limit
      if (initial_timed_out.load() ||
          chrono::duration<double>(chrono::high_resolution_clock::now() -
                                   overallStartTime)
                  .count() > SOFT_TIME_LIMIT * (PRECOMPUTATION_TIME_FACTOR +
                                                INITIAL_SOLUTION_TIME_FACTOR)) {
        initial_timed_out.store(true);           // Signal timeout
        initial_solution_pairs[idx] = {{}, -1};  // Mark as invalid
        break;  // Stop processing more depots in this thread
      }

      IntersectionIdx potentialStartDepot = depots[idx];
      Solution sol = generateInitialSolution_NearestNeighbor(
          deliveries, potentialStartDepot, bestOverallEndDepot, distance_cache,
          overallStartTime);
      initial_solution_pairs[idx] = {sol, potentialStartDepot};
    }
  };

  // Launch threads to generate initial solutions
  int num_initial_threads = min((int)depots.size(), NUM_THREADS);
  for (int i = 0; i < num_initial_threads; ++i)
    initial_threads.emplace_back(initial_worker_task);
  for (auto& t : initial_threads)
    if (t.joinable()) t.join();

  // Find the best initial solution from all generated candidates
  Solution bestInitialSolution;           // Default time is max
  IntersectionIdx actualStartDepot = -1;  // Initialize as invalid
  for (const auto& sol_pair : initial_solution_pairs) {
    // Check if the solution is valid (depot index is valid and travel time is
    // not max)
    if (sol_pair.second != -1 &&
        sol_pair.first.travelTime != numeric_limits<double>::max()) {
      if (sol_pair.first.travelTime < bestInitialSolution.travelTime) {
        bestInitialSolution = sol_pair.first;
        actualStartDepot = sol_pair.second;
      }
    }
  }

  // Check if *any* valid initial solution was found
  if (actualStartDepot == -1) {
    cerr << "Error: Failed to generate any valid initial solution from any "
            "depot."
         << endl;
    return {};
  }

  // 4. Run the multi-start optimization pipeline using the best initial
  // solution found
  vector<Solution> final_solutions = parallelMultiStartOptimizer(
      deliveries, actualStartDepot, bestOverallEndDepot, turn_penalty,
      distance_cache, overallStartTime, bestInitialSolution);

  // 5. Find the best solution among all optimization threads
  // Initialize with the best *initial* solution found before optimization
  Solution bestOverallSolution = bestInitialSolution;
  for (const auto& sol : final_solutions) {
    // Check if the optimized solution is valid and better than the current best
    if (sol.travelTime != numeric_limits<double>::max() &&
        sol.travelTime < bestOverallSolution.travelTime) {
      bestOverallSolution = sol;
    }
  }

  // 6. Check if a valid solution exists after optimization
  if (bestOverallSolution.travelTime == numeric_limits<double>::max()) {
    cerr << "Warning: No valid solution found after optimization phase."
         << endl;
    // The best we had was the initial solution, but if it was also invalid,
    // return empty (This case is handled by the actualStartDepot == -1 check
    // earlier) If bestInitialSolution was valid, but optimization failed, we
    // proceed with bestInitialSolution
    if (!isSolutionLegal(
            bestOverallSolution
                .deliveryOrder)) {  // Double check legality if needed
      return {};
    }
    cerr << "Info: Proceeding with best initial solution found before "
            "optimization."
         << endl;
  }

  // Check remaining time before final path construction - Use SOFT limit as a
  // warning
  double time_elapsed =
      chrono::duration<double>(chrono::high_resolution_clock::now() -
                               overallStartTime)
          .count();
  if (time_elapsed >
      SOFT_TIME_LIMIT * 0.98) {  // Leave a tiny buffer relative to SOFT limit
    cerr
        << "Warning: Approaching SOFT time limit before final path "
           "construction. Path quality may be affected if construction is slow."
        << endl;
  }

  // 7. Construct the final path from the best solution order
  vector<CourierSubPath> result_path =
      buildFinalPath(bestOverallSolution, deliveries, actualStartDepot,
                     bestOverallEndDepot, turn_penalty);

  // Final time check
  double final_time_elapsed =
      chrono::duration<double>(chrono::high_resolution_clock::now() -
                               overallStartTime)
          .count();
  cout << "Info: Total time taken: " << final_time_elapsed << "s / "
       << HARD_TIME_LIMIT << "s" << endl;  // Report against HARD limit
  if (bestOverallSolution.travelTime != numeric_limits<double>::max()) {
    cout << "Info: Best solution travel time: "
         << bestOverallSolution.travelTime << endl;
  } else {
    cout << "Info: Best solution found was invalid." << endl;
  }

  // *** MODIFIED: Removed the check that returns {} if HARD_TIME_LIMIT is
  // exceeded ***
  // // Ensure we don't exceed the HARD time limit even if path building was
  // slow if (final_time_elapsed > HARD_TIME_LIMIT) { // *** Use HARD limit for
  // final cutoff ***
  //     cerr << "Error: Exceeded HARD time limit (" << final_time_elapsed <<
  //     "s). Returning empty path." << endl; return {};
  // }

  return result_path;  // Always return the constructed path (which might be
                       // empty if buildFinalPath failed internally)
}

// Path ending ------------------------------

extern vector<vector<TypedOSMID>> subway_osm_ids;
void draw_main_canvas(ezgl::renderer* g) {
  draw_map_contents(g);  // Draw the map contents
}

gboolean toggle_night_mode(GtkSwitch* /*darkModeSwitch*/, gboolean switch_state,
                           ezgl::application* application) {
  night_mode = switch_state;       // Toggle night mode
  application->refresh_drawing();  // Redraw
  return false;
}
// A callback for getting the text from TextEntry; called when the user hits
// enter
void textEntry_enter(GtkEntry* textEntryWidget,
                     ezgl::application* application) {
  // Get the text from the TextEntry widget
  const gchar* text = gtk_entry_get_text(textEntryWidget);
  // draw this text in the status bar at the bottom of the application
  search_bar_click(text, application);
  application->update_message(text);
}

void initial_setup(ezgl::application* application, bool /*new_window*/) {
  // Update status bar message
  application->update_message("EZGL Application");

  // Set the starting row for UI elements (default zoom/pan buttons take up the
  // first five rows)
  int row = 6;

  application->create_label(row++, "Options: ");

  // Create the "Load Map" button and link it to its callback function
  application->create_button("Load Map", row++, load_button_click);
  application->create_button("Help", row++, load_button_click_help);
  application->create_button("Find Route", row++, find_route_button_click);

  GObject* darkModeSwitch = application->get_object("darkModeSwitch");
  g_signal_connect(darkModeSwitch, "state-set", G_CALLBACK(toggle_night_mode),
                   application);
  GObject* textEntry = application->get_object("word");
  g_signal_connect(textEntry, "activate", G_CALLBACK(textEntry_enter),
                   application);
}
void find_route_button_click(GtkWidget* /*widget*/,
                             ezgl::application* application) {
  if (find_route == false && find_route_intersections.size() == 0) {
    find_route = true;
  } else {
    find_route_intersections.clear();
    application->refresh_drawing();
  }
}

void preprocessFeatures() {
  features_data.reserve(getNumFeatures());  // Reserve space for efficiency

  for (int i = 0; i < getNumFeatures(); ++i) {
    FeatureData feature;
    feature.type = getFeatureType(i);

    // Precompute points in meters
    for (int j = 0; j < getNumFeaturePoints(i); ++j) {
      vector<double> coords = getCoordinateMeters(getFeaturePoint(i, j));
      feature.points.emplace_back(coords[0], coords[1]);
    }

    // Compute area for lakes
    if (feature.type == LAKE) {
      feature.area = findFeatureArea(i);
    } else {
      feature.area = 0.0;
    }

    features_data.push_back(feature);
  }
}

// Helper function to calculate the distance from a point to a line segment
double pointToLineDistance(double px, double py, double x1, double y1,
                           double x2, double y2) {
  double A = px - x1;
  double B = py - y1;
  double C = x2 - x1;
  double D = y2 - y1;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;
  if (param < 0) {
    xx = x1;
    yy = y1;
  } else if (param > 1) {
    xx = x2;
    yy = y2;
  } else {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  double dx = px - xx;
  double dy = py - yy;
  return sqrt(dx * dx + dy * dy);
}

std::vector<ClickableObject> clickableObjects;

// Function to determine the priority of a clickable object type
int getPriority(ClickableObject::Type type) {
  switch (type) {
    case ClickableObject::POI:
      return 0;  // Highest priority
    case ClickableObject::INTERSECTION:
      return 1;
    case ClickableObject::STREET_SEGMENT:
      return 2;
    case ClickableObject::FEATURE:
      return 3;  // Lowest priority
    default:
      return 4;  // Invalid type
  }
}

// Function to find the closest clickable object within a radius
std::pair<int, ClickableObject::Type> clicked(double x, double y) {
  const double CLICK_RADIUS = 10.0;  // 10 meters
  double closestDistance = std::numeric_limits<double>::infinity();
  int highestPriority =
      std::numeric_limits<int>::max();  // Start with the lowest possible
                                        // priority
  std::pair<int, ClickableObject::Type> result = {
      -1, ClickableObject::STREET_SEGMENT};  // Default invalid result

  for (const auto& obj : clickableObjects) {
    double dist = 0.0;

    if (obj.type == ClickableObject::STREET_SEGMENT ||
        obj.type == ClickableObject::FEATURE) {
      // Check distance to line segments
      double minDist = std::numeric_limits<double>::infinity();
      for (size_t i = 1; i < obj.geometry.size(); ++i) {
        double segmentDist = pointToLineDistance(
            x, y, obj.geometry[i - 1].first, obj.geometry[i - 1].second,
            obj.geometry[i].first, obj.geometry[i].second);
        minDist = std::min(minDist, segmentDist);
      }
      dist = minDist;
    } else {
      // Check distance to point
      dist = sqrt(pow(x - obj.center.first, 2) + pow(y - obj.center.second, 2));
    }

    if (dist <= CLICK_RADIUS) {
      int priority = getPriority(obj.type);

      // Update result if:
      // 1. The current object has a higher priority, or
      // 2. The current object has the same priority but is closer.
      if (priority < highestPriority ||
          (priority == highestPriority && dist < closestDistance)) {
        closestDistance = dist;
        highestPriority = priority;
        result = {obj.id, obj.type};
      }
    }
  }

  return result;
}

std::unordered_map<std::string, double> road_thicknesses = {
    {"motorway", 4},       // Highways: thickest
    {"trunk", 4},          // Major roads: slightly thinner
    {"primary", 3.5},      // Important roads: medium thickness
    {"secondary", 3},      // Medium roads: thinner
    {"tertiary", 2},       // Smaller roads: even thinner
    {"residential", 2.0},  // Residential roads: very thin
    {"service", 1.5},      // Service roads: thinnest
    {"footway", 1.0}       // Footpaths: thinnest
};

// Default road color
ezgl::color default_road_color = ezgl::GREY_55;

/**
 * Function to get classification for a given OSMID.
 * @param id The OpenStreetMap ID of the feature.
 * @return The classification string if found, or an empty string otherwise.
 */
std::string getClassificationForOSMID(OSMID id) {
  auto it = osm_id_to_classification.find(id);
  if (it != osm_id_to_classification.end()) {
    return it->second;  // Return the classification if found
  }
  return "";  // Return an empty string if not found
}
void draw_one_way_arrows(ezgl::renderer* g,
                         const StreetSegmentInfo& segmentInfo, int id) {
  g->set_color(ezgl::ORANGE);

  // Get the start and end points
  std::vector<double> segFrom =
      getCoordinateMeters(intersection_positions[segmentInfo.from]);
  std::vector<double> segTo =
      getCoordinateMeters(intersection_positions[segmentInfo.to]);

  std::vector<std::pair<double, double>> points;
  points.emplace_back(segFrom[0], segFrom[1]);  // Start point

  // Add curve points if they exist
  for (int j = 0; j < segmentInfo.numCurvePoints; ++j) {
    std::vector<double> curvePoint =
        getCoordinateMeters(getStreetSegmentCurvePoint(id, j));
    points.emplace_back(curvePoint[0], curvePoint[1]);
  }

  points.emplace_back(segTo[0], segTo[1]);  // End point

  // need at least 2 points to draw arrows
  if (points.size() < 2) return;

  double arrowSize = 5.0;  // Size of the arrow

  // Iterate through segments
  for (size_t i = 1; i < points.size(); ++i) {
    double segFromX = points[i - 1].first;
    double segFromY = points[i - 1].second;
    double segToX = points[i].first;
    double segToY = points[i].second;

    // Calculate direction vector
    double dx = segToX - segFromX;
    double dy = segToY - segFromY;
    double length = sqrt(dx * dx + dy * dy);

    if (length > 0) {
      // Normalize (unit vector)
      dx /= length;
      dy /= length;

      int numArrows = std::max(
          1, static_cast<int>(length /
                              50));  // Number of arrows based on segment length

      // Draw multiple arrows along the current segment
      for (int j = 1; j <= numArrows; ++j) {
        double t = j / (double)(numArrows + 1);  // Position along the segment
        double arrowX = segFromX + t * dx * length;
        double arrowY = segFromY + t * dy * length;

        // Calculate perpendicular offset for the rest of the points
        double perpX = -dy * arrowSize * 0.5;
        double perpY = dx * arrowSize * 0.5;

        ezgl::point2d tip(arrowX, arrowY);
        ezgl::point2d base1(arrowX - dx * arrowSize + perpX,
                            arrowY - dy * arrowSize + perpY);
        ezgl::point2d base2(arrowX - dx * arrowSize - perpX,
                            arrowY - dy * arrowSize - perpY);

        // Draw the arrow as a filled triangle
        g->fill_poly({tip, base1, base2});
      }
    }
  }
}

void draw_filled_feature(ezgl::renderer* g,
                         const std::vector<ezgl::point2d>& polygon,
                         ezgl::color fillColor) {
  // Ensure the feature has enough points to form a closed polygon
  if (polygon.size() > 2) {
    g->set_color(fillColor);
    g->fill_poly(polygon);

    // Store feature information for interactivity
    ClickableObject obj;
    obj.type = ClickableObject::FEATURE;
    for (const auto& point : polygon) {
      obj.geometry.emplace_back(point.x, point.y);
    }
    clickableObjects.push_back(obj);
  } else {
    // For features with fewer than 3 points (e.g., lines or points)
    if (polygon.size() == 2) {
      g->set_color(fillColor);
      g->set_line_width(2.0);  // Set line thickness
      g->draw_line(polygon[0], polygon[1]);
      g->set_line_width(1.0);  // Reset line thickness
    } else if (polygon.size() == 1) {
      g->set_color(fillColor);
      g->fill_arc(polygon[0], 5, 0, 360);  // Draw as a small circle
    }

    // Store feature information for interactivity
    ClickableObject obj;
    obj.type = ClickableObject::FEATURE;
    for (const auto& point : polygon) {
      obj.geometry.emplace_back(point.x, point.y);
    }
    clickableObjects.push_back(obj);
  }
}
/**
 * @param g The EZGL renderer object.
 * @param text The text to display in the label.
 * @param x The x-coordinate of the label's position (world coordinates).
 * @param y The y-coordinate of the label's position (world coordinates).
 */
void draw_styled_label(ezgl::renderer* g, const std::string& text, double x,
                       double y) {
  // Approximate the bounding box of the text
  double font_size = 14;
  double text_width = font_size * text.size() * 0.6;
  double text_height = font_size * 1.2;

  const double padding_x = 10;
  const double padding_y = 5;
  double label_width = (text_width + 2 * padding_x) * 2;
  double label_height = text_height + 2 * padding_y;

  // Draw the background rectangle
  g->set_color(0, 0, 0, 150);  // Semi-transparent
  g->fill_rectangle({x - (label_width / 2), y},
                    {x + label_width / 2, y + label_height});

  // Draw the label text
  g->set_color(ezgl::WHITE);  // White text
  g->set_font_size(font_size);

  // Center the text within the label box
  double text_x = x;
  double text_y = y + padding_y + text_height / 2;  // Vertically center text
  g->draw_text({text_x, text_y}, text);
  g->set_color(ezgl::BLUE);
}
void draw_nice_label(ezgl::renderer* g, const std::string& text, double x,
                     double y) {
  const double font_size = 12;
  const double padding_x = 8;
  const double padding_y = 4;

  // Calculate text dimensions
  double text_width = font_size * text.size() * 0.6;
  double text_height = font_size * 1.2;

  // Background rectangle dimensions
  double label_width = text_width + 4 * padding_x;
  double label_height = text_height + 4 * padding_y;

  // Border rectangle dimensions (2px larger on each side)
  double border_width = label_width + 8;
  double border_height = label_height + 8;

  // Position the border rectangle centered at (x,y)
  double border_x = x - (border_width / 2);
  double border_y = y - (border_height / 2);

  // Draw dark blue border
  g->set_color(ezgl::BLUE);  // RGB(0, 0, 139)
  g->fill_rectangle({border_x, border_y}, border_width, border_height);

  // Draw light blue background
  g->set_color(ezgl::LIGHT_SKY_BLUE);  // RGB(173, 216, 230)
  g->fill_rectangle({x - (label_width / 2), y - (label_height / 2)},
                    label_width, label_height);

  // Draw white text
  g->set_color(ezgl::WHITE);
  g->set_font_size(font_size);
  g->draw_text({x, y}, text);
  g->set_color(ezgl::BLUE);  // RGB(0, 0, 139)
}
// Function to check if two labels overlap
bool labelsOverlap(double x1, double y1, double width1, double height1,
                   double x2, double y2, double width2, double height2) {
  return !(x1 + width1 < x2 || x2 + width2 < x1 || y1 + height1 < y2 ||
           y2 + height2 < y1);
}

void drawArrowOnLine(ezgl::renderer* g, const std::vector<double>& from,
                     const std::vector<double>& to, double spacing) {
  double dx = to[0] - from[0];
  double dy = to[1] - from[1];
  double length = sqrt(dx * dx + dy * dy);

  if (length < 1e-6) return;  // Skip zero-length segments

  // Draw the main line segment
  g->draw_line({from[0], from[1]}, {to[0], to[1]});

  double angle = atan2(dy, dx);   // Direction of the segment
  const double arrowSize = 10.0;  // Increased size for better visibility

  // Start at spacing/2 to center arrows and ensure visibility on shorter
  // segments
  double currentDistance = spacing / 2.0;

  while (currentDistance < length) {
    double t = currentDistance / length;
    double midX = from[0] + t * dx;
    double midY = from[1] + t * dy;

    // Calculate arrow points behind midX to point forward
    double angleMinus = angle - M_PI / 6;  // -30 degrees
    double anglePlus = angle + M_PI / 6;   // +30 degrees

    double arrowX1 = midX - arrowSize * cos(angleMinus);
    double arrowY1 = midY - arrowSize * sin(angleMinus);
    double arrowX2 = midX - arrowSize * cos(anglePlus);
    double arrowY2 = midY - arrowSize * sin(anglePlus);

    // Draw the arrowhead lines
    g->draw_line({midX, midY}, {arrowX1, arrowY1});
    g->draw_line({midX, midY}, {arrowX2, arrowY2});

    currentDistance += spacing;
  }
}

/**
 * @param g The renderer object used for drawing.
 */
void draw_map_contents(ezgl::renderer* g) {
  clickableObjects.clear();  // Clear previous data to avoid duplicates

  if (night_mode) {
    default_road_color = ezgl::GREY_75;  // Roads appear white in night mode }
    g->set_color(ezgl::BLACK);           // Set the desired background color
  } else {
    default_road_color = ezgl::GREY_55;
    g->set_color(ezgl::WHITE);  // Set the desired background color
  }
  g->fill_rectangle({0, 0}, {top_right[0], top_right[1]});

  // Step 1: Separate lakes into large and small based on their area
  std::vector<int> large_lakes;
  std::vector<int> small_lakes;

  for (int i = 0; i < features_data.size(); ++i) {
    const FeatureData& feature = features_data[i];
    if (feature.type == LAKE) {
      if (feature.area > 1e6)  // Define "large" as > 1 million square meters
      {
        large_lakes.push_back(i);
      } else {
        small_lakes.push_back(i);
      }
    }
  }

  // Step 2: Draw large lakes first
  for (int id : large_lakes) {
    draw_filled_feature(g, features_data[id].points,
                        night_mode ? ezgl::BLUE : ezgl::LIGHT_SKY_BLUE);
  }

  // Step 3: Draw islands
  for (int i = 0; i < features_data.size(); ++i) {
    const FeatureData& feature = features_data[i];
    if (feature.type == ISLAND) {
      draw_filled_feature(g, feature.points,
                          night_mode ? ezgl::BLACK : ezgl::WHITE);
    }
  }

  // Step 4: Draw greenery (parks, greenspaces, golf courses)
  for (int i = 0; i < features_data.size(); ++i) {
    const FeatureData& feature = features_data[i];
    if (feature.type == PARK || feature.type == GREENSPACE ||
        feature.type == GOLFCOURSE) {
      draw_filled_feature(g, feature.points, ezgl::LIME_GREEN);
    }
  }

  // Step 5: Draw small lakes and other bodies of water (rivers, streams)
  for (int id : small_lakes) {
    draw_filled_feature(g, features_data[id].points,
                        night_mode ? ezgl::BLUE : ezgl::LIGHT_SKY_BLUE);
  }

  for (int i = 0; i < features_data.size(); ++i) {
    const FeatureData& feature = features_data[i];
    if (feature.type == RIVER || feature.type == STREAM) {
      // Rivers and streams are drawn as lines with varying thickness
      double thickness = (feature.type == RIVER)
                             ? 6.0
                             : 3.0;  // Rivers are thicker than streams
      g->set_color(night_mode ? ezgl::BLUE : ezgl::LIGHT_SKY_BLUE);
      g->set_line_width(thickness);

      // Draw the river/stream as a series of connected lines
      for (size_t j = 1; j < feature.points.size(); ++j) {
        g->draw_line(feature.points[j - 1], feature.points[j]);
      }

      // Reset line width after drawing
      g->set_line_width(1.0);
    }
  }

  // Step 6: Draw glaciers
  for (int i = 0; i < features_data.size(); ++i) {
    const FeatureData& feature = features_data[i];
    if (feature.type == GLACIER) {
      draw_filled_feature(g, feature.points, ezgl::WHITE);
    }
  }

  // Step 7: Draw buildings
  for (int i = 0; i < features_data.size(); ++i) {
    const FeatureData& feature = features_data[i];
    if (feature.type == BUILDING) {
      draw_filled_feature(g, feature.points,
                          night_mode ? ezgl::WHITE : ezgl::GREY_75);
    }
  }

  // Step 8: Draw roads and one-way arrows, including curve points
  for (int i = 0; i < getNumStreetSegments(); i++) {
    StreetSegmentInfo segmentInfo = street_segment_info[i];
    vector<double> segfrom =
        getCoordinateMeters(intersection_positions[segmentInfo.from]);
    vector<double> segto =
        getCoordinateMeters(intersection_positions[segmentInfo.to]);
    OSMID wayID = segmentInfo.wayOSMID;
    string road_type = getClassificationForOSMID(wayID);

    // Default road thickness
    double road_thickness = 1.0;

    // Apply road thickness based on classification
    if (!road_type.empty() &&
        road_thicknesses.find(road_type) != road_thicknesses.end()) {
      road_thickness = road_thicknesses[road_type];
    }

    // Set the road color and thickness
    g->set_color(default_road_color);
    g->set_line_width(road_thickness);

    // Draw the road segment, accounting for curve points
    if (segmentInfo.numCurvePoints == 0) {
      // No curve points: draw a straight line
      g->draw_line({segfrom[0], segfrom[1]}, {segto[0], segto[1]});
    } else {
      // With curve points: draw a series of connected lines
      vector<double> prevPoint = segfrom;
      for (int j = 0; j < segmentInfo.numCurvePoints; j++) {
        vector<double> curvePoint =
            getCoordinateMeters(getStreetSegmentCurvePoint(i, j));
        g->draw_line({prevPoint[0], prevPoint[1]},
                     {curvePoint[0], curvePoint[1]});
        prevPoint = curvePoint;
      }
      // Connect the last curve point to the end point
      g->draw_line({prevPoint[0], prevPoint[1]}, {segto[0], segto[1]});
    }

    // Reset line width for other elements
    g->set_line_width(1.0);

    // Store street segment info for interactivity
    ClickableObject obj;
    obj.id = i;
    obj.type = ClickableObject::STREET_SEGMENT;
    obj.geometry = {{segfrom[0], segfrom[1]}, {segto[0], segto[1]}};
    clickableObjects.push_back(obj);

    // Draw one-way arrows if applicable
    if (segmentInfo.oneWay) {
      draw_one_way_arrows(g, segmentInfo, i);
    }
  }

  // Step 9: Draw intersections
  for (int i = 0; i < getNumIntersections(); i++) {
    vector<double> position = getCoordinateMeters(intersection_positions[i]);
    // Highlight the intersection if it matches the last clicked object
    if (lastClickedType == 1 && intersection_index == i && showLabel) {
      g->set_color(ezgl::YELLOW);
      g->fill_arc({position[0], position[1]}, 5, 0, 360);
    }
    if (std::find(highlighted_intersections.begin(),
                  highlighted_intersections.end(),
                  i) != highlighted_intersections.end()) {
      g->set_color(ezgl::DARK_GREEN);
      g->fill_arc({position[0], position[1]}, 10, 0, 360);
    }

    // Store intersection info for interactivity
    ClickableObject obj;
    obj.id = i;
    obj.type = ClickableObject::INTERSECTION;
    obj.center = {position[0], position[1]};
    clickableObjects.push_back(obj);
  }
  // Draw BikeLanes
  for (int i = 0; i < bike_lane_ways.size(); i++) {
    // Get the first point of the current bike lane segment
    vector<double> prev_point = getCoordinateMeters(
        getNodeCoords(getNodeByIndex(osm_id_to_node_id[bike_lane_ways[i][0]])));

    // Loop through the remaining points in the bike lane segment
    for (int x = 1; x < bike_lane_ways[i].size(); x++) {
      // Get the current point
      vector<double> current_point = getCoordinateMeters(getNodeCoords(
          getNodeByIndex(osm_id_to_node_id[bike_lane_ways[i][x]])));

      // Draw a line between the previous point and the current point
      g->set_color(ezgl::DARK_GREEN);
      g->set_line_width(2);
      g->draw_line({prev_point[0], prev_point[1]},
                   {current_point[0], current_point[1]});

      // Update the previous point to the current point
      prev_point = current_point;
    }
  }
  // Draw BikeLanes
  for (int i = 0; i < subway_osm_ids.size(); i++) {
    // Get the first point of the current bike lane segment
    vector<double> prev_point = getCoordinateMeters(
        getNodeCoords(getNodeByIndex(osm_id_to_node_id[subway_osm_ids[i][0]])));

    // Loop through the remaining points in the bike lane segment
    for (int x = 1; x < subway_osm_ids[i].size(); x++) {
      // Get the current point
      vector<double> current_point = getCoordinateMeters(getNodeCoords(
          getNodeByIndex(osm_id_to_node_id[subway_osm_ids[i][x]])));

      // Draw a line between the previous point and the current point
      g->set_color(ezgl::PURPLE);
      g->set_line_width(2);
      g->draw_line({prev_point[0], prev_point[1]},
                   {current_point[0], current_point[1]});

      // Update the previous point to the current point
      prev_point = current_point;
    }
  }

  // Step 10: Draw points of interest (POIs)
  for (int i = 0; i < getNumPointsOfInterest(); i++) {
    vector<double> position = getCoordinateMeters(getPOIPosition(i));
    g->set_color(ezgl::RED);
    g->fill_arc({position[0], position[1]}, 7, 0, 360);

    // Store POI info for interactivity
    ClickableObject obj;
    obj.id = i;
    obj.type = ClickableObject::POI;
    obj.center = {position[0], position[1]};
    clickableObjects.push_back(obj);
  }
  // Step 11: Draw labels if enabled
  if (showLabel) {
    draw_styled_label(g, currentLabel, labelPosition.first,
                      labelPosition.second);
  }
  if (showIntersectionLabels && find_route_intersections.size() == 2) {
    vector<double> pos1 = getCoordinateMeters(
        getIntersectionPosition(find_route_intersections[0]));
    vector<double> pos2 = getCoordinateMeters(
        getIntersectionPosition(find_route_intersections[1]));
    g->set_color(ezgl::PINK);
    g->fill_arc({pos1[0], pos1[1]}, 10, 0, 360);
    g->set_color(ezgl::PURPLE);
    g->fill_arc({pos2[0], pos2[1]}, 10, 0, 360);
    draw_styled_label(g, getIntersectionName(find_route_intersections[0]),
                      pos1[0], pos1[1]);
    draw_styled_label(g, getIntersectionName(find_route_intersections[1]),
                      pos2[0], pos2[1]);
  }
  // path

  if (find_route_intersections.size() == 2 && !find_route) {
    std::vector<StreetSegmentIdx> path = findPathBetweenIntersections(
        15.0, std::make_pair(find_route_intersections[0],
                             find_route_intersections[1]));

    if (!path.empty()) {
      g->set_color(ezgl::BLUE);
      g->set_line_width(3.0);

      // Map to store segments grouped by street ID
      std::map<StreetSegmentIdx, std::vector<StreetSegmentInfo>>
          street_segments;
      std::map<StreetSegmentIdx, double> street_distances;

      // Initialize previous_intersection_id
      int previous_intersection_id = find_route_intersections[0];
      StreetSegmentIdx previous_street_id = -1;
      std::string direction;

      // Group segments by street ID and calculate distances
      for (const auto& seg_idx : path) {
        StreetSegmentInfo seg_info = getStreetSegmentInfo(seg_idx);
        street_segments[seg_info.streetID].push_back(seg_info);
        double distance = 0.0;
        if (seg_info.numCurvePoints == 0) {
          int from_id = seg_info.from;
          int to_id = seg_info.to;
          std::vector<double> from_coord =
              getCoordinateMeters(intersection_positions[from_id]);
          std::vector<double> to_coord =
              getCoordinateMeters(intersection_positions[to_id]);
          distance = std::sqrt(std::pow(to_coord[0] - from_coord[0], 2) +
                               std::pow(to_coord[1] - from_coord[1], 2));
        } else {
          std::vector<double> prevPoint =
              getCoordinateMeters(intersection_positions[seg_info.from]);
          for (int j = 0; j < seg_info.numCurvePoints; ++j) {
            std::vector<double> point =
                getCoordinateMeters(getStreetSegmentCurvePoint(seg_idx, j));
            distance += std::sqrt(std::pow(point[0] - prevPoint[0], 2) +
                                  std::pow(point[1] - prevPoint[1], 2));
            prevPoint = point;
          }
          std::vector<double> to_coord =
              getCoordinateMeters(intersection_positions[seg_info.to]);
          distance += std::sqrt(std::pow(to_coord[0] - prevPoint[0], 2) +
                                std::pow(to_coord[1] - prevPoint[1], 2));
        }
        street_distances[seg_info.streetID] += distance;
      }

      // Track previous direction vector
      double prev_dir_x = 0.0;
      double prev_dir_y = 0.0;

      // Vectors to store turn labels and distance labels
      std::vector<std::tuple<std::string, double, double>> turn_labels;
      std::vector<std::tuple<std::string, double, double>> distance_labels;

      // Draw segments and handle turns
      for (size_t i = 0; i < path.size(); ++i) {
        StreetSegmentIdx seg_idx = path[i];
        StreetSegmentInfo seg_info = getStreetSegmentInfo(seg_idx);
        int current_from_id = seg_info.from;
        int current_to_id = seg_info.to;

        int start_id =
            (i == 0) ? find_route_intersections[0] : previous_intersection_id;
        int end_id =
            (start_id == current_from_id) ? current_to_id : current_from_id;
        std::vector<double> start_coord =
            getCoordinateMeters(intersection_positions[start_id]);
        std::vector<double> end_coord =
            getCoordinateMeters(intersection_positions[end_id]);

        // Compute current direction vector
        double current_dx = end_coord[0] - start_coord[0];
        double current_dy = end_coord[1] - start_coord[1];

        // Turn detection (if not first segment)
        if (i > 0) {
          double cross_product =
              (prev_dir_x * current_dy) - (prev_dir_y * current_dx);

          if (fabs(cross_product) < 1e-6) {
            direction = "straight";
          } else if (cross_product < 0) {
            direction = "right";
          } else {
            direction = "left";
          }

          // Check for street change
          if (previous_street_id != -1 &&
              previous_street_id != seg_info.streetID) {
            // Store turn label at turn point
            std::string turn_label = "Turn " + direction;
            double turn_x = start_coord[0] + 20;  // Offset to right
            double turn_y = start_coord[1];       // At turn intersection
            turn_labels.emplace_back(turn_label, turn_x, turn_y);
          }
        }

        // Update previous direction vector
        prev_dir_x = current_dx;
        prev_dir_y = current_dy;

        // Drawing and distance accumulation
        if (seg_info.numCurvePoints == 0) {
          drawArrowOnLine(g, start_coord, end_coord, 40.0);
          g->draw_line({start_coord[0], start_coord[1]},
                       {end_coord[0], end_coord[1]});
        } else {
          std::vector<std::vector<double>> curvePoints;
          for (int j = 0; j < seg_info.numCurvePoints; ++j) {
            curvePoints.push_back(
                getCoordinateMeters(getStreetSegmentCurvePoint(seg_idx, j)));
          }
          if (start_id == current_to_id) {
            std::reverse(curvePoints.begin(), curvePoints.end());
          }

          std::vector<double> prevPoint = start_coord;
          for (const auto& point : curvePoints) {
            drawArrowOnLine(g, prevPoint, point, 20.0);
            g->draw_line({prevPoint[0], prevPoint[1]}, {point[0], point[1]});
            prevPoint = point;
          }
          drawArrowOnLine(g, prevPoint, end_coord, 20.0);
          g->draw_line({prevPoint[0], prevPoint[1]},
                       {end_coord[0], end_coord[1]});
        }

        previous_street_id = seg_info.streetID;
        previous_intersection_id = end_id;
      }

      // Store distance labels at the midpoint of each street
      for (const auto& [street_id, segments] : street_segments) {
        double total_distance = street_distances[street_id];
        std::vector<double> start_coord =
            getCoordinateMeters(intersection_positions[segments.front().from]);
        std::vector<double> end_coord =
            getCoordinateMeters(intersection_positions[segments.back().to]);

        // Calculate midpoint
        double label_x = (start_coord[0] + end_coord[0]) / 2.0;
        double label_y = (start_coord[1] + end_coord[1]) / 2.0 + 30.0;

        // Get street name
        std::string street_name = getStreetName(street_id);

        // Format distance to two decimal places using sprintf
        char distance_str[20];
        sprintf(distance_str, "%.2f", total_distance);
        std::string formatted_distance(distance_str);

        // Store distance label with street name
        std::string distance_label =
            "Drive " + formatted_distance + "m along " + street_name;
        distance_labels.emplace_back(distance_label, label_x, label_y);
      }

      // Adjust label positions to avoid overlap
      for (auto& [label, x, y] : distance_labels) {
        double font_size = 12;
        double padding_x = 8;
        double padding_y = 4;
        double text_width = font_size * label.size() * 0.6;
        double text_height = font_size * 1.2;
        double label_width = text_width + 2 * padding_x;
        double label_height = text_height + 2 * padding_y;

        for (auto& [other_label, other_x, other_y] : distance_labels) {
          if (&label != &other_label) {
            double other_text_width = font_size * other_label.size() * 0.6;
            double other_text_height = font_size * 1.2;
            double other_label_width = other_text_width + 2 * padding_x;
            double other_label_height = other_text_height + 2 * padding_y;

            if (labelsOverlap(x, y, label_width, label_height, other_x, other_y,
                              other_label_width, other_label_height)) {
              // Move the label to avoid overlap
              y += 40;  // Move down by 40 pixels
            }
          }
        }
      }

      // Draw turn labels
      for (const auto& [label, x, y] : turn_labels) {
        draw_nice_label(g, label, x, y);
      }

      // Draw distance labels
      for (const auto& [label, x, y] : distance_labels) {
        draw_nice_label(g, label, x, y);
      }
    }
  }
}
/**
 * A callback function for the "Load Map" button.
 * Prompts the user to enter a city name, loads the corresponding map, and
 * updates the application's canvas.
 */
void load_button_click(GtkWidget* /*widget*/, ezgl::application* application) {
  // Close the current map/canvas before loading a new one
  closeMap();

  // Default map path template (points to Toronto by default)
  std::string maps_osm = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";

  // Prompt the user to enter the desired city name
  std::string city_name;
  std::cout << "Enter the city name (e.g., montreal-canada): ";
  std::getline(std::cin, city_name);

  // Replace "toronto_canada" in the default path with the user-provided city
  // name
  size_t pos = maps_osm.find("toronto_canada");
  if (pos != std::string::npos) {
    maps_osm.replace(
        pos, 14, city_name);  // Replace "toronto_canada" with the new city name
  } else {
    // If the default path does not contain "toronto_canada", log an error
    std::cerr << "Error: Default path does not contain 'toronto_canada'.\n";
  }

  // Attempt to load the updated map file
  bool load_success = loadMap(maps_osm);
  if (!load_success) {
    // If loading fails, attempt to load the default Toronto map as a fallback
    std::cerr << "Failed to load map '" << maps_osm << "'\n";
    std::cout << "Loading default Toronto map instead...\n";
    load_success =
        loadMap("/cad2/ece297s/public/maps/toronto_canada.streets.bin");

    // If even the fallback fails, log a critical error and exit
    if (!load_success) {
      std::cerr << "Critical Error: Could not load any map!\n";
      return;
    }
  }

  // Log successful map loading
  std::cout << "Successfully loaded map '" << maps_osm << "'\n";

  // Set up the coordinate system for the newly loaded map
  setCoordinateSystem();
  features_data.clear();
  features_data.shrink_to_fit();
  preprocessFeatures();

  // Convert the maximum latitude/longitude to meters to define the visible
  // world
  vector<double> endpoint = getCoordinateMeters(maximum);
  static ezgl::rectangle initial_world{{0, 0}, endpoint[0], endpoint[1]};

  // Log the top-right corner of the visible world for debugging purposes
  double x = initial_world.top_right().x;
  double y = initial_world.top_right().y;
  top_right = {x, y};

  // Update the visible world in the renderer
  application->get_renderer()->set_visible_world(initial_world);

  // Check if the "MainCanvas" is already added; if not, add it
  if (!application->get_canvas("MainCanvas")) {
    application->add_canvas("MainCanvas", draw_main_canvas, initial_world);
  } else {
    // If "MainCanvas" exists, update its visible world
    application->get_renderer()->set_visible_world(initial_world);
  }

  // Refresh the drawing to apply the new zoom level and map
  application->refresh_drawing();
}

/**
 * Converts a string to lowercase and removes leading/trailing spaces.
 * @param input The input string to process.
 * @return The processed string in lowercase without extra spaces.
 */
std::string toLowerCase(const std::string& input) {
  std::string result = input;

  // Convert all characters to lowercase
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  // Remove all whitespace characters
  result.erase(std::remove_if(result.begin(), result.end(),
                              [](unsigned char c) { return std::isspace(c); }),
               result.end());

  return result;
}

/**
 * Callback function for the search bar.
 * Allows the user to search for intersections or streets based on their input.
 */
std::vector<StreetIdx> getUniqueElements(const std::vector<StreetIdx>& input) {
  // Use an unordered_set to store unique elements
  std::unordered_set<StreetIdx> uniqueSet(input.begin(), input.end());

  // Convert the unordered_set back to a vector
  std::vector<StreetIdx> uniqueVector(uniqueSet.begin(), uniqueSet.end());

  return uniqueVector;
}
void search_bar_click(const gchar* text, ezgl::application* application) {
  // Convert the input text to a C++ string
  std::string input = text ? text : "";
  input = toLowerCase(input);  // Normalize the input

  // Check if the input contains a comma
  size_t comma_pos1 = input.find(',');
  if (find_route) {
    // Check for exactly one pipe character
    size_t pipe_pos = input.find('|');
    if (pipe_pos == std::string::npos || pipe_pos == 0 ||
        pipe_pos == input.length() - 1) {
      application->create_popup_message(
          "Error", "Invalid format. Use 'street1,street2 | street3,street4'");
      return;
    }

    // Split input into left and right parts
    std::string left_part = input.substr(0, pipe_pos);
    std::string right_part = input.substr(pipe_pos + 1);

    // Helper function to process a comma-separated street pair
    auto process_pair = [&](const std::string& pair_str)
        -> std::pair<bool, std::vector<IntersectionIdx>> {
      std::vector<IntersectionIdx> intersections;
      comma_pos1 = pair_str.find(',');
      if (comma_pos1 == std::string::npos) {
        return {false, intersections};
      }

      std::string street1 = pair_str.substr(0, comma_pos1);
      std::string street2 = pair_str.substr(comma_pos1 + 1);

      // Trim and lowercase
      auto trim_and_lower = [](std::string s) {
        s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s;
      };
      street1 = trim_and_lower(street1);
      street2 = trim_and_lower(street2);

      // Find all possible street IDs for both streets
      std::vector<StreetIdx> streets1 =
          getUniqueElements(findStreetIdsFromPartialStreetName(street1));
      std::vector<StreetIdx> streets2 =
          getUniqueElements(findStreetIdsFromPartialStreetName(street2));

      // Check if both streets have valid IDs
      if (streets1.empty() || streets2.empty()) {
        return {false, intersections};
      }

      // Generate all combinations and collect intersections
      std::vector<IntersectionIdx> temp_intersections;
      for (const auto& s1 : streets1) {
        for (const auto& s2 : streets2) {
          auto temp = findIntersectionsOfTwoStreets({s1, s2});
          temp_intersections.insert(temp_intersections.end(), temp.begin(),
                                    temp.end());
        }
      }

      // Deduplicate intersections
      std::unordered_set<IntersectionIdx> unique_intersections(
          temp_intersections.begin(), temp_intersections.end());
      intersections.assign(unique_intersections.begin(),
                           unique_intersections.end());

      return {!intersections.empty(), intersections};
    };

    // Process each part
    auto left_result = process_pair(left_part);
    auto right_result = process_pair(right_part);

    std::string error_message;
    std::vector<IntersectionIdx> all_intersections;

    // Check left part
    if (left_result.first) {
      all_intersections.insert(all_intersections.end(),
                               left_result.second.begin(),
                               left_result.second.end());
    } else {
      error_message += "Error: First pair invalid or no intersections found. ";
    }

    // Check right part
    if (right_result.first) {
      all_intersections.insert(all_intersections.end(),
                               right_result.second.begin(),
                               right_result.second.end());
    } else {
      error_message += "Error: Second pair invalid or no intersections found. ";
    }

    // Deduplicate final results
    std::unordered_set<IntersectionIdx> final_unique(all_intersections.begin(),
                                                     all_intersections.end());
    all_intersections.assign(final_unique.begin(), final_unique.end());

    // Handle errors
    if (!error_message.empty()) {
      application->create_popup_message("Input Errors", error_message.c_str());
      return;
    }

    // No intersections found in valid pairs
    if (all_intersections.empty()) {
      application->create_popup_message(
          "No Intersections", "No intersections found between valid pairs.");
      return;
    }

    // Highlight intersections and show results
    highlight = true;

    // Save the first intersections from left and right into the global array
    find_route_intersections.clear();
    find_route_intersections.push_back(
        left_result.second[0]);  // First intersection from left pair
    find_route_intersections.push_back(
        right_result.second[0]);  // First intersection from right pair

    // Prepare message with intersection names
    std::string message = "";
    // Prepare message with intersection names
    if (find_route_intersections.size() >= 2) {
      std::string left_name = getIntersectionName(find_route_intersections[0]);
      std::string right_name = getIntersectionName(find_route_intersections[1]);
      std::vector<StreetSegmentIdx> path = findPathBetweenIntersections(
          15.0, std::make_pair(find_route_intersections[0],
                               find_route_intersections[1]));
      message = "Showing route between:\n";
      message += "- " + left_name + " (Left side)\n";
      message += "- " + right_name + " (Right side)\n\n";
      message +=
          "Travel time is " +
          (std::to_string(computePathTravelTime(15, path) / 60) + " minutes") +
          "\n\n";
      message += "All intersections found:\n";
      for (const auto& id : all_intersections) {
        message += "- " + getIntersectionName(id) + "\n";
      }
      application->create_popup_message("Route Details", message.c_str());
    } else {
      // Fallback to original message if intersections aren't properly saved
      message = "Intersections Found:\n";
      for (const auto& id : all_intersections) {
        message += "- " + getIntersectionName(id) + "\n";
      }
      application->create_popup_message("Intersections Found", message.c_str());
    }
    find_route = false;
    application->refresh_drawing();
    return;  // Exit after processing find_route
  }

  // Existing comma case (only executes if find_route is false)
  else if (input.find(',') != std::string::npos) {
    // Input contains a comma: Search for intersections
    comma_pos1 = input.find(',');
    std::string street1 = input.substr(0, comma_pos1);
    std::string street2 = input.substr(comma_pos1 + 1);

    // Trim whitespace and convert to lowercase
    street1 = toLowerCase(street1);
    street2 = toLowerCase(street2);
    street1.erase(std::remove_if(street1.begin(), street1.end(), ::isspace),
                  street1.end());
    street2.erase(std::remove_if(street2.begin(), street2.end(), ::isspace),
                  street2.end());

    // Find all possible street IDs for both streets using partial matches
    std::vector<StreetIdx> streets1 =
        getUniqueElements(findStreetIdsFromPartialStreetName(street1));
    std::vector<StreetIdx> streets2 =
        getUniqueElements(findStreetIdsFromPartialStreetName(street2));

    // Handle no matches for either street
    if (streets1.empty() || streets2.empty()) {
      std::string errorMessage = "Error: ";
      if (streets1.empty()) {
        errorMessage += "No streets found for '" + street1 + "'. ";
      }
      if (streets2.empty()) {
        errorMessage += "No streets found for '" + street2 + "'.";
      }
      showIntersectionLabels = true;
      application->create_popup_message("Error", errorMessage.c_str());
      return;
    }

    // Collect all intersections between all combinations of streets
    std::vector<IntersectionIdx> temp_intersections;
    for (const auto& s1 : streets1) {
      for (const auto& s2 : streets2) {
        auto intersections = findIntersectionsOfTwoStreets({s1, s2});
        temp_intersections.insert(temp_intersections.end(),
                                  intersections.begin(), intersections.end());
      }
    }

    // Deduplicate intersections
    std::unordered_set<IntersectionIdx> unique_intersections(
        temp_intersections.begin(), temp_intersections.end());

    // Prepare results
    highlighted_intersections.assign(unique_intersections.begin(),
                                     unique_intersections.end());
    highlight = true;
    application->refresh_drawing();

    // Display results
    if (highlighted_intersections.empty()) {
      application->create_popup_message(
          "No Intersections", "No intersections found between the streets.");
    } else {
      std::string message = "Intersections Found:\n";
      for (const auto& id : highlighted_intersections) {
        message += "- " + getIntersectionName(id) + "\n";
      }
      application->create_popup_message("Intersections Found", message.c_str());
    }
  } else {
    // Input does not contain a comma: Search for streets
    std::vector<StreetIdx> streets_found =
        getUniqueElements(findStreetIdsFromPartialStreetName(input));

    if (streets_found.empty()) {
      application->create_popup_message(
          "Error", ("No matches found for street name: " + input).c_str());
    } else {
      std::string popup_message = "Streets Found:\n";
      cout << getNumStreets() << endl;
      for (auto id : streets_found) {
        cout << id << endl;
        popup_message += "- " + getStreetName(id) + "\n";
      }
      application->create_popup_message("Search Results",
                                        popup_message.c_str());
    }
  }
}

void load_button_click_help(GtkWidget* widget, ezgl::application* application) {
  // Create a message dialog
  const gchar* message =
      "Hello! \n Finding a path between two intersections on our map requires "
      "\n either clicking on two intersections on the map or manually typing "
      "their names";

  application->get_main_canvas_id();
  GtkWidget* dialog = gtk_message_dialog_new(
      GTK_WINDOW(gtk_widget_get_toplevel(widget)), GTK_DIALOG_MODAL,
      GTK_MESSAGE_INFO,  // Message type (INFO, WARNING, ERROR, etc.)
      GTK_BUTTONS_NONE,  // Buttons (OK, CANCEL, etc.)
      "%s", message      // Pass the formatted message to the dialog
  );
  gtk_dialog_add_button(GTK_DIALOG(dialog), "Next", GTK_RESPONSE_OK);
  // Show the dialog and wait for the user to acknowledge it
  gtk_dialog_run(GTK_DIALOG(dialog));

  // Destroy the dialog after the user clicks OK
  gtk_widget_destroy(dialog);

  // Create the second dialog
  GtkWidget* dialog2 = gtk_message_dialog_new(
      GTK_WINDOW(gtk_widget_get_toplevel(widget)), GTK_DIALOG_MODAL,
      GTK_MESSAGE_INFO, GTK_BUTTONS_NONE,
      "Once clicked, the intersections will be highlighted with a yellow dot "
      "to indicate it has been selected.\n You need to select two for the "
      "program to continue. \n Click next to see an example.");
  gtk_dialog_add_button(GTK_DIALOG(dialog2), "Next", GTK_RESPONSE_OK);

  // Show the second dialog
  gtk_dialog_run(GTK_DIALOG(dialog2));
  gtk_widget_destroy(dialog2);

  GtkWidget* image_dialog = gtk_dialog_new_with_buttons(
      "Zoom Preview",  // Dialog title
      GTK_WINDOW(gtk_widget_get_ancestor(
          widget, GTK_TYPE_WINDOW)),  // Ensure valid parent window
      GTK_DIALOG_MODAL, "Next", GTK_RESPONSE_OK, NULL);

  // Get the dialog's content area
  GtkWidget* content_area =
      gtk_dialog_get_content_area(GTK_DIALOG(image_dialog));

  // Load and create an image widget
  GtkWidget* image = gtk_image_new_from_file("mapfinal.png");

  // Check if image loaded successfully
  if (!image) {
    g_printerr("Error: Could not load image 'mapexample.png, sorry!'\n");
    return;
  }

  // Add the image to the dialog
  gtk_container_add(GTK_CONTAINER(content_area), image);

  // Show everything in the dialog
  gtk_widget_show_all(image_dialog);

  // Run the dialog and wait for user response
  gtk_dialog_run(GTK_DIALOG(image_dialog));
  gtk_widget_destroy(image_dialog);

  // Create the second dialog
  GtkWidget* dialog3 = gtk_message_dialog_new(
      GTK_WINDOW(gtk_widget_get_toplevel(widget)), GTK_DIALOG_MODAL,
      GTK_MESSAGE_INFO, GTK_BUTTONS_NONE,
      "Alternatively, use the search ? to enter two intersections in the "
      "following format: \n \n Street1,Street2|Street3,Street4");
  gtk_dialog_add_button(GTK_DIALOG(dialog3), "Finish", GTK_RESPONSE_OK);

  // Show the second dialog
  gtk_dialog_run(GTK_DIALOG(dialog3));
  gtk_widget_destroy(dialog3);
}

/**
 * Function to handle mouse press event
 * The current mouse position in the main canvas' world coordinate system is
 * returned. A pointer to the application and the entire GDK event are also
 * provided.
 */
void act_on_mouse_press(ezgl::application* application, GdkEventButton* event,
                        double x, double y) {
  // Update the status bar message to indicate a mouse click
  application->update_message("Mouse Clicked");

  // Determine the object clicked based on the mouse coordinates (x, y)
  auto result = clicked(x, y);

  // Check if a valid object was clicked (result.first != -1 indicates a valid
  // object)
  if (result.first != -1) {
    std::cout << "Clicked Object - Type: " << result.second
              << ", ID: " << result.first << std::endl;

    // If the same object was clicked again, clear the label
    if (result.first == lastClickedID && result.second == lastClickedType) {
      showLabel = false;    // Disable label display
      lastClickedID = -1;   // Reset the last clicked object's ID
      lastClickedType = 0;  // Reset the type of the last clicked object
      std::cout << "Label removed." << std::endl;
    }

    else {
      // Update the label with information about the newly clicked object
      if (result.second == 0) {  // STREET_SEGMENT
        currentLabel =
            getStreetName(street_segment_info[result.first].streetID);
        labelPosition = {x, y};  // Set label position to the click coordinates
      } else if (result.second == 1) {  // INTERSECTION
        intersection_index =
            result.first;  // Store the index of the clicked intersection
        currentLabel =
            getIntersectionName(result.first);  // Get the intersection name
        vector<double> location =
            getCoordinateMeters(intersection_positions[result.first]);
        labelPosition = {location[0],
                         location[1]};  // Set label position to the
                                        // intersection's coordinates
        if (find_route) {
          if (find_route_intersections.size() < 2) {
            find_route_intersections.push_back(result.first);
          }
          if (find_route_intersections.size() == 2) {
            location = getCoordinateMeters(
                intersection_positions[find_route_intersections[0]]);
            showIntersectionLabels = true;
            find_route = false;
            if (find_route_intersections.size() == 2 && !find_route) {
              std::vector<StreetSegmentIdx> path = findPathBetweenIntersections(
                  15.0, std::make_pair(find_route_intersections[0],
                                       find_route_intersections[1]));
              application->create_popup_message(
                  "Time to Destination",
                  (std::to_string(computePathTravelTime(15, path) / 60) +
                   " minutes")
                      .c_str());
            }
          }
        }
      } else if (result.second == 2) {            // POI
        currentLabel = getPOIName(result.first);  // Get the POI name
        vector<double> location =
            getCoordinateMeters(getPOIPosition(result.first));
        labelPosition = {
            location[0],
            location[1]};  // Set label position to the POI's coordinates
      }

      // Update the last clicked object's ID and type
      lastClickedID = result.first;
      lastClickedType = result.second;
      showLabel = true;  // Enable label display
      std::cout << "Label updated." << std::endl;
    }
  } else {
    // No valid object was clicked within the 10-meter radius
    std::cout << "No object clicked within the 10-meter radius." << std::endl;
    showLabel = false;  // Clear the label
  }

  // Log the mouse click coordinates

  // Refresh the canvas to reflect any changes (e.g., label updates)
  application->refresh_drawing();

  // Check for modifier keys (Ctrl, Shift) and log their state
  if ((event->state & GDK_CONTROL_MASK) && (event->state & GDK_SHIFT_MASK)) {
    std::cout << "with control and shift pressed ";
  } else if (event->state & GDK_CONTROL_MASK) {
    std::cout << "with control pressed ";
  } else if (event->state & GDK_SHIFT_MASK) {
    std::cout << "with shift pressed ";
  }

  std::cout << std::endl;
}
/**
 * Function to handle mouse move event
 * The current mouse position in the main canvas' world coordinate system is
 * returned. A pointer to the application and the entire GDK event are also
 * provided.
 */
void act_on_mouse_move(ezgl::application* /*application*/,
                       GdkEventButton* /*event*/, double x, double y) {
  // Log the current mouse position
  std::cout << "Mouse move at coordinates (" << x << "," << y << ") "
            << std::endl;
}

/**
 * Function to handle keyboard press event
 * The name of the key pressed is returned (e.g., 0-9, a-z, A-Z, Up, Down, Left,
 * Right, Shift_R, Control_L, space, Tab, ...). A pointer to the application and
 * the entire GDK event are also provided.
 */
void act_on_key_press(ezgl::application* application, GdkEventKey* /*event*/,
                      char* key_name) {
  // Update the status bar message to indicate a key press
  application->update_message("Key Pressed");

  // Log the name of the key that was pressed
  std::cout << key_name << " key is pressed" << std::endl;
}

/**
 * Function to set up the coordinate system for the map
 * Determines the bounding box (minimum and maximum latitude/longitude) and
 * calculates the average latitude.
 */
void setCoordinateSystem() {
  // Initialize maximum and minimum coordinates to extreme values
  maximum = LatLon(-200, -200);  // Smallest possible latitude/longitude
  mininimum = LatLon(200, 200);  // Largest possible latitude/longitude

  // Clear the previous mapping
  osm_id_to_node_id.clear();

  // Iterate through all nodes to find the actual bounding box and populate the
  // mapping
  for (int i = 0; i < getNumberOfNodes(); i++) {
    const OSMNode* node = getNodeByIndex(i);  // Get the current node
    if (!node) continue;                      // Skip invalid nodes

    const LatLon temp = node->coords();  // Get the node's coordinates
    OSMID osm_id = node->id();           // Get the OSM ID of the node

    // Update the maximum and minimum latitude/longitude values
    maximum = LatLon(max(temp.latitude(), maximum.latitude()),
                     max(temp.longitude(), maximum.longitude()));
    mininimum = LatLon(min(temp.latitude(), mininimum.latitude()),
                       min(temp.longitude(), mininimum.longitude()));

    // Populate the mapping from OSM ID to node ID
    osm_id_to_node_id[osm_id] = i;
  }

  // Calculate the average latitude for coordinate conversion
  lat_avg = (maximum.latitude() + mininimum.latitude()) / 2.0;

  // Log the bounding box and average latitude for debugging purposes
  // cout << "Max: " << maximum.latitude() << ", " << maximum.longitude() <<
  // endl; cout << "Min: " << mininimum.latitude() << ", " <<
  // mininimum.longitude() << endl; cout << "Lat Avg: " << lat_avg << endl;

  // Debugging: Print the size of the mapping
  // cout << "Number of nodes mapped: " << osm_id_to_node_id.size() << endl;
}

/**
 * Function to initialize and draw the map
 * Sets up the EZGL graphics window, defines the initial coordinate system, and
 * starts the application loop.
 */
void drawMap() {
  // Configure settings for the EZGL application
  ezgl::application::settings settings;

  // Path to the UI layout file (main.ui) containing the XML description of the
  // interface
  settings.main_ui_resource = "libstreetmap/resources/main.ui";

  // Identifier for the main application window (defined in main.ui)
  settings.window_identifier = "MainWindow";
  void load_button_click_help(GtkWidget * widget);

  // Create the EZGL application instance with the specified settings
  ezgl::application application(settings);

  // Set up the coordinate system for the map
  setCoordinateSystem();
  preprocessFeatures();
  // Convert the maximum latitude/longitude to meters to define the visible
  // world
  vector<double> endpoint = getCoordinateMeters(maximum);
  static ezgl::rectangle initial_world{{0, 0}, endpoint[0], endpoint[1]};

  // Log the top-right corner of the visible world for debugging purposes
  double x = initial_world.top_right().x;
  double y = initial_world.top_right().y;
  top_right = {x, y};
  // std::cout << "Top right (" << x << "," << y << ") " << std::endl;

  // Add the main canvas to the application and associate it with the drawing
  // function
  application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

  // Start the application loop, passing callback functions for initialization
  // and event handling
  application.run(initial_setup, act_on_mouse_press, act_on_mouse_move,
                  act_on_key_press);
}
