#  TerraLink: Move With Nature

## 🧭 Overview
**TerraLink** is an eco-friendly navigation system that integrates **nature trails** and **urban transit** into one seamless digital map.  
It is designed to help users explore cities sustainably by combining **multipurpose trails**, **subway lines**, and **eco-routing algorithms** to recommend the best paths for walking, cycling, or public transport.

TerraLink bridges the gap between **traditional city navigation tools** (like Google Maps) and **trail information platforms** (like AllTrails) by providing both accurate wayfinding and environmentally conscious route planning.

---

## Design Principles

| Principle | Description |
|------------|-------------|
| **Usability** | Enables users to complete navigation tasks accurately, quickly, and comfortably. |
| **Responsiveness** | Reacts to user interactions instantly; optimized algorithms support smooth real-time updates. |
| **Integration** | Unifies multiple data sources — trails, public transport, and city roads — into a cohesive interface. |
| **Sustainability** | Encourages environmentally conscious travel by promoting green routes and active commuting. |

---

## Map Features
- **Integrated Trail and Subway Visualization**  
  Displays city subway lines alongside multipurpose trails to show **path continuity** and promote mixed-mode travel.  

- **Filter System**  
  Allows users to prioritize eco-friendly, scenic, or accessible routes.

- **Smart Wayfinding**  
  Uses an **efficiency heuristic** to balance route speed with environmental relevance — ensuring travel time remains under 3× that of the fastest route.

- **Real-Time Responsiveness**  
  Route generation and map loading occur in **under four seconds**, even for large cities like Toronto, NYC, and Boston.

- **Visual Accessibility**  
  Designed following accessibility standards for map color, contrast, and layout, ensuring readability and clarity for all users.

---

## How to Use TerraLink

1. **Open the TerraLink Map Interface**  
   Access the web or mobile interface.

2. **Select Your Start and Destination Points**  
   Choose any location within supported cities (e.g., Toronto, NYC, Boston).

3. **Apply Filters**  
   - Choose your travel mode (trail, transit, or hybrid).  
   - Enable eco-priority to view the most sustainable routes.  

4. **View Suggested Routes**  
   TerraLink will compute and display optimal paths using Dijkstra’s and A* algorithms.  
   Each route includes:
   - Distance and estimated travel time  
   - Trail and subway connectivity  
   - Eco-efficiency score  

5. **Navigate and Explore**  
   Follow the guided route to enjoy a smoother, greener commute — **moving with nature**.


## Algorithms Used

### **Dijkstra’s Algorithm**
- Used for calculating the **shortest and most efficient paths**.
- Employs a **priority queue (min-heap)** to process nodes with the lowest travel cost first.  
- Caches all computed paths in a **Distance Cache** for optimized lookup and quick route recalculations.

### **A* (A-Star) Algorithm**
- Enhances Dijkstra’s approach by using a **heuristic function** for faster performance.
- The heuristic is based on the **Euclidean distance** divided by the **maximum speed (maxSpeedMPS)**.
- Enables **real-time eco-routing**, balancing travel distance and energy efficiency.

Together, these algorithms ensure that TerraLink provides **rapid, accurate, and adaptive routing**, even across large urban datasets.

---

---

**Author(s):** [Your Name / Team Name]  
**Project:** TerraLink – Eco-Friendly Urban Navigation  
**Year:** 2025
