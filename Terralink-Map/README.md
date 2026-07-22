#  TerraLink: Move With Nature

# Overview
**m1.cpp** is about geographic queries by graph-based GIS mapping system
**m2.cpp** is about planning optimal delivery routine for couriers.
  
**TerraLink** is an eco-friendly navigation system that integrates **nature trails** and **urban transit** into one seamless digital map.  
It is designed to help users explore cities sustainably by combining **multipurpose trails**, **subway lines**, and **eco-routing algorithms** to recommend the best paths for walking, cycling, or public transport.

TerraLink bridges the gap between **traditional city navigation tools** (like Google Maps) and **trail information platforms** (like AllTrails) by providing both accurate wayfinding and environmentally conscious route planning.



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


## UI
<img width="922" height="652" alt="image" src="https://github.com/user-attachments/assets/b1ad0d44-cff6-4e3f-b4bc-b9a8e0afad16" />

## How to Use TerraLink

### Load Map: Clik the button then enter the country name in format like: montreal-canada

### Find Route button: Clik two intersections on the map or type their name manually
<img width="636" height="712" alt="image (2)" src="https://github.com/user-attachments/assets/bdeba334-8161-4b4a-bb83-eaba95333f45" />

### Find Rounte button: Wayfinding & optimal delivery rounte
<img width="1178" height="569" alt="Screenshot 2026-07-22 at 5 54 59 PM" src="https://github.com/user-attachments/assets/069d291b-1e2d-4965-b5ee-4684cdde28b0" />

### Search 🔍: Enter two intersections in format: **Street1,Street2|Street3,Stree4**, 
then click proceed.
This can also find out the rounte you want!
    
### If need help: Pressing Help button will pop out the insturtcion of using map
<img width="877" height="627" alt="image (1)" src="https://github.com/user-attachments/assets/62949954-e996-4b9b-9dee-3e4bbc0f302b" />


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



