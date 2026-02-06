# 🔍 Search Algorithms Comparison: DFS vs BFS vs Dijkstra vs A*

## Quick Summary Table

| Feature | DFS | BFS | Dijkstra | A* |
|---------|-----|-----|----------|-----|
| **Data Structure** | Stack (LIFO) | Queue (FIFO) | Priority Queue (by g) | Priority Queue (by f=g+h) |
| **Exploration Pattern** | Deep first, then backtrack | Level by level (waves) | Waves by cost (like BFS) | Guided toward goal |
| **Guarantees Shortest Path?** | ❌ No | ✅ Yes (unweighted) | ✅ Yes (weighted too) | ✅ Yes |
| **Uses Heuristic?** | ❌ No (Uninformed) | ❌ No (Uninformed) | ❌ No (Uninformed) | ✅ Yes (Informed) |
| **Memory Usage** | Low (current path only) | High (all nodes at level) | Medium-High | Medium-High |
| **Time Complexity** | O(V + E) | O(V + E) | O(E log V) | O(E log V) |
| **Best For** | Checking if path exists | Shortest path (unweighted) | Shortest path (weighted) | Efficient shortest path |

---

## 1. DFS (Depth-First Search) 🔮

### How It Works
DFS explores as **deep as possible** along each branch before backtracking. Think of it like exploring a maze by always taking the first available turn until you hit a dead end.

```
Algorithm:
1. Start at the beginning
2. Go as deep as possible in one direction
3. When stuck, backtrack and try another path
4. Repeat until goal is found or all paths explored
```

### Data Structure: STACK (Last In, First Out)
```
Push A → Stack: [A]
Push B → Stack: [A, B]
Push C → Stack: [A, B, C]
Pop    → Returns C, Stack: [A, B]  ← Most recent first!
```

### Visual Example
```
Start: S, Goal: G

    S ─── 1 ─── 2 ─── 3
    │
    4 ─── 5 ─── 6
    │
    7 ─── 8 ─── G

DFS Exploration Order: S → 1 → 2 → 3 (dead end!) → backtrack → 4 → 5 → 6 (dead end!) → backtrack → 7 → 8 → G

Path found: S → 4 → 7 → 8 → G (length 4)
But there might be a shorter path we didn't check!
```

### Pros ✅
- **Memory efficient**: Only stores the current path
- **Fast to implement**: Simple recursive or stack-based
- **Good for**: Checking if ANY path exists

### Cons ❌
- **Not optimal**: May find a long, winding path
- **Can get stuck**: In deep branches that lead nowhere
- **Order dependent**: Results change based on neighbor order

### Python Implementation Key Part
```python
stack = [(start, [start])]  # (position, path_so_far)
visited = set()

while stack:
    current, path = stack.pop()  # LIFO - takes from END
    
    if current == goal:
        return path  # Found it! (but maybe not shortest)
    
    for neighbor in get_neighbors(current):
        if neighbor not in visited:
            visited.add(neighbor)
            stack.append((neighbor, path + [neighbor]))
```

---

## 2. BFS (Breadth-First Search) 🌊

### How It Works
BFS explores **all neighbors at the current depth** before moving deeper. Think of it like ripples spreading out from a stone dropped in water - expanding in all directions equally.

```
Algorithm:
1. Start at the beginning
2. Visit ALL cells at distance 1
3. Then visit ALL cells at distance 2
4. Continue until goal is found
```

### Data Structure: QUEUE (First In, First Out)
```
Enqueue A → Queue: [A]
Enqueue B → Queue: [A, B]
Enqueue C → Queue: [A, B, C]
Dequeue   → Returns A, Queue: [B, C]  ← Oldest first!
```

### Visual Example
```
Start: S, Goal: G

         1
        /
    S ─── 2 ─── 5 ─── G
        \     /
         3 ─ 4

BFS Exploration Order:
  Distance 0: S
  Distance 1: 1, 2, 3
  Distance 2: 5, 4
  Distance 3: G

Path found: S → 2 → 5 → G (length 3) ← SHORTEST!
```

### Why BFS Guarantees Shortest Path 🎯
```
Key insight: BFS explores in "waves" of increasing distance.

Wave 0: Start (distance 0)
Wave 1: All cells 1 step away
Wave 2: All cells 2 steps away
...

The FIRST time we reach the goal is guaranteed to be 
via the shortest path, because we've already checked 
ALL shorter paths!
```

### Pros ✅
- **Optimal**: Always finds the shortest path
- **Complete**: Will find a solution if one exists
- **Predictable**: Same result regardless of neighbor order

### Cons ❌
- **Memory hungry**: Stores all nodes at current depth
- **Slower start**: Must explore everything at each level
- **No guidance**: Explores equally in all directions

### Python Implementation Key Part
```python
from collections import deque

queue = deque([start])  # Use deque for O(1) popleft
visited = {start}
came_from = {}

while queue:
    current = queue.popleft()  # FIFO - takes from START
    
    if current == goal:
        return reconstruct_path(came_from, goal)  # Shortest!
    
    for neighbor in get_neighbors(current):
        if neighbor not in visited:
            visited.add(neighbor)
            came_from[neighbor] = current
            queue.append(neighbor)
```

---

## 3. A* (A-Star Search) ⭐

### How It Works
A* combines the **best of both worlds**: it guarantees the shortest path like BFS, but uses a **heuristic** to guide the search toward the goal, making it much more efficient.

```
Algorithm:
1. Evaluate each cell with: f(n) = g(n) + h(n)
   - g(n) = actual cost from start to current cell
   - h(n) = estimated cost from current cell to goal
2. Always explore the cell with lowest f(n) first
3. This "pulls" the search toward the goal
```

### The Magic Formula: f(n) = g(n) + h(n)

```
f(n) = Total estimated cost of path through n
g(n) = Known cost from Start to n (actual steps taken)
h(n) = Heuristic estimate from n to Goal

Example:
  Start ----[5 steps]---- Current ----[???]---- Goal
                            │
                        g(n) = 5
                        h(n) = estimated 3 (Manhattan distance)
                        f(n) = 5 + 3 = 8
```

### Manhattan Distance Heuristic
For a grid where you can only move up/down/left/right:

```
Manhattan Distance = |x1 - x2| + |y1 - y2|

Example:
  From (1, 1) to (4, 5):
  |1-4| + |1-5| = 3 + 4 = 7

  This is the MINIMUM possible steps needed
  (if there were no walls)
```

### Why Manhattan Distance Works 🎯
```
ADMISSIBLE: Never overestimates the true cost
            (walls can only make the path longer)

This guarantee means A* will always find the optimal path!
```

### Visual Example
```
Start: S, Goal: G

    S ─── A ─── B ─── C
    │     │           │
    D ─── E ─── F ─── G

Evaluating from S:
  A: g=1, h=4, f=5
  D: g=1, h=3, f=4  ← Lower f, explore first!

A* tends to explore cells that are:
  1. Close to the start (low g)
  2. Close to the goal (low h)

Result: Explores FEWER cells than BFS while still finding shortest path!
```

### Pros ✅
- **Optimal**: Guarantees shortest path (with admissible heuristic)
- **Efficient**: Explores fewer cells than BFS
- **Smart**: Uses domain knowledge to guide search

### Cons ❌
- **More complex**: Requires good heuristic function
- **Memory**: Still needs to track open/closed sets
- **Heuristic dependent**: Bad heuristic = bad performance

### Python Implementation Key Part
```python
import heapq

def manhattan(pos, goal):
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

# Priority queue: (f_score, counter, position)
open_set = [(manhattan(start, goal), 0, start)]
g_score = {start: 0}
came_from = {}

while open_set:
    _, _, current = heapq.heappop(open_set)  # Lowest f first!
    
    if current == goal:
        return reconstruct_path(came_from, goal)
    
    for neighbor in get_neighbors(current):
        tentative_g = g_score[current] + 1
        
        if tentative_g < g_score.get(neighbor, float('inf')):
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + manhattan(neighbor, goal)
            heapq.heappush(open_set, (f_score, counter, neighbor))
```

---

## 4. Dijkstra's Algorithm 🛤️

### How It Works
Dijkstra's algorithm finds the shortest path by always expanding the node with the **lowest known cost g(n)**. It is the foundation that A* builds upon — effectively A* without the heuristic.

```
Algorithm:
1. Assign cost 0 to start, infinity to all others
2. Always explore the unvisited node with lowest cost
3. Update neighbor costs: g(neighbor) = g(current) + edge_cost
4. Repeat until goal is reached
```

### Key Formula: f(n) = g(n)
```
No heuristic! Dijkstra only uses actual measured cost.

   g(n) = actual cost from Start to n
   h(n) = 0   (no estimate, no guidance)
   f(n) = g(n) + 0 = g(n)

This makes Dijkstra UNINFORMED — it doesn't know where the goal is.
It expands outward in waves of increasing cost, like BFS.
```

### The Algorithm Family
```
BFS       ⊆  Dijkstra  ⊆  A*

• BFS = Dijkstra with all edge costs = 1
  (a FIFO queue naturally orders by cost when costs are equal)

• Dijkstra = A* with h(n) = 0
  (no heuristic to guide the search)

• A* = Dijkstra + heuristic
  (adds h(n) to focus exploration toward the goal)
```

### On Unweighted vs Weighted Graphs
```
UNWEIGHTED GRID (all costs = 1):
  Dijkstra ≈ BFS  → same result, same exploration pattern
  BFS is simpler and faster (no priority queue overhead)

WEIGHTED GRID (different costs):
  BFS FAILS → it counts hops, not total cost
  Dijkstra WORKS → it tracks actual cost to each node

Example with weights:
  S --[1]--> A --[1]--> G      cost = 2
  S --[5]----> G               cost = 5
  
  BFS: finds S→G (1 hop) ← WRONG, not cheapest!
  Dijkstra: finds S→A→G (cost 2) ← CORRECT!
```

### Visual Example
```
Start: S, Goal: G (unweighted grid)

    S ─── 1 ─── 2 ─── 3
    │     │           │
    4 ─── 5 ─── 6 ─── G

Dijkstra exploration (by increasing g):
  g=0: S
  g=1: 1, 4         (all cost-1 neighbors)
  g=2: 2, 5         (all cost-2 neighbors)
  g=3: 3, 6
  g=4: G

Identical to BFS because all edges cost 1!
The priority queue just happens to process in FIFO order.
```

### Pros ✅
- **Optimal**: Guarantees shortest path for ANY non-negative weights
- **Complete**: Will find a solution if one exists
- **General**: Works for weighted and unweighted graphs
- **Foundation**: Understanding Dijkstra makes A* easy to learn

### Cons ❌
- **Uninformed**: No knowledge of goal location (explores all directions)
- **Slower than A***: Explores more cells because it lacks heuristic guidance
- **Overkill for unweighted**: BFS is simpler and equally correct
- **Priority queue overhead**: Slower than BFS per node on unweighted graphs

### Python Implementation Key Part
```python
import heapq

# Priority queue: (g_score, counter, position)
open_set = [(0, 0, start)]   # g=0 for start
g_score = {start: 0}
came_from = {}

while open_set:
    current_g, _, current = heapq.heappop(open_set)  # Lowest g first!
    
    if current == goal:
        return reconstruct_path(came_from, goal)
    
    for neighbor in get_neighbors(current):
        tentative_g = g_score[current] + 1  # +1 for unweighted
        
        if tentative_g < g_score.get(neighbor, float('inf')):
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            heapq.heappush(open_set, (tentative_g, counter, neighbor))
```

### Spot the Difference: Dijkstra vs A*
```python
# Dijkstra: priority = g(n) only
heapq.heappush(open_set, (tentative_g, counter, neighbor))

# A*: priority = g(n) + h(n)
f_score = tentative_g + manhattan(neighbor, goal)
heapq.heappush(open_set, (f_score, counter, neighbor))

# That single "+manhattan()" is the ENTIRE difference!
```

---

## 5. Side-by-Side Comparison 📊

### Exploration Patterns

```
Same maze, different algorithms:

MAZE:
##########
#S.......#
#.#####..#
#.....#..#
#.###.#..#
#...#....#
#.#.####.#
#.#......#
#.#....G.#
##########

DFS Exploration (# = wall, * = visited):
##########
#S*......#        Explores deep into one
#*#####..#        direction first, might
#***..#..#        find a winding path
#*###.#..#
#*..#....#
#*#.####.#
#*#......#
#*#****G.#
##########
Visited: 15 cells, Path length: 15


BFS Exploration:
##########
#S*******#        Expands in waves from
#*#####**#        start, guaranteed to
#*****#**#        find shortest path
#*###*#**#
#***#****#
#*#*####*#
#*#******#
#*#****G*#
##########
Visited: 45 cells, Path length: 12


A* Exploration:
##########
#S*......#        Guided toward goal,
#*#####..#        explores fewer cells
#***..#..#        while still finding
#*###.#..#        optimal path
#***#..*.#
#.#*####*#
#.#*****G#
#.#......#
##########
Visited: 20 cells, Path length: 12
```

### When to Use Each Algorithm

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| "Is there ANY path?" | DFS | Fast, low memory |
| "Find the SHORTEST path" | BFS or A* | Both optimal |
| "Find shortest path EFFICIENTLY" | A* | Fewer explorations |
| "Edges have DIFFERENT costs" | Dijkstra or A* | BFS can't handle weights |
| "Simple implementation needed" | BFS | Easy to code correctly |
| "Memory is very limited" | DFS | Only stores current path |
| "Known goal location" | A* | Can use heuristic |
| "Unknown goal location" | BFS or Dijkstra | Can't estimate distance |

---

## 6. Key Takeaways for Learning 🎓

### 1. Data Structure Determines Behavior
```
STACK (DFS)          → Last In, First Out  → Goes DEEP
QUEUE (BFS)          → First In, First Out → Goes WIDE
PRIORITY QUEUE (Dijkstra) → Lowest g First → Goes by COST
PRIORITY QUEUE (A*)  → Lowest g+h First   → Goes SMART
```

### 2. Optimality Comes from Exploration Order
```
DFS:      Explores in order of discovery (depth)
          → Might find goal via a long path first
          → NOT optimal

BFS:      Explores in order of distance from start
          → First path to goal = shortest path
          → OPTIMAL (unweighted)

Dijkstra: Explores in order of g(n) (actual cost)
          → First path to goal = cheapest path
          → OPTIMAL (weighted too)

A*:       Explores in order of f = g + h (estimated total cost)
          → First path to goal = shortest (if h is admissible)
          → OPTIMAL (and efficient!)
```

### 3. The Algorithm Hierarchy
```
BFS  ⊆  Dijkstra  ⊆  A*

• BFS is Dijkstra with all weights = 1
• Dijkstra is A* with h(n) = 0
• A* is Dijkstra + heuristic guidance
```

### 4. The Trade-off Triangle
```
        SPEED
         /\
        /  \
       /    \
      /      \
     /        \
    /__________\
MEMORY      OPTIMALITY

DFS:      Fast, Low Memory, Not Optimal
BFS:      Slow, High Memory, Optimal (unweighted)
Dijkstra: Medium, Medium Memory, Optimal (weighted)
A*:       Medium, Medium Memory, Optimal (best balance!)
```

### 5. Heuristics Make the Difference
```
A* with good heuristic: Explores 20 cells to find path
Dijkstra (h=0):         Explores 40 cells for same path
BFS:                    Explores 50 cells for same path

The heuristic "guides" A* toward promising directions,
avoiding wasted exploration in the wrong direction.
```

---

## 7. Practice Exercise 🏋️

Try this in the visualizer:

1. **Create a simple maze** with start on left, goal on right
2. **Run DFS** - observe the winding exploration
3. **Clear and run BFS** - observe the wave pattern
4. **Clear and run A*** - observe the guided exploration
5. **Compare**: Which explored fewer cells for the same path?

### Challenge Mazes to Try:

```
1. OPEN FIELD (no walls)
   - DFS: Random winding path
   - BFS: Diamond-shaped expansion
   - A*: Mostly straight line!

2. WALL BLOCKING DIRECT PATH
   ##########
   #S...#...#
   #....#...#
   #....#...#
   #........#
   #...#....#
   #...#...G#
   ##########
   - Compare how each navigates around the wall

3. MAZE WITH DEAD ENDS
   - DFS might get stuck in dead ends
   - BFS explores everything methodically
   - A* avoids obvious dead ends
```

---

## 8. Common Interview/Exam Questions 📝

**Q1: Why doesn't DFS guarantee the shortest path?**
> Because DFS explores depth-first, it might find a deep path to the goal before exploring a shorter, shallower path.

**Q2: Why is BFS optimal for unweighted graphs?**
> BFS explores nodes in order of their distance from the start. The first time it reaches any node is via the shortest path.

**Q3: What is the relationship between BFS, Dijkstra, and A*?**
> BFS ⊆ Dijkstra ⊆ A*. BFS is Dijkstra with all weights = 1. Dijkstra is A* with h(n) = 0. A* is Dijkstra plus a heuristic.

**Q4: When would you use Dijkstra over BFS?**
> When edges have different costs (weights). BFS counts hops (steps), not total cost, so it fails on weighted graphs.

**Q5: When would you use Dijkstra over A*?**
> When no good heuristic is available, or when the goal location is unknown. A* degrades to Dijkstra when h(n) = 0.

**Q6: What makes A* better than Dijkstra?**
> A* uses a heuristic to prioritize exploring nodes that seem closer to the goal, reducing unnecessary exploration while still guaranteeing optimality.

**Q7: What happens if A*'s heuristic overestimates?**
> The algorithm might not find the shortest path. Only admissible (never overestimating) heuristics guarantee optimality.

**Q8: When would you choose DFS over BFS?**
> When you only need to check if a path exists (not the shortest), when memory is limited, or when solutions are known to be deep in the search tree.

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────┐
│                    SEARCH ALGORITHMS                        │
├─────────────────────────────────────────────────────────────┤
│  DFS (Depth-First)                                          │
│  ├── Structure: Stack (LIFO)                                │
│  ├── Pattern: Deep → Backtrack → Deep                       │
│  ├── Optimal: NO                                            │
│  └── Memory: O(depth)                                       │
├─────────────────────────────────────────────────────────────┤
│  BFS (Breadth-First)                                        │
│  ├── Structure: Queue (FIFO)                                │
│  ├── Pattern: Level 0 → Level 1 → Level 2 → ...             │
│  ├── Optimal: YES (unweighted)                              │
│  └── Memory: O(branching^depth)                             │
├─────────────────────────────────────────────────────────────┤
│  Dijkstra's Algorithm                                        │
│  ├── Structure: Priority Queue (by g)                       │
│  ├── Pattern: Waves by cost (like BFS for weighted)          │
│  ├── Optimal: YES (weighted + unweighted)                   │
│  └── Memory: O(V)                                           │
├─────────────────────────────────────────────────────────────┤
│  A* (A-Star)                                                │
│  ├── Structure: Priority Queue (by f = g + h)               │
│  ├── Pattern: Best-first guided by heuristic                │
│  ├── Optimal: YES (if h is admissible)                      │
│  └── Memory: O(branching^depth) but less in practice        │
└─────────────────────────────────────────────────────────────┘
```

---

*Created for learning search algorithms. Practice with the visualizer to truly understand how each algorithm behaves!*
