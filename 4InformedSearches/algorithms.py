"""
Search Algorithms Module - DFS, BFS, and A* implementations.

This module contains the core pathfinding algorithms used in the maze game.
All algorithms are implemented as **generators** (using yield) which allows
step-by-step visualization of the search process.

Educational Overview:
=====================

1. DFS (Depth-First Search) - UNINFORMED
   - Uses a STACK (LIFO - Last In, First Out)
   - Explores as deep as possible before backtracking
   - Does NOT guarantee shortest path
   - Memory efficient (stores only current path)
   - Time: O(V + E), Space: O(V) where V=vertices, E=edges

2. BFS (Breadth-First Search) - UNINFORMED  
   - Uses a QUEUE (FIFO - First In, First Out)
   - Explores all neighbors at current depth before going deeper
   - GUARANTEES shortest path (in unweighted graphs)
   - Uses more memory (stores all nodes at current depth)
   - Time: O(V + E), Space: O(V)

3. A* (A-Star) - INFORMED
   - Uses a PRIORITY QUEUE ordered by f(n) = g(n) + h(n)
   - g(n) = actual cost from start to current node
   - h(n) = heuristic estimate from current to goal
   - GUARANTEES shortest path (if heuristic is admissible)
   - More efficient than BFS due to heuristic guidance
   - Time: O(E log V), Space: O(V)

Key Concepts:
=============
- UNINFORMED: No knowledge of goal location (BFS, DFS)
- INFORMED: Uses heuristic to estimate distance to goal (A*, Greedy)
- ADMISSIBLE heuristic: Never overestimates actual cost
- Manhattan Distance: |x1-x2| + |y1-y2| - admissible for 4-connected grid
"""

from typing import List, Tuple, Dict, Set, Generator, Optional
from collections import deque
import heapq

from maze import Maze, CellType


# Type aliases for clarity
Position = Tuple[int, int]
Path = List[Position]


class SearchResult:
    """
    Container for search algorithm results.
    
    Attributes:
        path: List of positions from start to goal (empty if no path found)
        visited_count: Number of cells explored
        found: Whether a path was found
    """
    def __init__(self, path: Path = None, visited_count: int = 0, found: bool = False):
        self.path = path or []
        self.visited_count = visited_count
        self.found = found


def reconstruct_path(came_from: Dict[Position, Position], 
                     current: Position) -> Path:
    """
    Reconstruct the path from start to goal using the came_from dictionary.
    
    This is used by all algorithms to build the final path once the goal is found.
    
    Args:
        came_from: Dictionary mapping each position to its predecessor
        current: The goal position
        
    Returns:
        List of positions from start to goal
        
    How it works:
    -------------
    We start at the goal and follow the came_from links back to the start,
    then reverse the path to get start -> goal order.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# =============================================================================
# DFS - Depth-First Search
# =============================================================================

def dfs(maze: Maze) -> Generator[Tuple[Position, Set[Position], Path], None, SearchResult]:
    """
    Depth-First Search algorithm implemented as a generator.
    
    DFS explores as deep as possible along each branch before backtracking.
    It uses a STACK data structure (LIFO).
    
    Visualization:
    - DFS tends to explore in one direction until hitting a dead end
    - Creates long, winding exploration patterns
    - May find a goal quickly if it's in the search direction
    - Path found is usually NOT the shortest
    
    Yields:
        Tuple of (current_position, visited_set, current_path) at each step
        
    Returns:
        SearchResult with path (if found) and statistics
        
    Algorithm Steps:
    ----------------
    1. Push start onto stack
    2. While stack not empty:
       a. Pop top of stack (current)
       b. If current is goal, return path
       c. Mark current as visited
       d. Push all unvisited neighbors onto stack
    3. If stack empty, no path exists
    """
    if not maze.is_ready():
        return SearchResult(found=False)
    
    start = maze.start
    goal = maze.goal
    
    # Stack stores (position, path_to_position)
    # We store the path to reconstruct it easily
    stack: List[Tuple[Position, Path]] = [(start, [start])]
    
    # Set of visited positions to avoid cycles
    visited: Set[Position] = set()
    
    while stack:
        # Pop from stack (LIFO - Last In, First Out)
        current, path = stack.pop()
        
        # Skip if already visited
        if current in visited:
            continue
        
        # Mark as visited
        visited.add(current)
        
        # Yield current state for visualization
        yield (current, visited.copy(), path.copy())
        
        # Check if we reached the goal
        if current == goal:
            return SearchResult(path=path, visited_count=len(visited), found=True)
        
        # Add all unvisited neighbors to stack
        # Note: Order matters! Last added = first explored
        for neighbor in maze.get_neighbors(current[0], current[1]):
            if neighbor not in visited:
                new_path = path + [neighbor]
                stack.append((neighbor, new_path))
    
    # No path found
    return SearchResult(visited_count=len(visited), found=False)


# =============================================================================
# BFS - Breadth-First Search
# =============================================================================

def bfs(maze: Maze) -> Generator[Tuple[Position, Set[Position], Path], None, SearchResult]:
    """
    Breadth-First Search algorithm implemented as a generator.
    
    BFS explores all neighbors at the current depth before moving deeper.
    It uses a QUEUE data structure (FIFO).
    
    **GUARANTEES SHORTEST PATH** in unweighted graphs!
    
    Visualization:
    - BFS expands outward in "waves" from the start
    - Creates a circular/diamond exploration pattern
    - All cells at distance N are explored before any at distance N+1
    
    Yields:
        Tuple of (current_position, visited_set, current_path) at each step
        
    Returns:
        SearchResult with shortest path (if found) and statistics
        
    Algorithm Steps:
    ----------------
    1. Enqueue start
    2. While queue not empty:
       a. Dequeue front of queue (current)
       b. If current is goal, return path
       c. For each unvisited neighbor:
          - Mark as visited
          - Enqueue neighbor
    3. If queue empty, no path exists
    
    Why BFS finds shortest path:
    ----------------------------
    Because we explore level by level, the first time we reach any cell
    is guaranteed to be via the shortest path to that cell.
    """
    if not maze.is_ready():
        return SearchResult(found=False)
    
    start = maze.start
    goal = maze.goal
    
    # Queue stores positions (use deque for O(1) pop from left)
    queue: deque = deque([start])
    
    # Track visited AND where we came from
    # This allows path reconstruction without storing full paths
    visited: Set[Position] = {start}
    came_from: Dict[Position, Position] = {}
    
    while queue:
        # Dequeue from front (FIFO - First In, First Out)
        current = queue.popleft()
        
        # Build current path for visualization
        path = reconstruct_path(came_from, current)
        
        # Yield current state for visualization
        yield (current, visited.copy(), path)
        
        # Check if we reached the goal
        if current == goal:
            final_path = reconstruct_path(came_from, goal)
            return SearchResult(path=final_path, visited_count=len(visited), found=True)
        
        # Add all unvisited neighbors to queue
        for neighbor in maze.get_neighbors(current[0], current[1]):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
    
    # No path found
    return SearchResult(visited_count=len(visited), found=False)


# =============================================================================
# A* - A-Star Search
# =============================================================================

def manhattan_distance(pos1: Position, pos2: Position) -> int:
    """
    Calculate Manhattan distance between two positions.
    
    Also called "taxicab distance" or "L1 distance".
    This is the sum of horizontal and vertical distances.
    
    Formula: |x1 - x2| + |y1 - y2|
    
    Why Manhattan for grids:
    ------------------------
    - In a 4-connected grid (no diagonals), Manhattan distance
      is the EXACT minimum number of steps needed
    - This makes it an ADMISSIBLE heuristic (never overestimates)
    - Admissibility guarantees A* finds the optimal path
    
    Example:
        From (1, 1) to (4, 5):
        |1-4| + |1-5| = 3 + 4 = 7 steps minimum
    """
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


def astar(maze: Maze) -> Generator[Tuple[Position, Set[Position], Path], None, SearchResult]:
    """
    A* Search algorithm implemented as a generator.
    
    A* uses a heuristic to guide search toward the goal, making it much
    more efficient than BFS while still guaranteeing the shortest path.
    
    **INFORMED SEARCH** - Uses knowledge about goal location!
    **GUARANTEES SHORTEST PATH** (with admissible heuristic)
    
    Key formula: f(n) = g(n) + h(n)
    - f(n) = total estimated cost through node n
    - g(n) = actual cost from start to n
    - h(n) = heuristic estimate from n to goal
    
    Visualization:
    - A* explores cells in order of f(n) = g(n) + h(n)
    - Tends to explore toward the goal first
    - Explores fewer cells than BFS in most cases
    - Creates a more "focused" search pattern
    
    Yields:
        Tuple of (current_position, visited_set, current_path) at each step
        
    Returns:
        SearchResult with optimal path (if found) and statistics
        
    Algorithm Steps:
    ----------------
    1. Add start to priority queue with f = h(start)
    2. While priority queue not empty:
       a. Pop node with lowest f value (current)
       b. If current is goal, return path
       c. For each neighbor:
          - Calculate g = g(current) + 1
          - If new g is better than recorded g for neighbor:
            - Update g and f values
            - Update came_from
            - Add/update neighbor in priority queue
    3. If queue empty, no path exists
    """
    if not maze.is_ready():
        return SearchResult(found=False)
    
    start = maze.start
    goal = maze.goal
    
    # Priority queue: (f_score, counter, position)
    # Counter is used to break ties (FIFO behavior)
    counter = 0
    open_set: List[Tuple[int, int, Position]] = []
    heapq.heappush(open_set, (manhattan_distance(start, goal), counter, start))
    
    # Track where we came from for path reconstruction
    came_from: Dict[Position, Position] = {}
    
    # g_score: actual cost from start to each node
    # Default to infinity for unvisited nodes
    g_score: Dict[Position, int] = {start: 0}
    
    # f_score: g_score + heuristic
    f_score: Dict[Position, int] = {start: manhattan_distance(start, goal)}
    
    # Track what's in open_set for O(1) lookup
    open_set_hash: Set[Position] = {start}
    
    # Track all visited nodes (for visualization)
    visited: Set[Position] = set()
    
    while open_set:
        # Pop node with lowest f_score
        _, _, current = heapq.heappop(open_set)
        open_set_hash.discard(current)
        
        # Mark as visited (closed set)
        visited.add(current)
        
        # Build current path for visualization
        path = reconstruct_path(came_from, current)
        
        # Yield current state for visualization
        yield (current, visited.copy(), path)
        
        # Check if we reached the goal
        if current == goal:
            final_path = reconstruct_path(came_from, goal)
            return SearchResult(path=final_path, visited_count=len(visited), found=True)
        
        # Explore neighbors
        for neighbor in maze.get_neighbors(current[0], current[1]):
            # Calculate tentative g_score
            # (cost to reach neighbor through current)
            tentative_g = g_score[current] + 1  # All edges have cost 1
            
            # Is this a better path to neighbor?
            if tentative_g < g_score.get(neighbor, float('inf')):
                # Yes! Update the path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + manhattan_distance(neighbor, goal)
                
                # Add to open set if not already there
                if neighbor not in open_set_hash:
                    counter += 1
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    open_set_hash.add(neighbor)
    
    # No path found
    return SearchResult(visited_count=len(visited), found=False)


# =============================================================================
# Algorithm Registry - For easy access by name
# =============================================================================

ALGORITHMS = {
    'dfs': ('DFS (Depth-First Search)', dfs),
    'bfs': ('BFS (Breadth-First Search)', bfs),
    'astar': ('A* (A-Star Search)', astar),
}


def get_algorithm(name: str):
    """
    Get an algorithm function by name.
    
    Args:
        name: One of 'dfs', 'bfs', 'astar'
        
    Returns:
        The algorithm generator function
        
    Raises:
        KeyError if name is not recognized
    """
    return ALGORITHMS[name.lower()][1]


def get_algorithm_name(name: str) -> str:
    """Get the full display name of an algorithm."""
    return ALGORITHMS[name.lower()][0]


# =============================================================================
# Demo / Test
# =============================================================================

if __name__ == "__main__":
    # Create a simple test maze
    print("=" * 60)
    print("Search Algorithms Demo")
    print("=" * 60)
    
    maze = Maze(10, 15)
    maze.set_start(1, 1)
    maze.set_goal(8, 13)
    
    # Add some walls
    for i in range(2, 8):
        maze.set_wall(i, 7)
    
    print("\nMaze layout:")
    print(maze)
    print()
    
    # Test each algorithm
    for algo_key in ['dfs', 'bfs', 'astar']:
        algo_name, algo_func = ALGORITHMS[algo_key]
        print(f"\n{algo_name}:")
        print("-" * 40)
        
        # Reset visualization
        maze.reset_visualization()
        
        # Run algorithm (consume generator to get result)
        gen = algo_func(maze)
        result = None
        steps = 0
        
        try:
            while True:
                current, visited, path = next(gen)
                steps += 1
        except StopIteration as e:
            result = e.value
        
        if result and result.found:
            print(f"  Path found! Length: {len(result.path)}")
            print(f"  Cells explored: {result.visited_count}")
            print(f"  Steps taken: {steps}")
            print(f"  Path: {' -> '.join(str(p) for p in result.path[:5])}...")
        else:
            print("  No path found!")
