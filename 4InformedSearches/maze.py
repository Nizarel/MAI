"""
Maze Module - Grid representation for pathfinding algorithms.

This module provides the core maze data structure used by all search algorithms.
The maze is represented as a 2D grid where each cell can be:
- EMPTY: A passable cell
- WALL: An impassable obstacle
- START: The starting position for pathfinding
- GOAL: The target destination
- VISITED: Cells that have been explored (for visualization)
- PATH: Cells on the final solution path (for visualization)

Educational Notes:
==================
- The maze uses a coordinate system where (0,0) is top-left
- row increases downward, col increases rightward
- Neighbors are the 4 adjacent cells (up, down, left, right)
- Diagonal movement is NOT allowed (4-connected grid)
"""

from enum import Enum
from typing import List, Tuple, Optional, Set


class CellType(Enum):
    """
    Enumeration of possible cell states in the maze.
    
    Using an enum makes the code more readable and catches typos at runtime.
    """
    EMPTY = 0      # Passable cell
    WALL = 1       # Blocked cell
    START = 2      # Starting position (only one allowed)
    GOAL = 3       # Goal position (only one allowed)
    VISITED = 4    # Cell has been explored by algorithm
    PATH = 5       # Cell is part of the final solution path


class Maze:
    """
    A 2D grid maze for pathfinding algorithm visualization.
    
    The maze maintains:
    - A grid of cells with their types
    - Positions of start and goal
    - Methods for modifying and querying the maze
    
    Example usage:
    --------------
    maze = Maze(rows=20, cols=20)
    maze.set_start(1, 1)
    maze.set_goal(18, 18)
    maze.set_wall(5, 5)
    neighbors = maze.get_neighbors(1, 1)
    """
    
    def __init__(self, rows: int = 25, cols: int = 25):
        """
        Initialize a maze with the given dimensions.
        
        Args:
            rows: Number of rows in the grid (default 25)
            cols: Number of columns in the grid (default 25)
        
        The maze is initialized with:
        - All interior cells as EMPTY
        - Border cells as WALL (creates a boundary)
        """
        self.rows = rows
        self.cols = cols
        self.start: Optional[Tuple[int, int]] = None
        self.goal: Optional[Tuple[int, int]] = None
        
        # Initialize grid with empty cells
        self.grid: List[List[CellType]] = [
            [CellType.EMPTY for _ in range(cols)] for _ in range(rows)
        ]
        
        # Create border walls
        self._create_border()
    
    def _create_border(self) -> None:
        """Create wall borders around the maze perimeter."""
        for row in range(self.rows):
            self.grid[row][0] = CellType.WALL           # Left border
            self.grid[row][self.cols - 1] = CellType.WALL  # Right border
        for col in range(self.cols):
            self.grid[0][col] = CellType.WALL           # Top border
            self.grid[self.rows - 1][col] = CellType.WALL  # Bottom border
    
    def is_valid(self, row: int, col: int) -> bool:
        """
        Check if coordinates are within maze bounds.
        
        Args:
            row: Row index to check
            col: Column index to check
            
        Returns:
            True if coordinates are valid, False otherwise
        """
        return 0 <= row < self.rows and 0 <= col < self.cols
    
    def is_passable(self, row: int, col: int) -> bool:
        """
        Check if a cell can be traversed (not a wall).
        
        Args:
            row: Row index to check
            col: Column index to check
            
        Returns:
            True if the cell is passable (EMPTY, START, GOAL, VISITED, or PATH)
        """
        if not self.is_valid(row, col):
            return False
        return self.grid[row][col] != CellType.WALL
    
    def get_cell(self, row: int, col: int) -> CellType:
        """Get the type of cell at the given position."""
        return self.grid[row][col]
    
    def set_cell(self, row: int, col: int, cell_type: CellType) -> None:
        """Set the type of cell at the given position."""
        if self.is_valid(row, col):
            self.grid[row][col] = cell_type
    
    def set_wall(self, row: int, col: int) -> bool:
        """
        Place a wall at the given position.
        
        Args:
            row: Row index
            col: Column index
            
        Returns:
            True if wall was placed, False if position is start/goal
        """
        if not self.is_valid(row, col):
            return False
        if (row, col) == self.start or (row, col) == self.goal:
            return False
        self.grid[row][col] = CellType.WALL
        return True
    
    def remove_wall(self, row: int, col: int) -> None:
        """Remove a wall (set cell to EMPTY) at the given position."""
        if self.is_valid(row, col) and self.grid[row][col] == CellType.WALL:
            # Don't remove border walls
            if row == 0 or row == self.rows - 1 or col == 0 or col == self.cols - 1:
                return
            self.grid[row][col] = CellType.EMPTY
    
    def set_start(self, row: int, col: int) -> bool:
        """
        Set the starting position for pathfinding.
        
        If a start already exists, it's cleared first.
        Cannot place start on a wall.
        
        Args:
            row: Row index for start position
            col: Column index for start position
            
        Returns:
            True if start was set successfully
        """
        if not self.is_valid(row, col) or self.grid[row][col] == CellType.WALL:
            return False
        
        # Clear previous start if exists
        if self.start:
            old_row, old_col = self.start
            self.grid[old_row][old_col] = CellType.EMPTY
        
        self.start = (row, col)
        self.grid[row][col] = CellType.START
        return True
    
    def set_goal(self, row: int, col: int) -> bool:
        """
        Set the goal position for pathfinding.
        
        If a goal already exists, it's cleared first.
        Cannot place goal on a wall.
        
        Args:
            row: Row index for goal position
            col: Column index for goal position
            
        Returns:
            True if goal was set successfully
        """
        if not self.is_valid(row, col) or self.grid[row][col] == CellType.WALL:
            return False
        
        # Clear previous goal if exists
        if self.goal:
            old_row, old_col = self.goal
            self.grid[old_row][old_col] = CellType.EMPTY
        
        self.goal = (row, col)
        self.grid[row][col] = CellType.GOAL
        return True
    
    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """
        Get all valid, passable neighbors of a cell.
        
        This is a key function used by all search algorithms!
        Returns the 4-connected neighbors (up, down, left, right).
        
        Args:
            row: Row index of current cell
            col: Column index of current cell
            
        Returns:
            List of (row, col) tuples for valid, passable neighbors
        
        Algorithm Note:
        ---------------
        The order of directions affects how DFS explores the maze.
        We use: UP, RIGHT, DOWN, LEFT (clockwise from top)
        """
        # Direction vectors: (row_delta, col_delta)
        # UP, RIGHT, DOWN, LEFT
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        
        neighbors = []
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if self.is_passable(new_row, new_col):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def reset_visualization(self) -> None:
        """
        Clear visited and path markers, keeping walls and start/goal.
        
        Call this before running a new algorithm to clean up
        the visualization from the previous run.
        """
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] in (CellType.VISITED, CellType.PATH):
                    self.grid[row][col] = CellType.EMPTY
        
        # Restore start and goal markers
        if self.start:
            self.grid[self.start[0]][self.start[1]] = CellType.START
        if self.goal:
            self.grid[self.goal[0]][self.goal[1]] = CellType.GOAL
    
    def reset_all(self) -> None:
        """
        Completely reset the maze to initial empty state.
        
        Clears all walls, start, goal, and visualization markers.
        Only the border walls remain.
        """
        self.start = None
        self.goal = None
        self.grid = [
            [CellType.EMPTY for _ in range(self.cols)] for _ in range(self.rows)
        ]
        self._create_border()
    
    def mark_visited(self, row: int, col: int) -> None:
        """Mark a cell as visited (for visualization)."""
        if self.is_valid(row, col):
            # Don't overwrite start and goal markers
            if self.grid[row][col] not in (CellType.START, CellType.GOAL, CellType.WALL):
                self.grid[row][col] = CellType.VISITED
    
    def mark_path(self, row: int, col: int) -> None:
        """Mark a cell as part of the solution path (for visualization)."""
        if self.is_valid(row, col):
            # Don't overwrite start and goal markers
            if self.grid[row][col] not in (CellType.START, CellType.GOAL, CellType.WALL):
                self.grid[row][col] = CellType.PATH
    
    def is_ready(self) -> bool:
        """Check if maze has both start and goal set."""
        return self.start is not None and self.goal is not None
    
    def __str__(self) -> str:
        """
        Create a string representation of the maze for debugging.
        
        Symbols:
        - '#' = Wall
        - 'S' = Start
        - 'G' = Goal
        - '.' = Empty
        - '*' = Visited
        - 'o' = Path
        """
        symbols = {
            CellType.EMPTY: '.',
            CellType.WALL: '#',
            CellType.START: 'S',
            CellType.GOAL: 'G',
            CellType.VISITED: '*',
            CellType.PATH: 'o',
        }
        
        lines = []
        for row in self.grid:
            line = ''.join(symbols[cell] for cell in row)
            lines.append(line)
        return '\n'.join(lines)


if __name__ == "__main__":
    # Simple test/demo
    print("Creating a 10x10 maze...")
    maze = Maze(10, 10)
    
    print("\nSetting start at (1,1) and goal at (8,8)...")
    maze.set_start(1, 1)
    maze.set_goal(8, 8)
    
    print("\nAdding some walls...")
    for i in range(2, 7):
        maze.set_wall(i, 5)
    
    print("\nMaze layout:")
    print(maze)
    
    print(f"\nNeighbors of (1,1): {maze.get_neighbors(1, 1)}")
    print(f"Is maze ready? {maze.is_ready()}")
