"""
Terminal UI - Curses-based maze visualization.

This module provides a text-based interface for the maze pathfinding game.
It uses the curses library (built into Python on Linux/Mac) to create
an interactive terminal application with colors and keyboard input.

Controls:
=========
  Arrow keys  - Move cursor
  w           - Place/toggle wall at cursor
  s           - Set start position
  g           - Set goal position
  1           - Run DFS
  2           - Run BFS
  3           - Run A*
  4           - Run Dijkstra
  r           - Reset maze (clear walls and markers)
  c           - Clear visualization only (keep walls)
  q           - Quit

Color Legend:
=============
  Red         - Wall
  Yellow      - Start position
  Cyan        - Goal position  
  Magenta     - Visited cells
  Green       - Solution path
  White       - Empty cells
"""

import curses
import time
from typing import Optional

from maze import Maze, CellType
from algorithms import dfs, bfs, astar, dijkstra, SearchResult


# Animation delay in seconds (how fast the visualization runs)
ANIMATION_DELAY = 0.03  # 30ms per step


class TerminalUI:
    """
    Curses-based terminal interface for the maze pathfinding game.
    
    This class handles:
    - Drawing the maze to the terminal
    - Processing keyboard input
    - Animating search algorithm execution
    """
    
    def __init__(self, rows: int = 20, cols: int = 35):
        """
        Initialize the terminal UI.
        
        Args:
            rows: Number of rows in the maze
            cols: Number of columns in the maze
        """
        self.maze = Maze(rows, cols)
        self.cursor_row = 1
        self.cursor_col = 1
        self.status_message = "Use arrow keys to move, 'w'=wall, 's'=start, 'g'=goal"
        self.running = True
        self.animation_speed = ANIMATION_DELAY
        
    def _init_colors(self) -> None:
        """Initialize curses color pairs."""
        curses.start_color()
        curses.use_default_colors()
        
        # Color pairs: (foreground, background)
        curses.init_pair(1, curses.COLOR_RED, -1)      # Walls
        curses.init_pair(2, curses.COLOR_YELLOW, -1)   # Start
        curses.init_pair(3, curses.COLOR_CYAN, -1)     # Goal
        curses.init_pair(4, curses.COLOR_MAGENTA, -1)  # Visited
        curses.init_pair(5, curses.COLOR_GREEN, -1)    # Path
        curses.init_pair(6, curses.COLOR_WHITE, -1)    # Empty
        curses.init_pair(7, curses.COLOR_BLACK, curses.COLOR_WHITE)  # Cursor
        
    def _get_cell_display(self, cell_type: CellType) -> tuple:
        """
        Get the character and color pair for a cell type.
        
        Returns:
            Tuple of (character, color_pair_number)
        """
        display_map = {
            CellType.WALL:    ('█', 1),   # Red block
            CellType.START:   ('S', 2),   # Yellow S
            CellType.GOAL:    ('G', 3),   # Cyan G
            CellType.VISITED: ('·', 4),   # Magenta dot
            CellType.PATH:    ('●', 5),   # Green filled circle
            CellType.EMPTY:   (' ', 6),   # White space
        }
        return display_map.get(cell_type, (' ', 6))
    
    def _draw_maze(self, stdscr) -> None:
        """Draw the entire maze to the screen."""
        for row in range(self.maze.rows):
            for col in range(self.maze.cols):
                cell_type = self.maze.get_cell(row, col)
                char, color_pair = self._get_cell_display(cell_type)
                
                # Highlight cursor position
                if row == self.cursor_row and col == self.cursor_col:
                    stdscr.addstr(row, col * 2, f"[{char}]"[:3], 
                                  curses.color_pair(7) | curses.A_BOLD)
                else:
                    # Double-width for better aspect ratio
                    stdscr.addstr(row, col * 2, char * 2, curses.color_pair(color_pair))
    
    def _draw_ui(self, stdscr) -> None:
        """Draw the full UI including maze and status."""
        stdscr.clear()
        
        # Draw title
        title = "=== MAZE PATHFINDING VISUALIZER ==="
        stdscr.addstr(0, 0, title, curses.A_BOLD)
        
        # Offset maze display
        maze_start_row = 2
        
        # Draw maze
        for row in range(self.maze.rows):
            for col in range(self.maze.cols):
                cell_type = self.maze.get_cell(row, col)
                char, color_pair = self._get_cell_display(cell_type)
                
                display_row = row + maze_start_row
                display_col = col * 2
                
                try:
                    # Highlight cursor position
                    if row == self.cursor_row and col == self.cursor_col:
                        stdscr.addstr(display_row, display_col, char * 2, 
                                      curses.color_pair(7) | curses.A_BOLD)
                    else:
                        stdscr.addstr(display_row, display_col, char * 2, 
                                      curses.color_pair(color_pair))
                except curses.error:
                    pass  # Ignore if writing outside screen
        
        # Draw status bar
        status_row = maze_start_row + self.maze.rows + 1
        try:
            stdscr.addstr(status_row, 0, self.status_message[:80])
            
            # Draw controls help
            controls = [
                "Controls: ↑↓←→=move  w=wall  s=start  g=goal",
                "          1=DFS  2=BFS  3=A*  4=Dijkstra  r=reset  c=clear  q=quit",
            ]
            for i, line in enumerate(controls):
                stdscr.addstr(status_row + 2 + i, 0, line)
                
        except curses.error:
            pass
        
        stdscr.refresh()
    
    def _run_algorithm(self, stdscr, algorithm_func, name: str) -> None:
        """
        Run a search algorithm with step-by-step visualization.
        
        Args:
            stdscr: Curses screen object
            algorithm_func: Generator function (dfs, bfs, or astar)
            name: Display name of the algorithm
        """
        if not self.maze.is_ready():
            self.status_message = "ERROR: Set both START (s) and GOAL (g) first!"
            return
        
        # Clear previous visualization
        self.maze.reset_visualization()
        self.status_message = f"Running {name}..."
        self._draw_ui(stdscr)
        
        # Run algorithm with animation
        gen = algorithm_func(self.maze)
        steps = 0
        result: Optional[SearchResult] = None
        
        try:
            while True:
                current, visited, path = next(gen)
                steps += 1
                
                # Update visited cells in maze
                for pos in visited:
                    self.maze.mark_visited(pos[0], pos[1])
                
                # Highlight current cell
                self.status_message = f"{name}: Step {steps}, exploring {current}"
                self._draw_ui(stdscr)
                
                # Animation delay
                time.sleep(self.animation_speed)
                
                # Check for keypress to cancel
                stdscr.nodelay(True)
                key = stdscr.getch()
                if key == ord('q') or key == 27:  # q or ESC
                    self.status_message = f"{name} cancelled after {steps} steps"
                    stdscr.nodelay(False)
                    return
                stdscr.nodelay(False)
                
        except StopIteration as e:
            result = e.value
        
        # Show result
        if result and result.found:
            # Mark the path
            for pos in result.path:
                self.maze.mark_path(pos[0], pos[1])
            
            self.status_message = (
                f"{name} DONE! Path length: {len(result.path)}, "
                f"Cells explored: {result.visited_count}"
            )
        else:
            self.status_message = f"{name}: No path found! Explored {result.visited_count if result else 0} cells"
        
        self._draw_ui(stdscr)
    
    def _handle_input(self, stdscr, key: int) -> None:
        """Process a single keypress."""
        # Arrow keys for cursor movement
        if key == curses.KEY_UP:
            self.cursor_row = max(1, self.cursor_row - 1)
        elif key == curses.KEY_DOWN:
            self.cursor_row = min(self.maze.rows - 2, self.cursor_row + 1)
        elif key == curses.KEY_LEFT:
            self.cursor_col = max(1, self.cursor_col - 1)
        elif key == curses.KEY_RIGHT:
            self.cursor_col = min(self.maze.cols - 2, self.cursor_col + 1)
        
        # Wall toggle
        elif key == ord('w'):
            cell = self.maze.get_cell(self.cursor_row, self.cursor_col)
            if cell == CellType.WALL:
                self.maze.remove_wall(self.cursor_row, self.cursor_col)
                self.status_message = f"Removed wall at ({self.cursor_row}, {self.cursor_col})"
            elif cell == CellType.EMPTY:
                self.maze.set_wall(self.cursor_row, self.cursor_col)
                self.status_message = f"Placed wall at ({self.cursor_row}, {self.cursor_col})"
        
        # Start position
        elif key == ord('s'):
            if self.maze.set_start(self.cursor_row, self.cursor_col):
                self.status_message = f"Start set at ({self.cursor_row}, {self.cursor_col})"
            else:
                self.status_message = "Cannot place start here!"
        
        # Goal position
        elif key == ord('g'):
            if self.maze.set_goal(self.cursor_row, self.cursor_col):
                self.status_message = f"Goal set at ({self.cursor_row}, {self.cursor_col})"
            else:
                self.status_message = "Cannot place goal here!"
        
        # Run algorithms
        elif key == ord('1'):
            self._run_algorithm(stdscr, dfs, "DFS")
        elif key == ord('2'):
            self._run_algorithm(stdscr, bfs, "BFS")
        elif key == ord('3'):
            self._run_algorithm(stdscr, astar, "A*")
        elif key == ord('4'):
            self._run_algorithm(stdscr, dijkstra, "Dijkstra")
        
        # Reset
        elif key == ord('r'):
            self.maze.reset_all()
            self.status_message = "Maze reset!"
        
        # Clear visualization only
        elif key == ord('c'):
            self.maze.reset_visualization()
            self.status_message = "Visualization cleared (walls kept)"
        
        # Quit
        elif key == ord('q'):
            self.running = False
        
        # Speed adjustment
        elif key == ord('+') or key == ord('='):
            self.animation_speed = max(0.01, self.animation_speed - 0.01)
            self.status_message = f"Speed increased (delay: {self.animation_speed:.2f}s)"
        elif key == ord('-'):
            self.animation_speed = min(0.5, self.animation_speed + 0.01)
            self.status_message = f"Speed decreased (delay: {self.animation_speed:.2f}s)"
    
    def run(self, stdscr) -> None:
        """Main loop for the terminal UI."""
        # Setup
        curses.curs_set(0)  # Hide cursor
        self._init_colors()
        
        # Main loop
        while self.running:
            self._draw_ui(stdscr)
            
            key = stdscr.getch()
            self._handle_input(stdscr, key)


def main():
    """Entry point for terminal UI."""
    print("Starting Maze Pathfinding Visualizer (Terminal Mode)...")
    print("Window size should be at least 80x30 characters.")
    time.sleep(1)
    
    ui = TerminalUI(rows=20, cols=35)
    curses.wrapper(ui.run)
    
    print("\nThanks for exploring search algorithms!")


if __name__ == "__main__":
    main()
