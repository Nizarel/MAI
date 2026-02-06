"""
Pygame UI - Graphical maze visualization.

This module provides a graphical interface for the maze pathfinding game
using the Pygame library. It creates an interactive window matching the
reference design with clickable buttons and animated algorithm execution.

Controls:
=========
  Left Click       - Place wall (or set start/goal in order)
  Right Click      - Remove wall
  Left Click+Drag  - Draw multiple walls
  
Buttons:
========
  Reset   - Clear entire maze
  DFS     - Run Depth-First Search
  BFS     - Run Breadth-First Search  
  Dijkstra - Run Dijkstra's Algorithm
  A*      - Run A* Search

Color Legend:
=============
  Red (dark)   - Wall
  Red (light)  - Border
  Yellow       - Start position
  Green        - Goal position
  Purple       - Visited cells
  Orange/Red   - Solution path
  Black        - Empty cells
"""

import pygame
import sys
import time
from typing import Optional, Tuple

from maze import Maze, CellType
from algorithms import dfs, bfs, astar, dijkstra, SearchResult


# Window configuration
CELL_SIZE = 20
GRID_ROWS = 25
GRID_COLS = 25
GRID_WIDTH = GRID_COLS * CELL_SIZE
GRID_HEIGHT = GRID_ROWS * CELL_SIZE

# UI layout
BUTTON_HEIGHT = 40
BUTTON_WIDTH = 80
BUTTON_MARGIN = 10
TOP_MARGIN = 60  # Space for buttons above grid

WINDOW_WIDTH = GRID_WIDTH + 40  # Padding on sides
WINDOW_HEIGHT = GRID_HEIGHT + TOP_MARGIN + 60  # Buttons + grid + status

# Colors (RGB)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
DARK_GRAY = (40, 40, 40)

# Maze colors (matching reference image)
COLOR_EMPTY = (0, 0, 0)            # Black
COLOR_WALL = (220, 60, 60)         # Red
COLOR_BORDER = (180, 50, 50)       # Darker red for border
COLOR_START = (255, 220, 0)        # Yellow
COLOR_GOAL = (0, 255, 100)         # Green
COLOR_VISITED = (150, 80, 180)     # Purple
COLOR_PATH = (255, 100, 50)        # Orange-red
COLOR_CURRENT = (255, 255, 255)    # White for current cell

# Button colors
BUTTON_COLOR = (60, 130, 60)       # Green
BUTTON_HOVER = (80, 160, 80)       # Lighter green
BUTTON_TEXT = WHITE


class Button:
    """A clickable button with hover effect."""
    
    def __init__(self, x: int, y: int, width: int, height: int, text: str):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.hovered = False
    
    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        """Draw the button."""
        color = BUTTON_HOVER if self.hovered else BUTTON_COLOR
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, WHITE, self.rect, 2)  # Border
        
        text_surface = font.render(self.text, True, BUTTON_TEXT)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)
    
    def handle_event(self, event: pygame.event.Event) -> bool:
        """Check if button was clicked. Returns True if clicked."""
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False


class PygameUI:
    """
    Pygame-based graphical interface for the maze pathfinding game.
    """
    
    def __init__(self):
        """Initialize Pygame and create the game window."""
        pygame.init()
        pygame.display.set_caption("Maze Pathfinding Visualizer - Learn Search Algorithms")
        
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 28)
        self.small_font = pygame.font.Font(None, 22)
        
        # Maze
        self.maze = Maze(GRID_ROWS, GRID_COLS)
        
        # Grid offset (to center it)
        self.grid_x = (WINDOW_WIDTH - GRID_WIDTH) // 2
        self.grid_y = TOP_MARGIN
        
        # State
        self.placing_mode = 'start'  # 'start', 'goal', 'wall'
        self.status_message = "Click to place START, then GOAL, then draw walls"
        self.running = True
        self.algorithm_running = False
        
        # Animation settings
        self.animation_delay = 20  # milliseconds
        
        # Create buttons
        self._create_buttons()
    
    def _create_buttons(self) -> None:
        """Create the control buttons."""
        button_y = 10
        start_x = (WINDOW_WIDTH - (5 * BUTTON_WIDTH + 4 * BUTTON_MARGIN)) // 2
        
        self.buttons = {
            'reset': Button(start_x, button_y, BUTTON_WIDTH, BUTTON_HEIGHT, "Reset"),
            'dfs': Button(start_x + BUTTON_WIDTH + BUTTON_MARGIN, button_y, 
                         BUTTON_WIDTH, BUTTON_HEIGHT, "DFS"),
            'bfs': Button(start_x + 2 * (BUTTON_WIDTH + BUTTON_MARGIN), button_y,
                         BUTTON_WIDTH, BUTTON_HEIGHT, "BFS"),
            'dijkstra': Button(start_x + 3 * (BUTTON_WIDTH + BUTTON_MARGIN), button_y,
                              BUTTON_WIDTH, BUTTON_HEIGHT, "Dijkstra"),
            'astar': Button(start_x + 4 * (BUTTON_WIDTH + BUTTON_MARGIN), button_y,
                           BUTTON_WIDTH, BUTTON_HEIGHT, "A*"),
        }
    
    def _get_cell_color(self, cell_type: CellType) -> Tuple[int, int, int]:
        """Get the color for a cell type."""
        color_map = {
            CellType.EMPTY: COLOR_EMPTY,
            CellType.WALL: COLOR_WALL,
            CellType.START: COLOR_START,
            CellType.GOAL: COLOR_GOAL,
            CellType.VISITED: COLOR_VISITED,
            CellType.PATH: COLOR_PATH,
        }
        return color_map.get(cell_type, COLOR_EMPTY)
    
    def _screen_to_grid(self, x: int, y: int) -> Optional[Tuple[int, int]]:
        """Convert screen coordinates to grid coordinates."""
        grid_x = (x - self.grid_x) // CELL_SIZE
        grid_y = (y - self.grid_y) // CELL_SIZE
        
        if 0 <= grid_x < GRID_COLS and 0 <= grid_y < GRID_ROWS:
            return (grid_y, grid_x)  # Return as (row, col)
        return None
    
    def _draw_grid(self) -> None:
        """Draw the maze grid."""
        for row in range(self.maze.rows):
            for col in range(self.maze.cols):
                cell_type = self.maze.get_cell(row, col)
                color = self._get_cell_color(cell_type)
                
                x = self.grid_x + col * CELL_SIZE
                y = self.grid_y + row * CELL_SIZE
                
                # Draw cell
                pygame.draw.rect(self.screen, color, 
                               (x, y, CELL_SIZE - 1, CELL_SIZE - 1))
                
                # Draw grid lines (subtle)
                pygame.draw.rect(self.screen, DARK_GRAY,
                               (x, y, CELL_SIZE, CELL_SIZE), 1)
    
    def _draw_ui(self) -> None:
        """Draw the complete UI."""
        self.screen.fill(BLACK)
        
        # Draw buttons
        for button in self.buttons.values():
            button.draw(self.screen, self.font)
        
        # Draw grid
        self._draw_grid()
        
        # Draw status bar
        status_y = self.grid_y + GRID_HEIGHT + 15
        status_surface = self.small_font.render(self.status_message, True, WHITE)
        self.screen.blit(status_surface, (self.grid_x, status_y))
        
        # Draw mode indicator
        mode_text = f"Mode: {self.placing_mode.upper()}"
        mode_surface = self.small_font.render(mode_text, True, GRAY)
        self.screen.blit(mode_surface, (self.grid_x, status_y + 20))
        
        pygame.display.flip()
    
    def _handle_grid_click(self, pos: Tuple[int, int], button: int) -> None:
        """Handle mouse click on the grid."""
        grid_pos = self._screen_to_grid(pos[0], pos[1])
        if not grid_pos:
            return
        
        row, col = grid_pos
        
        # Right click: remove wall
        if button == 3:
            self.maze.remove_wall(row, col)
            return
        
        # Left click: place based on mode
        if button == 1:
            if self.placing_mode == 'start':
                if self.maze.set_start(row, col):
                    self.placing_mode = 'goal'
                    self.status_message = "Now click to place GOAL"
            elif self.placing_mode == 'goal':
                if self.maze.set_goal(row, col):
                    self.placing_mode = 'wall'
                    self.status_message = "Draw walls with left-click, right-click to erase"
            else:  # wall mode
                cell = self.maze.get_cell(row, col)
                if cell == CellType.EMPTY:
                    self.maze.set_wall(row, col)
                elif cell == CellType.WALL:
                    self.maze.remove_wall(row, col)
    
    def _run_algorithm(self, algorithm_func, name: str) -> None:
        """Run a search algorithm with visualization."""
        if not self.maze.is_ready():
            self.status_message = "ERROR: Place both START and GOAL first!"
            return
        
        self.algorithm_running = True
        self.maze.reset_visualization()
        self.status_message = f"Running {name}..."
        self._draw_ui()
        
        # Run algorithm with animation
        gen = algorithm_func(self.maze)
        steps = 0
        result: Optional[SearchResult] = None
        
        try:
            while True:
                current, visited, path = next(gen)
                steps += 1
                
                # Update visited cells
                for pos in visited:
                    self.maze.mark_visited(pos[0], pos[1])
                
                # Draw update
                self.status_message = f"{name}: Step {steps}, exploring ({current[0]}, {current[1]})"
                self._draw_ui()
                
                # Handle events during animation
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.status_message = f"{name} cancelled"
                            self.algorithm_running = False
                            return
                
                # Animation delay
                pygame.time.wait(self.animation_delay)
                
        except StopIteration as e:
            result = e.value
        
        # Show result
        if result and result.found:
            # Mark the path
            for pos in result.path:
                self.maze.mark_path(pos[0], pos[1])
            
            self.status_message = (
                f"{name} COMPLETE! Path: {len(result.path)} cells, "
                f"Explored: {result.visited_count} cells"
            )
        else:
            explored = result.visited_count if result else 0
            self.status_message = f"{name}: No path found! Explored {explored} cells"
        
        self.algorithm_running = False
        self._draw_ui()
    
    def _handle_events(self) -> None:
        """Process pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            
            # Button events
            for name, button in self.buttons.items():
                if button.handle_event(event):
                    if name == 'reset':
                        self.maze.reset_all()
                        self.placing_mode = 'start'
                        self.status_message = "Maze reset! Click to place START"
                    elif name == 'dfs':
                        self._run_algorithm(dfs, "DFS")
                    elif name == 'bfs':
                        self._run_algorithm(bfs, "BFS")
                    elif name == 'dijkstra':
                        self._run_algorithm(dijkstra, "Dijkstra")
                    elif name == 'astar':
                        self._run_algorithm(astar, "A*")
            
            # Grid click
            if event.type == pygame.MOUSEBUTTONDOWN:
                if not self.algorithm_running:
                    self._handle_grid_click(event.pos, event.button)
            
            # Mouse drag for drawing walls
            elif event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0] and self.placing_mode == 'wall':
                    grid_pos = self._screen_to_grid(event.pos[0], event.pos[1])
                    if grid_pos:
                        row, col = grid_pos
                        if self.maze.get_cell(row, col) == CellType.EMPTY:
                            self.maze.set_wall(row, col)
            
            # Keyboard shortcuts
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    self.maze.reset_all()
                    self.placing_mode = 'start'
                    self.status_message = "Maze reset!"
                elif event.key == pygame.K_c:
                    self.maze.reset_visualization()
                    self.status_message = "Visualization cleared"
                elif event.key == pygame.K_1:
                    self._run_algorithm(dfs, "DFS")
                elif event.key == pygame.K_2:
                    self._run_algorithm(bfs, "BFS")
                elif event.key == pygame.K_3:
                    self._run_algorithm(astar, "A*")
                elif event.key == pygame.K_4:
                    self._run_algorithm(dijkstra, "Dijkstra")
                elif event.key == pygame.K_ESCAPE:
                    self.running = False
    
    def run(self) -> None:
        """Main game loop."""
        while self.running:
            self._handle_events()
            self._draw_ui()
            self.clock.tick(60)  # 60 FPS
        
        pygame.quit()


def main():
    """Entry point for Pygame UI."""
    print("Starting Maze Pathfinding Visualizer (GUI Mode)...")
    ui = PygameUI()
    ui.run()
    print("\nThanks for exploring search algorithms!")


if __name__ == "__main__":
    main()
