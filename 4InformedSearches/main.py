#!/usr/bin/env python3
"""
Maze Pathfinding Visualizer - Main Entry Point

A visual tool for learning and comparing search algorithms:
- DFS (Depth-First Search) - Stack-based, explores deep first
- BFS (Breadth-First Search) - Queue-based, guarantees shortest path
- A* (A-Star) - Priority queue with heuristic, optimal and efficient

This program offers two visualization modes:
1. Terminal Mode (--mode terminal): ASCII graphics using curses
2. GUI Mode (--mode gui): Graphical interface using Pygame

Usage:
======
  python main.py                 # Default: GUI mode if pygame available
  python main.py --mode gui      # Force GUI mode (requires pygame)
  python main.py --mode terminal # Force terminal mode (no dependencies)
  python main.py --help          # Show help

Learning Objectives:
====================
- Understand how different search algorithms explore a space
- See why BFS guarantees shortest path but A* is more efficient
- Observe how DFS can find long, winding paths
- Compare the number of cells explored by each algorithm

Author: MAI Course
"""

import argparse
import sys


def check_pygame_available() -> bool:
    """Check if pygame is installed."""
    try:
        import pygame
        return True
    except ImportError:
        return False


def run_terminal_mode():
    """Start the terminal-based UI."""
    from terminal_ui import main as terminal_main
    terminal_main()


def run_gui_mode():
    """Start the Pygame-based UI."""
    if not check_pygame_available():
        print("ERROR: Pygame is not installed!")
        print("Install it with: pip install pygame")
        print("\nAlternatively, run in terminal mode: python main.py --mode terminal")
        sys.exit(1)
    
    from pygame_ui import main as pygame_main
    pygame_main()


def run_demo():
    """Run a quick algorithm demo without UI."""
    print("=" * 60)
    print("      MAZE PATHFINDING ALGORITHMS - QUICK DEMO")
    print("=" * 60)
    
    from maze import Maze
    from algorithms import dfs, bfs, astar, ALGORITHMS
    
    # Create test maze
    maze = Maze(15, 20)
    maze.set_start(1, 1)
    maze.set_goal(13, 18)
    
    # Add walls to make it interesting
    # Vertical wall
    for i in range(2, 12):
        maze.set_wall(i, 10)
    # Horizontal walls
    for j in range(5, 15):
        maze.set_wall(5, j)
    for j in range(3, 12):
        maze.set_wall(10, j)
    
    print("\nMaze layout:")
    print("  '#' = Wall, 'S' = Start, 'G' = Goal, '.' = Empty")
    print()
    print(maze)
    
    print("\n" + "=" * 60)
    print("RUNNING ALGORITHMS")
    print("=" * 60)
    
    results = {}
    
    for algo_key in ['dfs', 'bfs', 'astar']:
        algo_name, algo_func = ALGORITHMS[algo_key]
        
        # Reset maze visualization
        maze.reset_visualization()
        
        # Run algorithm
        gen = algo_func(maze)
        steps = 0
        result = None
        
        try:
            while True:
                next(gen)
                steps += 1
        except StopIteration as e:
            result = e.value
        
        results[algo_key] = {
            'name': algo_name,
            'steps': steps,
            'path_length': len(result.path) if result and result.found else 0,
            'explored': result.visited_count if result else 0,
            'found': result.found if result else False
        }
    
    # Display results table
    print("\n{:<30} {:>10} {:>10} {:>12}".format(
        "Algorithm", "Path Len", "Explored", "Optimal?"))
    print("-" * 65)
    
    for algo_key in ['dfs', 'bfs', 'astar']:
        r = results[algo_key]
        optimal = "✓" if algo_key in ['bfs', 'astar'] else "✗"
        print("{:<30} {:>10} {:>10} {:>12}".format(
            r['name'],
            r['path_length'] if r['found'] else "N/A",
            r['explored'],
            optimal
        ))
    
    print("\nKey Observations:")
    print("-" * 40)
    print("• BFS and A* find the SHORTEST path (optimal)")
    print("• DFS may find a longer path (not optimal)")
    print("• A* typically explores FEWER cells than BFS")
    print("• DFS explores deep before wide (may be faster or slower)")
    
    print("\n" + "=" * 60)
    print("Run with UI for interactive exploration:")
    print("  python main.py --mode gui      # Graphical")
    print("  python main.py --mode terminal # Text-based")
    print("=" * 60)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Maze Pathfinding Visualizer - Learn Search Algorithms",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py                 # Default GUI mode
  python main.py --mode terminal # Terminal mode (no pygame needed)
  python main.py --demo          # Quick algorithm comparison demo
  
Controls (GUI mode):
  Left Click    - Place start → goal → walls
  Right Click   - Remove wall
  Click buttons - Run algorithms
  
Controls (Terminal mode):
  Arrow keys    - Move cursor
  w/s/g         - Wall/Start/Goal
  1/2/3         - DFS/BFS/A*
  r/c/q         - Reset/Clear/Quit
"""
    )
    
    parser.add_argument(
        '--mode', '-m',
        choices=['gui', 'terminal', 'auto'],
        default='auto',
        help='Visualization mode (default: auto-detect)'
    )
    
    parser.add_argument(
        '--demo', '-d',
        action='store_true',
        help='Run a quick algorithm comparison demo'
    )
    
    args = parser.parse_args()
    
    # Demo mode
    if args.demo:
        run_demo()
        return
    
    # Determine mode
    mode = args.mode
    if mode == 'auto':
        mode = 'gui' if check_pygame_available() else 'terminal'
        print(f"Auto-detected mode: {mode}")
    
    # Run selected mode
    if mode == 'gui':
        run_gui_mode()
    else:
        run_terminal_mode()


if __name__ == "__main__":
    main()
