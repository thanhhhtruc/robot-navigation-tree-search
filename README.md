# Robot Navigation with Tree Search Algorithms

A Python-based robot navigation system that implements various search algorithms to solve single-goal and multi-goal pathfinding problems in a 2D grid world with obstacles.

## 📋 Overview

This project provides a comprehensive implementation of classic AI search algorithms for robot navigation. The system can find optimal paths from a starting position to one or multiple goal positions while avoiding walls and obstacles. It includes both a command-line interface (CLI) and a graphical user interface (GUI) for visualization.

## ✨ Features

- **Multiple Search Algorithms**:
  - **BFS** (Breadth-First Search) - Guarantees shortest path
  - **DFS** (Depth-First Search) - Memory efficient exploration
  - **GBFS** (Greedy Best-First Search) - Heuristic-driven search
  - **A*** (A-Star) - Optimal pathfinding with heuristics
  - **CUS1** (Bidirectional BFS) - Simultaneous forward/backward search
  - **CUS2** (Bidirectional A*) - Optimal bidirectional search

- **Multi-Goal Search**: Extended versions of all algorithms to find paths to multiple goals sequentially

- **Interactive GUI**: 
  - Visual representation of the grid world
  - Real-time search visualization
  - Path highlighting with different colors
  - Zoom controls for better viewing
  - Toggle visibility for multiple paths

- **CLI Mode**: Run algorithms from command line with detailed output

## 🚀 Getting Started

### Prerequisites

- Python 3.7 or higher
- tkinter (usually comes with Python)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/thanhhhtruc/robot-navigation-tree-search.git
cd robot-navigation-tree-search
```

2. No additional dependencies required - uses only Python standard library!

### Usage

#### GUI Mode

Run the application without arguments to launch the graphical interface:

```bash
python main.py
```

Steps in GUI:
1. Click "Load Problem" to select a test case file
2. Choose an algorithm from either "Single Goal Search" or "Multi Goal Search"
3. Click "Solve" or "Solve All Goals" to run the algorithm
4. View the visualization and path details in the output panel

#### CLI Mode

Run with a test case file and algorithm name:

```bash
python main.py <filename> <algorithm>
```

Examples:

```bash
# Single-goal search
python main.py TestCase1.txt bfs
python main.py TestCase2.txt astar
python main.py TestCase3.txt cus1

# Multi-goal search
python main.py TestCase4.txt multi-bfs
python main.py TestCase5.txt multi-astar
python main.py TestCase6.txt multi-cus2
```

**Available algorithm names:**
- Single-goal: `bfs`, `dfs`, `gbfs`, `astar`, `cus1`, `cus2`
- Multi-goal: `multi-bfs`, `multi-dfs`, `multi-gbfs`, `multi-astar`, `multi-cus1`, `multi-cus2`

## 📁 File Format

Test case files should follow this format:

```
[rows,cols]
(start_x,start_y)
(goal1_x,goal1_y) | (goal2_x,goal2_y) | ...
(wall1_x,wall1_y,width,height)
(wall2_x,wall2_y,width,height)
...
```

Example (`TestCase1.txt`):
```
[5,7]
(1,1)
(3,4) | (4,4)
(2,1,1,6)
(1,2,6,1)
(3,3,5,1)
```

This creates:
- 5 rows × 7 columns grid
- Start position at (1,1)
- Two goals at (3,4) and (4,4)
- Three rectangular walls

## 🏗️ Project Structure

```
robot-navigation-tree-search/
├── main.py                    # Entry point and CLI interface
├── models.py                  # Core data structures (Position, Wall, RobotWorld)
├── single_goal_search.py      # Single-goal search algorithms
├── multi_goals_search.py      # Multi-goal search algorithms
├── gui.py                     # Tkinter-based GUI application
├── TestCase1.txt - TestCase10.txt  # Sample test cases
└── README.md                  # This file
```

## 🔍 Algorithm Details

### Single-Goal Algorithms

| Algorithm | Completeness | Optimality | Time Complexity | Space Complexity |
|-----------|-------------|------------|-----------------|------------------|
| BFS       | Yes         | Yes        | O(b^d)          | O(b^d)           |
| DFS       | Yes*        | No         | O(b^m)          | O(bm)            |
| GBFS      | No          | No         | O(b^m)          | O(b^m)           |
| A*        | Yes         | Yes**      | O(b^d)          | O(b^d)           |
| CUS1      | Yes         | Yes        | O(b^(d/2))      | O(b^(d/2))       |
| CUS2      | Yes         | Yes**      | O(b^(d/2))      | O(b^(d/2))       |

*In finite state spaces  
**With admissible heuristic (Manhattan distance used)

### Custom Algorithms

- **CUS1 (Bidirectional BFS)**: Runs BFS simultaneously from start and goal(s), meeting in the middle for faster convergence
- **CUS2 (Bidirectional A*)**: Combines A* with bidirectional search for optimal performance with heuristics

### Multi-Goal Approach

Multi-goal algorithms find paths to all goals by:
1. Finding the nearest reachable goal from current position
2. Moving to that goal
3. Repeating until all goals are reached or no more goals are reachable

## 📊 Output Format

### CLI Output

```
TestCase1.txt BFS
<Node (3, 4)> 15
['right', 'right', 'down', 'down', 'down']
```

- Line 1: Filename and algorithm
- Line 2: Goal coordinates and nodes created
- Line 3: Path directions

### Multi-Goal CLI Output

```
TestCase4.txt MULTI-BFS
Found 2 paths! Nodes created: 28

Path 1 to Goal 1: <Node (3, 4)>
['right', 'right', 'down', 'down', 'down']

Path 2 to Goal 2: <Node (4, 4)>
['right']
```

## 🎨 GUI Features

- **Grid Visualization**: 
  - White cells: Open space
  - Gray cells: Walls
  - Red cell: Start position
  - Green cells: Goals
  - Light blue: Visited cells during search
  - Colored cells: Path(s) found

- **Real-time Search Visualization**: Watch algorithms explore the grid step-by-step

- **Multi-Path Visualization**: 
  - Different colors for different paths
  - Numbered markers show goal reaching order
  - Toggle individual paths on/off

- **Zoom Controls**: Adjust grid size for better viewing
