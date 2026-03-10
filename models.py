from dataclasses import dataclass
from typing import List
from enum import Enum

class Direction(Enum):
    """Enumeration of possible movement directions in the grid."""
    UP = (0, -1)    # dx, dy for moving up
    LEFT = (-1, 0)  # dx, dy for moving left
    DOWN = (0, 1)   # dx, dy for moving down
    RIGHT = (1, 0)  # dx, dy for moving right
    
@dataclass
class Position:
    """Represents a position in the 2D grid."""
    x: int
    y: int
    
    def __add__(self, other):
        """Add a direction tuple to the current position."""
        return Position(self.x + other[0], self.y + other[1])

    def as_tuple(self):
        """Convert the position to a tuple."""
        return (self.x, self.y)
    
    def __eq__(self, other):
        """Check if two positions are equals."""
        # Ensure other is also a Position before comparing x and y
        return isinstance(other, Position) and self.x == other.x and self.y == other.y
    
    def __hash__(self):
        """Generate a hash for the position."""
        # Use a tuple of x and y for hashing
        return hash((self.x, self.y))

class Wall:
    """Represents a wall in the grid."""
    def __init__(self, x: int, y: int, width: int, height: int):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        
    def contains(self, position: Position):
        """Check if the wall contains a given position."""
        # Check if the position is within the wall's boundaries
        return (self.x <= position.x < self.x + self.width and
                self.y <= position.y < self.y + self.height)


class RobotWorld:
    """Represents the entire robot world, including the grid, start position, goals, and walls."""
    def __init__(self, rows: int, cols: int, start: Position, goals: List[Position], walls: List[Wall]):
        self.rows = rows
        self.cols = cols
        self.start = start
        self.goals = goals
        self.walls = walls
        
    def is_valid_position(self, position: Position):
        """Check if a given position is valid (within bounds and not in a wall)."""
        # Check if the position is within the grid boundaries
        if position.x < 0 or position.x >= self.cols:
            return False
        if position.y < 0 or position.y >= self.rows:
            return False
        # Check if the position is not inside any wall
        return not any(wall.contains(position) for wall in self.walls)
    
    def get_neighbors(self, position: Position):
        """Get all valid neighboring positions for a given position."""
        neighbors = []
        for direction in Direction:
            dx, dy = direction.value
            new_pos = Position(position.x + dx, position.y + dy)
            # Only add the new position if it's valid
            if self.is_valid_position(new_pos):
                neighbors.append(new_pos)
        return neighbors
            
            
    @classmethod
    def from_file(cls, filename: str):
        """Create a RobotWorld instance by reading from a file."""
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
                
                if not lines:
                    raise ValueError("The file is empty.")
                
                # Parse grid size from the first line
                rows, cols = eval(lines[0])
                
                # Parse start position from the second line
                start_x, start_y = eval(lines[1])
                start = Position(start_x, start_y)
                
                # Parse goal positions from the thrid line
                goals = []
                for goal_str in lines[2].split('|'):
                    goal_x, goal_y = eval(goal_str.strip())
                    goals.append(Position(goal_x, goal_y))
                    
                # Parse walls from the remaining lines
                walls = []
                for line in lines[3:]:
                    if line.strip():
                        x, y, w, h = eval(line.strip())
                        walls.append(Wall(x, y, w, h))
                
                # Create and return a new RobotWorld instance
                return cls(rows, cols, start, goals, walls)
        except FileNotFoundError:
            print(f'Error: File "{filename}" not found.')
            return None
        except ValueError as e:
            print(f"Error: {str(e)}")
            return None
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}")
            return None
            