
import math
from strategy.behaviour import LeafNode, Selector, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy
from strategy.coach.running.utils import DefenseTriangle, PenaltyArea

class GetPoints():
    def __init__(self):
        self.blackboard = Blackboard()
        self.triangle = DefenseTriangle()
        self.penalty_area = PenaltyArea()
        self.points_to_defend = []

    def run(self):
        self.find_points()
        return self.points_to_defend
    
    def find_intersection(self, m1, n1, m2, n2):
        # Check if lines are parallel
        if m1 == m2:
            return None  # No intersection if lines are parallel
        
        # Calculate intersection points
        x = (n2 - n1) / (m1 - m2)
        y = m1 * x + n1
        return x, y
    
    def find_points(self):
        # Get triangle and penalty area line equations
        m_triangle_left, n_triangle_left = self.triangle.draw_line_left()
        m_triangle_right, n_triangle_right = self.triangle.draw_line_right()
        
        m_penalty_left, n_penalty_left = self.penalty_area.draw_line_left()
        m_penalty_right, n_penalty_right = self.penalty_area.draw_line_rigth()
        x_front = self.penalty_area.draw_line_front()  # x constant for the front line

        # Find intersection between left lines
        left_intersection_line2 = self.find_intersection(m_triangle_left, n_triangle_left, m_penalty_left, n_penalty_left)
        if left_intersection_line2:
            # Round values to integers
            rounded_point = (round(left_intersection_line2[0]), round(left_intersection_line2[1]))
            self.points_to_defend.append(rounded_point)
        
        left_intersection_line1 = self.find_intersection(m_triangle_right, n_triangle_right, m_penalty_left, n_penalty_left)
        if left_intersection_line1:
            rounded_point = (round(left_intersection_line1[0]), round(left_intersection_line1[1]))
            self.points_to_defend.append(rounded_point)
        
        # Find intersection between right lines
        right_intersection_line1 = self.find_intersection(m_triangle_right, n_triangle_right, m_penalty_right, n_penalty_right)
        if right_intersection_line1:
            rounded_point = (round(right_intersection_line1[0]), round(right_intersection_line1[1]))
            self.points_to_defend.append(rounded_point)

        right_intersection_line2 = self.find_intersection(m_triangle_left, n_triangle_left, m_penalty_right, n_penalty_right)
        if right_intersection_line2:
            rounded_point = (round(right_intersection_line2[0]), round(right_intersection_line2[1]))
            self.points_to_defend.append(rounded_point)
        
        # Find intersection with the front line (vertical line)
        # Front line has constant x = x_front, so we use triangle lines to find y at this x.
        
        # Intersection of front line with left triangle line
        if m_triangle_left != 0:  # Avoid division by zero in case of horizontal line
            y_left_front = m_triangle_left * x_front + n_triangle_left
            rounded_point = (round(x_front), round(y_left_front))
            self.points_to_defend.append(rounded_point)
        
        # Intersection of front line with right triangle line
        if m_triangle_right != 0:  # Avoid division by zero in case of horizontal line
            y_right_front = m_triangle_right * x_front + n_triangle_right
            rounded_point = (round(x_front), round(y_right_front))
            self.points_to_defend.append(rounded_point)
        
        self.remove_invalid_points()
        print(self.points_to_defend)


    def remove_invalid_points(self):
        for point in self.points_to_defend[:]:
            if self.blackboard.gui.is_field_side_left:
                if point[0] < -2250 or point[0] > -1750 or abs(point[1]) > 675:
                    print(f"removendo em 1: {point}")
                    self.points_to_defend.remove(point) 
            else:
                if point[0] > 2250 or point[0] < 1750 or abs(point[1]) > 675:
                    print(f"removendo em 2: {point}")
                    self.points_to_defend.remove(point)                 


class DefensivePlay():
    def __init__(self):
        self.blackboard = Blackboard()
        self.points = []
        self.distances = {}
        self.assignments = {}

    def run(self):
        self.points = GetPoints().run()
        self.distance_p2r()
        self.calculate_wanted_points()
        self.distribute_points()
        return self.assignments
    
    """Calculate the points outside the penalty area line that the robots need to go"""
    def calculate_wanted_points(self):
        adjusted_points = []
        self.padding = 100

        print(self.points)
        # New approach: adjust points outside the penalty area line
        left_point = list(self.points[0])  # convert to list to allow modification
        right_point = list(self.points[1])

        self.mult = -1
        # Horizontal alignment
        if self.blackboard.gui.is_field_side_left:
            self.mult = 1


        if left_point[0] == right_point[0]:  
            left_point[0] += self.padding*self.mult
            right_point[0] += self.padding*self.mult
            if self.blackboard.gui.is_field_side_left:
                if left_point[1] > right_point[1]:
                    left_point[1] -= self.padding*self.mult
                    right_point[1] += self.padding*self.mult
                else:
                    left_point[1] += self.padding*self.mult
                    right_point[1] -= self.padding*self.mult
            else:
                if left_point[1] > right_point[1]:
                    left_point[1] += self.padding*self.mult
                    right_point[1] -= self.padding*self.mult
                else:
                    left_point[1] -= self.padding*self.mult
                    right_point[1] += self.padding*self.mult

            adjusted_points.append(tuple(left_point))
            adjusted_points.append(tuple(right_point))

        
        # Vertical alignment
        elif left_point[1] == right_point[1]:  
            if left_point[1] > 0:
                left_point[1] += self.padding*self.mult
                right_point[1] += self.padding*self.mult
            else:
                left_point[1] -= self.padding*self.mult
                right_point[1] -= self.padding*self.mult
            if self.blackboard.gui.is_field_side_left:
                if left_point[0] > right_point[0]:
                    left_point[0] -= self.padding*self.mult
                    right_point[0] += self.padding*self.mult
                else:
                    left_point[0] += self.padding*self.mult
                    right_point[0] -= self.padding*self.mult
            else:
                if left_point[0] > right_point[0]:
                    left_point[0] += self.padding*self.mult
                    right_point[0] -= self.padding*self.mult
                else:
                    left_point[0] -= self.padding*self.mult
                    right_point[0] += self.padding*self.mult
            adjusted_points.append(tuple(left_point))
            adjusted_points.append(tuple(right_point))

        
        # Quadrant-based positioning
        else:
            one_point = [0, 0]
            if self.blackboard.balls[0].position_y > 0 and self.blackboard.balls[0].position_x < 0:
                one_point[0] = -1750 + self.padding
                one_point[1] = +675 + self.padding
            elif self.blackboard.balls[0].position_y < 0 and self.blackboard.balls[0].position_x > 0:
                one_point[0] = +1750 - self.padding
                one_point[1] = -675 - self.padding
            elif self.blackboard.balls[0].position_y > 0 and self.blackboard.balls[0].position_x > 0:
                one_point[0] = +1750 - self.padding
                one_point[1] = +675 + self.padding
            elif self.blackboard.balls[0].position_y < 0 and self.blackboard.balls[0].position_x < 0:
                one_point[0] = -1750 + self.padding
                one_point[1] = -675 - self.padding
            adjusted_points.append(tuple(one_point))

        self.points = adjusted_points
        print(self.points)

    """Calculate the distance between a point and a robot"""
    def distance_p2r(self):
        for robot in self.blackboard.ally_robots:
            if robot != self.blackboard.referee.teams[self.blackboard.gui.is_team_color_yellow].goalkeeper:
                self.distances[robot] = []
                for point in self.points:
                    distance = math.sqrt((point[0] - self.blackboard.ally_robots[robot].position_x) ** 2 + (point[1] - self.blackboard.ally_robots[robot].position_y) ** 2)
                    self.distances[robot].append(distance)
        
    """Distribute the position to each defender choosing the most close to point"""
    def distribute_points(self):
        assigned_points = set()  # Track points already assigned
        self.assignments = {}  # Reset assignments for each robot

        # Sort robots by their minimum distance to points
        sorted_robots = sorted(
            ((robot, distances) for robot, distances in self.distances.items() if distances),
            key=lambda item: min(item[1]) if item[1] else float('inf')
        )

        # Assign each robot to the closest available point
        for robot, distances in sorted_robots:
            closest_point = None
            min_distance = float('inf')

            # Find the closest unassigned point for this robot
            for i, distance in enumerate(distances):
                # Ensure index i is within the range of self.points
                if distance < min_distance and i < len(self.points) and i not in assigned_points:
                    min_distance = distance
                    closest_point = i

            # Assign the closest available point if found
            if closest_point is not None:
                self.assignments[robot] = self.points[closest_point]
                assigned_points.add(closest_point)  # Mark this point as assigned

                


        
            


