import math
from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus
from strategy.coach.running.Defense_play import DefensivePlay
from strategy.robots.running.attacker import OurActionAttacker
from strategy.robots.running.defensive import OurActionDefender

class CheckOurDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.goal_position_x = 2250
        self.goal_position_y = 0
        self.position_x = self.blackboard.ally_robots[0].position_x
        self.position_y = self.blackboard.ally_robots[0].position_y
        self.radius = 500

    def run(self):

        distance = math.sqrt((self.position_x - self.goal_position_x) ** 2 + (self.position_y - self.goal_position_y) ** 2)

        if distance <= self.radius:
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None
        
        
class CheckTheirCondition(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

    def run(self):
        status = True
        if status:
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None
        


class IsTheirPossession(LeafNode):
    def __init__(self, name):
        self.name =name
        self.blackboard = Blackboard()
        self.points = DefensivePlay().run()
        self.commands = {}
    def run(self):
        for robot in self.blackboard.ally_robots:
            self.commands[robot] = OurActionDefender("Defend!!!", self.points[robot])

        return TaskStatus.SUCCESS, self.commands
        

class IsOurPossession(LeafNode):
    def __init__(self, name):
        self.name =name

    def run(self):    
        return TaskStatus.SUCCESS, OurActionAttacker("Attack!!!!")


class CheckStart(LeafNode):
    def __init__(self, name, commands):
        self.blackboard = Blackboard()
        self.name = name
        self.commands = commands

    def run(self):
        if self.blackboard.referee.command in self.commands:
            return TaskStatus.SUCCESS, None
        
        return TaskStatus.FAILURE, None


class Running(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        self.blackboard = Blackboard()
        
        commands = ["NORMAL_START", "FORCE_START"]

        is_running = CheckStart("CheckStart", commands)
        
        ours_action = IsOurPossession("IsOurPossession")

        ours_with_ball = CheckOurDistance("CheckOurDistance")

        theirs_action = IsTheirPossession("IsTheirPossession")

        theirs_with_ball = CheckTheirCondition("CheckTheirCondition")

        ours = Sequence("OursNormalStart", [ours_with_ball, ours_action])

        theirs = Sequence("TheirNormalStart", [theirs_with_ball, theirs_action])

        running_selector = Selector("RunningSelector", [theirs, ours])


        self.add_children([is_running, running_selector])

    def run(self):
        return super().run()


       
if __name__ == "__main__":
    root = Running("StartRoot")
    root.run()