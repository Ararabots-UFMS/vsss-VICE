from strategy.behaviour import LeafNode, Sequence, TaskStatus
from strategy.blackboard import Blackboard
from strategy.robots.halt.halt import GameHalted

class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        self.name = name
        self.blackboard = Blackboard()
        self.desired_states = _desired_states

    def run(self):
        if self.blackboard.referee.command in self.desired_states:
            return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None
    
class HaltAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.commands = {}

    def run(self):
        for robot in self.blackboard.ally_robots:
            self.commands[robot] = GameHalted()

        return TaskStatus.SUCCESS, self.commands
    
class Halt(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        
        """ List with possible inputs to this state """
        commands = ["HALT"]
        check_halt = CheckState("CheckHalt", commands)

        halt_action = HaltAction("HaltAction")
        
        
        self.add_children([check_halt, halt_action])
        
    def run(self):
        """Access the second element in tuple"""
        return super().run()

if __name__ == "__main__":
    halt = Halt("Halt")
    print(halt.run()[1])