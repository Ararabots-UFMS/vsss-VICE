from strategy.behaviour import BaseTree, LeafNode, Selector

from strategy.blackboard import Blackboard
from strategy.coach.freekick import FreeKick
from strategy.coach.halt import Halt
from strategy.coach.kickoff import Kickoff
from strategy.coach.penalty import Penalty
from strategy.coach.stop import Stop
from strategy.coach.timeout import _Timeout

class CoachStrategy(Selector):
    def __init__(self, name):
        super().__init__(name, [])
        kickoff = Kickoff("Kickoff")
        freekick = FreeKick("FreeKick")
        stop = Stop("Stop")
        penalty = Penalty("Penalty")
        timeout = _Timeout("Timeout")
        halt = Halt("Halt")

        self.add_children([stop, halt, kickoff, freekick, penalty, timeout])

    def run(self):
        return super().run()
    

        
