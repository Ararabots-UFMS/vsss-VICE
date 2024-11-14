from strategy.blackboard import SingletonMeta


class LastCommand(metaclass=SingletonMeta):
    def __init__(self):
        self.last_comand = None

    def set_command(self, new_command):
        self.last_comand = new_command

    def get_command(self):
        return self.last_comand