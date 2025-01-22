from abc import ABC, abstractmethod

class AbstractState(ABC):
    """
    Abstract class for all states
    """
    def __init__(self, robot):
        self.robot = robot

    @abstractmethod
    def execute(self):
        pass