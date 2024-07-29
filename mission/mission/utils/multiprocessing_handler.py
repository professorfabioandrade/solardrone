
from multiprocessing import Process, Manager
from typing import Callable

class MultiprocessingHandler:

    def __init__(self, manager: Manager) -> None:
        self.manager_ = manager

    def parallel_float_values(self, num_val: int) -> tuple:
        return tuple(
            self.manager_.Value('d', 0.0) for _ in range(num_val)
            )
            
    def set_value(self, var, value: float) -> None:
        with self.local_data_lock:
            var.value = value

    def lock(self) -> None:
        self.local_data_lock = self.manager_.Lock()
    
    def run(self, target: Callable, args: tuple) -> None:
        self.p_ = Process(target=target, args=args)
        self.p_.start()