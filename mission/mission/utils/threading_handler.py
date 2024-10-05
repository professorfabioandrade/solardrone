from threading import Thread, Lock
from typing import Callable

class ThreadingHandler:

    def __init__(self) -> None:
        self.lock_ = Lock()

    def parallel_float_values(self, num_val: int) -> tuple:
        # Using lists as mutable containers for shared float values
        return tuple([0.0] for _ in range(num_val))

    def set_value(self, var, value: float) -> None:
        with self.lock_:
            var[0] = value

    def lock(self) -> None:
        self.local_data_lock = self.lock_

    def run(self, target: Callable, args: tuple) -> None:
        self.t_ = Thread(target=target, args=args)
        self.t_.start()

    def join(self) -> None:
        self.t_.join()

    def __del__(self) -> None:
        try:
            if self.t_:
                if self.t_.is_alive():
                    self.t_.join()
        except:
            pass
