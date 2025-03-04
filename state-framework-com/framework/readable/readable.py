from abc import ABCMeta, abstractmethod

class Readable(metaclass=ABCMeta):

    @abstractmethod
    def is_live_device(self) -> bool:
        pass

    @abstractmethod
    def bytes_avail(self) -> int:
        pass

    @abstractmethod
    def read(self, n: int) -> bytes:
        pass