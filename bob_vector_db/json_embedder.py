from abc import ABC, abstractmethod

class JsonEmbedder(ABC):
    """Abstract JSON embedding interface with optional media input
    Override the __call__ function in derived class to use it.
    """
    @abstractmethod
    def __call__(self, json: str, media: list=None) -> None:
        raise NotImplementedError()
