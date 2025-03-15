# example.py

from NightEngine.NightBase import NightBase

class Example(NightBase):
    def setup(self):
        pass


if __name__ == "__main__":
    
    engine = Example()
    engine.run()
