from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self,name:str,age:int, genre:str) -> None:
        print("WriterNode init方法被调用")
        super().__init__(name, age)
        self.genre = genre
    
def main():
    writer = WriterNode("Bob",25,"Science Fiction")
    print(writer.eat("fish and chips"))