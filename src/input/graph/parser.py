from pygraphml  import GraphMLParser
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parent.parent.parent
XML_DIR = "input/graph/TestTrack.graphml"

path = BASE_DIR / XML_DIR 

parser = GraphMLParser()

graph = parser.parse(path)

graph.show()