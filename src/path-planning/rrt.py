# Find a path avoiding obstacles using modified RRT
# Author -- Shikhar Dev Gupta

from random import *
from helpers.graph import *
from helpers.geometry import *
import matplotlib.pyplot as plt
import argparse

SHOW_PLOT = False;
PRINT_INFO = False;

parser = argparse.ArgumentParser()

parser.add_argument("-in",help="input file (default: input.txt)",default="input.txt")
parser.add_argument("-out",help="output file (default: output.txt)",default="output.txt")

args = vars(parser.parse_args())

# Check for empty lines
file_handler = open(args['in'],"r");
raw_data = file_handler.read();
raw_data = raw_data.split("\n");
step_size = 5;
bias_prob = 0.05;

if(len(raw_data) <2):
	print("Incorrect format of the input file");
	exit;

def parse_input_line(line):
	temp2 = [];
	line = [i.strip() for i in line.split(",")];
	vertex = [];
	for index,i in enumerate(line):
		if(i[0] == "("):
			i = i[1:];
		if(i[len(i)-1] == ")"):
			i= i[:-1];
		vertex.append(int(i));
		if(index%2 != 0):
			temp2.append(vertex);
			vertex = [];
	return temp2;


# Draw the obstacles and point the source and the destination----------------------------------------------
def draw_problem():
	bnd_x = [i.x for i in boundary];
	bnd_x.append(boundary[0].x);
	bnd_y = [i.y for i in boundary];
	bnd_y.append(boundary[0].y);
	poly_x = [];
	poly_y = []

	# Draw the boundary
	plt.plot(bnd_x, bnd_y);

	for index, i in enumerate(obstacles):
		poly_x.append([p[0] for p in i]);
		poly_y.append([p[1] for p in i]);

		plt.fill( poly_x[index], poly_y[index], color="#512DA8");

	plt.plot(source.x, source.y, marker="o");
	plt.plot(dest.x, dest.y, marker="o");
	plt.annotate('Source', xy=(source.x, source.y), xytext=(source.x+5, source.y-6) );
	plt.annotate('Destination', xy=(dest.x, dest.y), xytext=(dest.x-4, dest.y-10) );


# Choose goal with some probability--------------------------------------------
def choose_target():
	if(uniform(0,1) < bias_prob):
		return dest;
	else:
		return random_point([boundary[0].x, boundary[1].x], [boundary[0].y, boundary[2].y]);


# Extract vertices---------------------------------------------------
temp = parse_input_line(raw_data[0]);
boundary = [point(i[0], i[1]) for i in temp];

# Extract source and dest
temp = parse_input_line(raw_data[len(raw_data)-1]);
source = point(temp[0][0], temp[0][1]);
dest = point(temp[1][0], temp[1][1]);

# Extract obstacles
temp_obstacles = [];
for i in raw_data[1:len(raw_data)-1]:
	temp_obstacles.append(parse_input_line(i) );

obstacles = [];
for index, i in enumerate(temp_obstacles):
	temp_obs = [];
	for j in i:
		temp = point(j[0], j[1], index);
		temp_obs.append(temp);
	obstacles.append(temp_obs);

#-----------------------------------------------------------
graph_vertices = [];
graph = [];

#print dest;
#print ([boundary[0].x, boundary[1].x], [boundary[0].y, boundary[2].y]);

# Add source to vertices
graph_vertices.append(source);
graph.append([]);
found = False;

# Run the algorithm ! ---------------------------------------------
# Find the path and save in a graph
while(found is False):

	if( len(graph_vertices)>=3000 ):
		print "RRT: iteration limit reached";
		break;
	graph.append([]);

	potential_next_vertex = choose_target();
	if(potential_next_vertex.inside_polygon(obstacles) is True ):
		graph = graph[:-1];
		continue;

	vertex_index = potential_next_vertex.find_closest_point(graph_vertices);

	if(graph_vertices[vertex_index].equals(potential_next_vertex)):
		graph = graph[:-1];
		continue;

	# temp_vertex = find_point_on_line(graph_vertices[vertex_index], potential_next_vertex, step_size);
	temp_vertex = step_from_to(graph_vertices[vertex_index], potential_next_vertex, step_size);

	if(temp_vertex.inside_polygon(obstacles) is True ):

		graph = graph[:-1];
		continue;
	else:
		potential_next_vertex = temp_vertex;
	 	if(check_obstruction(obstacles, [potential_next_vertex, graph_vertices[vertex_index]]) is False):
	 		graph = graph[:-1];
			continue;
		if(check_obstruction(obstacles,[dest, potential_next_vertex]) is True):
			found = True;
			if(PRINT_INFO):
				print("Reached..!!");
			graph_vertices.append(potential_next_vertex);
			n = len(graph_vertices)-1;

			graph[vertex_index].append(n);
			graph[n].append(vertex_index);

			graph.append([]);
			graph_vertices.append(dest);
			n = len(graph_vertices)-1;

			graph[n-1].append(n);
			graph[n].append(n-1);
			break;
		else:
			graph_vertices.append(potential_next_vertex);
			n = len(graph_vertices)-1;

			graph[vertex_index].append(n);
			graph[n].append(vertex_index);


poly_x =[]
poly_y =[]
for index, i in enumerate(obstacles):
	poly_x.append([p.x for p in i]);
	poly_y.append([p.y for p in i]);

	plt.fill( poly_x[index], poly_y[index], color="#512DA8");

for index,i in enumerate(graph):
	for j in i:
		plt.plot([graph_vertices[index].x, graph_vertices[j].x], [graph_vertices[index].y, graph_vertices[j].y]);

# Find the path by running BFS on the graph
path = bfs(graph, 0, len(graph_vertices)-1);

x_array = [graph_vertices[i].x for i in path];
y_array = [graph_vertices[i].y for i in path];
plt.plot(x_array, y_array, color="#000000", linewidth=2);
plt.plot(graph_vertices[0].x, graph_vertices[0].y, marker="o")
plt.plot(graph_vertices[-1].x, graph_vertices[-1].y, marker="o")

if SHOW_PLOT:
	plt.show();

#----------------------------------------------------
# output into a file
file_output = open(args['out'], "w" );
str_to_write = "";

#for index in range(len(graph_vertices)):
#    if index == len(graph_vertices)-1:
#	    str_to_write = str_to_write + str(int(graph_vertices[index].x) )+  ","+ str(int(graph_vertices[index].y) )
#    else:
#        str_to_write = str_to_write + str(int(graph_vertices[index].x) )+  ","+ str(int(graph_vertices[index].y) ) + ","

#str_to_write = str_to_write[1:];

#total_write = str_to_write+"\n";
#str_to_write="";

#for i in graph:
#	str_to_write = str_to_write + ",(";
#	for j in i:
#		str_to_write = str_to_write + str(j) + ",";
#	str_to_write = str_to_write[:-1];
#	str_to_write = str_to_write + ")";

#str_to_write = str_to_write[1:];

#total_write = total_write+ str_to_write + "\n";

#str_to_write = "";
#str_to_write =','.join(str(x) for x in path);

for index in range(len(path)):
	if index == len(path)-1:
		str_to_write = str_to_write + str(int(graph_vertices[path[index]].x) )+  "," + str(int(graph_vertices[path[index]].y) )
	else:
		str_to_write = str_to_write + str(int(graph_vertices[path[index]].x) )+  "," + str(int(graph_vertices[path[index]].y) ) + "\n"


#total_write = total_write + str_to_write;

if PRINT_INFO:
	print "Output written to file.. Drawing the result";

#file_output.write(total_write);
file_output.write(str_to_write);
