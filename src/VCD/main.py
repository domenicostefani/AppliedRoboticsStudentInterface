import argparse
import sys
from utils.configuration_space import configuration_space
from algorithms.VCD import VerticalCellDecomposition
from algorithms.RRT import RRT

def write_to_file(planner,path_idx,fname):
    last_key = planner.roadmap.vertices_dict.keys()[-1]
    with open(fname,'w+') as f:
        if path_idx != None:
	        for key in planner.roadmap.vertices_dict.keys():
		        if key != last_key:
			        f.write((str(planner.roadmap.vertices_dict[key][0])+","+str(planner.roadmap.vertices_dict[key][1])+","))
		        else:
			        f.write((str(planner.roadmap.vertices_dict[key][0])+","+str(planner.roadmap.vertices_dict[key][1])))
	        f.write("\n")
	
	        for i in range(0,len(path_idx)):
		        if i != len(path_idx)-1:
			        f.write(str(path_idx[i])+",")
		        else:
			        f.write(str(path_idx[i]))
        else:
            f.write("-1")          
        
    
if __name__ == "__main__":
	parser = argparse.ArgumentParser()

	parser.add_argument("-in",help="input file (default: input.txt)",default="input.txt")
	parser.add_argument("-out",help="output file (default: output.txt)",default="output.txt")
	parser.add_argument("-plot",help="plot final output? [y/n] (default: y)",default='y')
	parser.add_argument("-algo",help="algorithms: vcd, rrt",default="rrt")
	parser.add_argument("-n",help="number of samples for PRM/RRT (default: 1000)",default=1000)

	args = vars(parser.parse_args())

	cspace = configuration_space(args['in'])
	
	
	if args['algo'] == 'rrt':
		planner = RRT(cspace,args['n'])
		planner.perform_sampling(False)
		path, path_idx = planner.search(args['plot']=='y')

	elif args['algo'] == 'vcd':
		planner = VerticalCellDecomposition(cspace)
		planner.construct_graph()
		path, path_idx = planner.search(args['plot']=='y')

	write_to_file(planner,path_idx,args['out'])
