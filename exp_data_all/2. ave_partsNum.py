import os
import pickle
from evaluation import eval_fitness
import matplotlib.pyplot as plt
import numpy as np
from pureples.shared.visualize import draw_net_3d
import glob

local_dir = os.path.dirname(__file__)

data_files = [glob.glob(local_dir+f"/genomes{i}.pkl")[0] for i in range(3,8)]
genome_data = []
for file in data_files:
  with open(file, "rb") as f:
    genome_data.append(pickle.load(f))

# fitness=[]
# Most_parts=[]
Average=[]
Best=[]
for i in range (5):
    total_fitness=[]
    parts={}
    best=[]
    for n,genome in enumerate(genome_data[i]):
        b=0
        gen_fitness=[]
        parts[n]=[]
        for ind,g in genome:
            parts[n].append((ind,len(g.substrate.output_coordinates)+1))
            gen_fitness.append(g.fitness)
            if b<g.fitness:
                b=g.fitness
                b_ind=ind
        best.append(b_ind)
        gen_fitness.sort()
        total_fitness.append(gen_fitness)
    # fitness.append(total_fitness)
    Ave_parts=[]
    Best_parts=[]
    for ind,p in parts.items():
        ave_parts=[]
        best_parts=[]
        # most_parts=[]
        ave_parts=np.mean(([q[1] for q in p]))
        best_parts=[q[1] for q in p if q[0]==best[ind]]
        # most_parts=sorted([(q[1],q[0]) for q in p])[-1]
        Ave_parts.append(ave_parts)
        Best_parts.append(*best_parts)
        # Most_parts.append(most_parts)
    Average.append(Ave_parts)
    Best.append(Best_parts)

ave=np.array(Average)
best=np.array(Best)

average_ave=np.average(ave, axis=0)
average_best=np.average(best, axis=0)


plt.figure()
plt.plot([gen for gen in range(len(genome_data[0]))],average_ave,[gen for gen in range(len(genome_data[0]))],average_best)
plt.legend(["Average","Best"])
plt.xlabel("Generation")
plt.ylabel("Block num")
plt.ylim(0,30)
plt.show()
plt.savefig(os.path.dirname(__file__) + "/partsnum.svg")

