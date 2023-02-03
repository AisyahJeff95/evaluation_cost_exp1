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

#find total fitness
fitness=[]
best_each=[]
for i in range (5):
    total_fitness=[]
    for genome in genome_data[i]:
        gen_fitness=[]
        for ind,g in genome:
            gen_fitness.append(g.fitness)
        
        gen_fitness.sort()
        total_fitness.append(gen_fitness)
   
    fitness.append(total_fitness)
    best_each.append([n[-1] for n in total_fitness])

#average
data = np.array(fitness)
average = np.average(data, axis=0) #the average of all fitness
best=np.array(best_each)
best_average=np.average(best, axis=0)

#plot it
plt.figure()
plt.plot(np.arange(len(genome_data[0])),[np.mean(n) for n in average], label="Average")
plt.plot(np.arange(len(genome_data[0])),best_average, label="Best")
# plt.plot(np.arange(len(genome_data[0])),[best], label="Trials")
# plt.plot(np.arange(len(genome_data[0])),[n[-1] for n in total_fitness[1,8]], label="Trials")

plt.legend(loc="upper right")
plt.xlabel("Generation")
plt.ylabel("Fitness")
plt.ylim(0,65)
plt.savefig('ave_fitness_case_.svg')
plt.show()