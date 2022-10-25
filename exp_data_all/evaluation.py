from creature.creature import create
import neat
from pureples.shared.substrate import Substrate
from pureples.shared.genome import CppnGenome
from pureples.hyperneat import create_phenotype_network
import pybullet as p
import pybullet_data
import numpy as np
import os
import pickle
from tqdm import tqdm
import copy
import glob
import creature.A_visualize as visualize
import random
from creature.create_data_dir import create_data_dir

comment=None

config_path = os.path.dirname(__file__)+'/A_config_cppn_N100_M20_def'
if __name__ == '__main__':
  config = neat.config.Config(CppnGenome, neat.reproduction.DefaultReproduction,
                              neat.species.DefaultSpeciesSet, neat.stagnation.DefaultStagnation,
                              config_path)

sphereRadius=0.3 # 仮想生物の長方形の短軸長
defaultPatrsNum=1 #初期状態のパーツ数
maxPartsNum=10 #パーツ数がこれ以上ならば発生イベントは生じない
growRate=50 # 成長の刻み幅 g.growstep=growRate ならそのパーツの成長は終了
growInterval=20 # 成長が起こるまでのインターバル
EventNum=4 # 発生イベントの回数 個体の持つジョイント数がPybulletで指定された閾値(127)を超えるとエラーを吐く
pop_radius=60
Total_Step=20000
save_interval=2

Genomes=[]

def eval_fitness(genomes,config=None,mode="DIRECT",camera=None):
  if p.isConnected()==0:
    if mode=="DIRECT": 
      p.connect(p.DIRECT)
    else:
      p.connect(p.GUI)
      p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
      p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
      p.resetDebugVisualizerCamera(cameraDistance=60, cameraYaw=0, cameraPitch=-80, cameraTargetPosition=[0,-10,0])
      # p.resetDebugVisualizerCamera(cameraDistance=30, cameraYaw=0, cameraPitch=-50, cameraTargetPosition=[60,0,0])
    p.setTimeStep(1/120)
    
  os.chdir(os.path.dirname(__file__)+"/creature")
  p.loadURDF('plane.urdf',[0,0,-50])
  p.setAdditionalSearchPath(pybullet_data.getDataPath())

  cube=p.loadURDF("cube.urdf",[0,0,2.5],globalScaling=5)
  p.setGravity(0, 0, -10)
  p.setRealTimeSimulation(0)
  p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)

  radar_matrix=[[8*sphereRadius*np.cos(2*np.pi*i/8),8*sphereRadius*np.sin(2*np.pi*i/8),0] for i in range(8)]
  
  if __name__ == '__main__':
    pos_list=[i for i in range(len(genomes))]
    for _,g in genomes:
      g.pos=random.choice(pos_list)   
      pos_list.remove(g.pos)   

  for _,g in genomes:
    g.creature=create() 
    g.creature.create_base(sphereRadius,[pop_radius*np.cos(2*np.pi*g.pos/len(genomes)),pop_radius*np.sin(2*np.pi*g.pos/len(genomes)),sphereRadius],p.getQuaternionFromEuler([0,0,2*np.pi*g.pos/len(genomes)]),defaultPatrsNum)
    input_coordinates  = [(g.creature.input_coordinate[i]) for i in g.creature.input_coordinate]+radar_matrix
    hidden_coordinates = [[(i, 0.0, 1) for i in range(-4,5)]]
    output_coordinates = [(g.creature.output_coordinate[i]) for i in g.creature.output_coordinate]
    g.position=[p.getBasePositionAndOrientation(g.creature.bodyId)[0]]
    if __name__ == '__main__':
      g.cppn=neat.nn.FeedForwardNetwork.create(g, config)
    g.substrate=Substrate(input_coordinates, output_coordinates, hidden_coordinates)
    g.net=create_phenotype_network(g.cppn, g.substrate,"sin")

    g.partsNum=[(0,defaultPatrsNum)] #(step,partsNum)

  growFlag=[False]*len(genomes)
  growstep=[0]*len(genomes)
  EventOppotunity=[0]*len(genomes)

  for step in tqdm(range(Total_Step)):
    for i,(ind,g) in enumerate(genomes):
      if np.mod(step,int(Total_Step/4))==0:
        growFlag[i]=True
      if np.mod(step,growInterval)==0 and EventOppotunity[i]<EventNum and growFlag[i]==True:
        growstep[i]+=1
        
        if growstep[i]==1:
          g.outputs=[]
          if p.getNumJoints(g.creature.bodyId)<maxPartsNum*2:
            input=list(g.creature.input_coordinate.keys())
            for n in input:
              for m in [0,1,2]:
                if len(g.creature.jointGrobalPosition[n][m])!=0:
                  cppn_output=g.cppn.activate(7*[0]+[step/Total_Step]+list(g.creature.input_coordinate[n])+list(g.creature.jointGrobalPosition[n][m]))
                  cppn_addORnot=cppn_output[1]
                  cppn_scale=4 if cppn_output[2]<4 else cppn_output[2] if 4<cppn_output[2] and cppn_output[2]<8 else 8
                  cppn_jointType=p.JOINT_REVOLUTE# if cppn_output[3]>=0 else p.JOINT_FIXED
                  cppn_orientation=p.getQuaternionFromEuler(cppn_output[4:])
                  cppn_linkParentInd=n
                  cppn_linkPositions=g.creature.jointLinkPosition[n][m]
                  g.creature.jointGrobalPosition[n][m]=[]
                  if cppn_addORnot>0:
                    g.outputs.append([cppn_scale,cppn_jointType,cppn_linkPositions,cppn_orientation,cppn_linkParentInd])
        if g.outputs!=[]:
          # 成長イベント
          g.creature.bodyId=g.creature.grow(growstep[i],growRate,g.outputs)

        if growstep[i]==growRate:
          EventOppotunity[i]+=1
          growFlag[i]=False
          growstep[i]=0
          g.substrate.input_coordinates=[(g.creature.input_coordinate[i]) for i in g.creature.input_coordinate]+radar_matrix
          g.substrate.output_coordinates=[(g.creature.output_coordinate[i]) for i in g.creature.output_coordinate]
          g.net=create_phenotype_network(g.cppn, g.substrate,"sin")
          g.partsNum.append((step,len(g.substrate.output_coordinates)+1))
          

      if __name__!="__main__":
        g.position.append(p.getBasePositionAndOrientation(g.creature.bodyId)[0])

      if type(camera)==int:
        if ind==camera:
          p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-20, cameraTargetPosition=p.getBasePositionAndOrientation(g.creature.bodyId)[0])

      radar=g.creature.radar(cube)

      jointlist=[i-1 for i, key in g.creature.identification.items() if key == 'joint']
      if len(jointlist)==0:
        targetPositions=g.net.activate([step/Total_Step]+radar)
        forces=[0]
      else:
        joint_angle=[i[0] for i in p.getJointStates(g.creature.bodyId,jointlist)]
        targetPositions=np.array(g.net.activate([step/Total_Step]+joint_angle+radar))
        targetPositions[np.abs(targetPositions)<0.15]=0
        mass=[g.creature.linkmasses[i+1] for i in jointlist]
        forces=1.5/np.array(mass)


      p.setJointMotorControlArray(g.creature.bodyId,
                                  jointlist,
                                  p.POSITION_CONTROL,
                                  targetPositions=targetPositions,
                                  forces=forces)

    p.stepSimulation()

  if __name__ == '__main__':
    # if np.mod(step,1000)==0:
    #   p.removeAllUserDebugItems()
    for _,g in genomes:
      # isMove=np.sqrt((p.getBasePositionAndOrientation(g.creature.bodyId)[0][0]-g.prePosition[0])**2+(p.getBasePositionAndOrientation(g.creature.bodyId)[0][1]-g.prePosition[1])**2)
      try:
        g.fitness=pop_radius-p.getClosestPoints(g.creature.bodyId,cube,pop_radius)[0][8]
      except:
        g.fitness=0

  if __name__ == '__main__':
    g.creature, g.outputs=None, None
    Genomes.append(copy.deepcopy(genomes))
    if len(Genomes)==save_interval:
      global data_dir
      data_dir=create_data_dir(os.path.dirname(__file__),os.path.abspath(__file__),config_path,os.path.basename(__file__),comment)
    if np.mod(len(Genomes),save_interval)==0:
      with open(data_dir+'/genomes_'+str(len(Genomes))+'.pkl', 'wb') as output:
        pickle.dump(Genomes, output, pickle.HIGHEST_PROTOCOL)
      if os.path.isfile(data_dir+'/genomes_'+str(len(Genomes)-save_interval)+'.pkl')==1:
        os.remove(data_dir+'/genomes_'+str(len(Genomes)-save_interval)+'.pkl')

  p.resetSimulation()

def run(gens):
  pop = neat.population.Population(config)
  # pop=neat.Checkpointer.restore_checkpoint(os.path.dirname(__file__)+"/neat-checkpoint-999")
  stats = neat.statistics.StatisticsReporter()
  pop.add_reporter(stats)
  pop.add_reporter(neat.reporting.StdOutReporter(True))
  pop.run(eval_fitness, gens)
  # save
  try:
    os.chdir(data_dir)
    n=neat.Checkpointer()
    n.save_checkpoint(config,pop.population,pop.species,len(Genomes)-1)
    pkls=glob.glob(data_dir+"/*.pkl")
    for pkl in pkls:
      os.remove(pkl)
    with open(data_dir+'/genomes.pkl', 'wb') as output:
      pickle.dump(Genomes, output, pickle.HIGHEST_PROTOCOL)
      visualize.plot_stats(stats, ylog=False, view=True)
 
  except:
    pass

if __name__ == '__main__':
  run(400)