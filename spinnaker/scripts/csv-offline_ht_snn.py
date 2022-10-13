import numpy as np
import os
import csv
import pyNN.spiNNaker as p
import pyNN.utility.plotting as plot
import matplotlib.pyplot as plt

################ USER INPUT ####################
# input csv file
csv_name = '~/ev_snn_percept/CSVs/sample_pavement.csv'
# directory to store the output numpy file and figure
output_directory = '~/ev_snn_percept/spinnaker/sample_pavement/'

# simulation time
simtime = 16000

# input (ssa) to xy population weight denominator
# the real weight is defined below
ssa_xy_weight_denominator = 1

# xy to planar population weight denominator
# the real weight is defined below
xy_planar_weight_denominator = 225

# planar population inhibiroty weight nominator
# the real weight is defined below
planar_planar_inh_weight_nominator = 10

# radius of inhibition circle
r_inh = 20

# window [0+L,127-L]x[0,127] on input (ssa) population 
# connects with xy population with a weight SSA_XY_WEIGHT * G
# the rest is connected with weigh SSA_XY_WEIGHT
# l, g define L and G below
# this is used to reduce noice induced in the edges
# to remove this functionality set G = 10
l = 24
g = 30

# input dimension is RETINA_DIM x RETINA_DIM
RETINA_DIM = 128
# xy dimension is RETINA_DIM/SCALEDOWN x RETINA_DIM/SCALEDOWN
SCALEDOWN = 4
# planar dimension is THETA_NUM x R_NUM
THETA_NUM = 90
R_NUM = 60   

################################################
SSA_XY_WEIGHT = 5/ssa_xy_weight_denominator
SSA_XY_DELAY = 1

XY_PLANAR_WEIGHT = 5/xy_planar_weight_denominator
XY_PLANAR_DELAY = 1

PLANAR_PLANAR_INH_WEIGHT = planar_planar_inh_weight_nominator/1
PLANAR_PLANAR_INH_DELAY = 1
R_INH = r_inh

L = l
G = g/10

X_NUM = round(RETINA_DIM/SCALEDOWN)
Y_NUM = round(RETINA_DIM/SCALEDOWN)
P_NUM = 2

npy_name =  output_directory + 'output_' + \
            str(simtime) + '_' + '5:' + str(ssa_xy_weight_denominator) + '_' + '5:' + str(xy_planar_weight_denominator) + '_' + \
            str(planar_planar_inh_weight_nominator) + ':1' + '_' + str(R_INH) + '_L' + str(L) + '_G' + str(G) + \
           '.npy'

#######################################################################################################    
rows = []
xs = []
ys = []
timestamps = []
polarities = []
neuron_ids = []
    
with open(csv_name, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
 
    for row in csvreader:
        rows.append(row)
 
    num_events = csvreader.line_num
    print("Total no. of rows -events-: %d"%(num_events))

for j, row in enumerate(rows):
    for i, col in enumerate(row):
        
        if (i == 0):
            col = float(col)
            col = round(col)
            xs.append(col)
        if (i == 1):
            col = float(col)
            col = round(col)
            ys.append(col)
        if (i == 2):
            col = round(1000*float(col))
            timestamps.append(col)
        if (i == 3):
            col = float(col)
            col = round(col)
            if col == -1:
                col = 0
            polarities.append(col)
    
    n = xs[j] + RETINA_DIM*ys[j] + RETINA_DIM*RETINA_DIM*polarities[j]
    neuron_ids.append(n)
        
        
events_per_neuron = []
for i in range(RETINA_DIM*RETINA_DIM*P_NUM):
    events_per_neuron.append([])
    
for i in range(num_events):
    events_per_neuron[neuron_ids[i]].append(timestamps[i])

#######################################################################################################
connections_ssa_xy = []
for ret_p in range(P_NUM):
    for ret_y in range(RETINA_DIM):
        for ret_x in range(RETINA_DIM):
            if (ret_x >= L) and (ret_x <= RETINA_DIM - L - 1):    
                ssa_xy_weight_mod = SSA_XY_WEIGHT * G
            else:
                ssa_xy_weight_mod = SSA_XY_WEIGHT

            tup = (ret_x + RETINA_DIM*ret_y + RETINA_DIM*RETINA_DIM*ret_p, 
                   ret_x//SCALEDOWN + X_NUM*(ret_y//SCALEDOWN) + X_NUM*Y_NUM*ret_p,
                   ssa_xy_weight_mod,
                   SSA_XY_DELAY) 
            connections_ssa_xy.append(tup)

#######################################################################################################
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

x_arr = np.linspace(0, X_NUM-1, num=X_NUM)
y_arr = np.linspace(0, Y_NUM-1, num=Y_NUM)

theta_arr = np.linspace(-np.pi/2, np.pi/2, num=THETA_NUM)
r_arr = np.linspace(-round(X_NUM*np.sqrt(2)), round(X_NUM*np.sqrt(2)), num=R_NUM)

connections_xy_planar = []
for x in x_arr:
    for y in y_arr:
        for polarity in range(P_NUM):
            for theta in theta_arr:
                r = x*np.cos(theta) + y*np.sin(theta)

                tup = (x + X_NUM*y + X_NUM*Y_NUM*polarity, 
                       np.where(theta_arr==find_nearest(theta_arr, theta))[0][0] + 
                         THETA_NUM*np.where(r_arr==find_nearest(r_arr, r))[0][0] +
                         THETA_NUM*R_NUM*polarity,
                       XY_PLANAR_WEIGHT,
                       XY_PLANAR_DELAY)
                connections_xy_planar.append(tup)

#######################################################################################################
connections_planar_planar_inh = []
for polarity in range(P_NUM):
    for j in range(R_NUM):
        for i in range(THETA_NUM):
            for k in range(-R_INH,R_INH+1):
                if j+k>=0 and j+k<R_NUM: 
                    for t in range(-R_INH,R_INH+1):
                        if i+t>=0 and i+t<THETA_NUM:
                            if k != 0 or t != 0:
                                tup = (i + THETA_NUM*j + THETA_NUM*R_NUM*polarity, 
                                       i+t + THETA_NUM*(j+k) + THETA_NUM*R_NUM*polarity,
                                       PLANAR_PLANAR_INH_WEIGHT,
                                       PLANAR_PLANAR_INH_DELAY)

                                connections_planar_planar_inh.append(tup)

#######################################################################################################
print('length connections_ssa_xy:', len(connections_ssa_xy))
print('length connections_xy_planar:', len(connections_xy_planar))
print('length connections_planar_planar_inh:', len(connections_planar_planar_inh))

#######################################################################################################
p.setup(1.0)

ssa = p.Population(RETINA_DIM*RETINA_DIM*P_NUM, p.SpikeSourceArray(spike_times=events_per_neuron))

pop_xy = p.Population(X_NUM*Y_NUM*P_NUM,
                      p.IF_curr_exp,
                      label = 'pop_xy')

pop_planar = p.Population(THETA_NUM*R_NUM*P_NUM,
                          p.IF_curr_exp,
                          label = 'pop_planar')

ssa.record(['spikes'])
pop_xy.record(['spikes'])
pop_planar.record(['spikes'])

p.Projection(ssa, 
             pop_xy, 
             p.FromListConnector(connections_ssa_xy),
             p.StaticSynapse())

p.Projection(pop_xy, 
             pop_planar, 
             p.FromListConnector(connections_xy_planar),
             p.StaticSynapse())

p.Projection(pop_planar, 
             pop_planar, 
             p.FromListConnector(connections_planar_planar_inh),
             p.StaticSynapse(),
             receptor_type="inhibitory")

#######################################################################################################
p.run(simtime)

#######################################################################################################
neo_ssa = ssa.get_data(variables=['spikes'])

spikes_ssa = neo_ssa.segments[0].spiketrains

neo_xy = pop_xy.get_data(variables=['spikes'])

spikes_xy = neo_xy.segments[0].spiketrains

neo_planar = pop_planar.get_data(variables=['spikes'])

spikes_planar = neo_planar.segments[0].spiketrains

spikes_planar_np = pop_planar.spinnaker_get_data(['spikes'])

p.end()

#######################################################################################################
plot.Figure(
        plot.Panel(spikes_xy, yticks=True, markersize=5, xlim=(0,simtime)),
        plot.Panel(spikes_planar, yticks=True, markersize=5, xlim=(0,simtime)),
        title='spikes')

plt.savefig(npy_name.replace('.npy','.png'))
np.save (npy_name, spikes_planar_np)