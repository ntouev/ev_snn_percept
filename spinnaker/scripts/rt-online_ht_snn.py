import pyNN.spiNNaker as p
from pacman.model.graphs.application import ApplicationSpiNNakerLinkVertex
from spinn_front_end_common.abstract_models import AbstractProvidesOutgoingPartitionConstraints
from pacman.model.constraints.key_allocator_constraints import FixedKeyAndMaskConstraint
from pacman.model.routing_info import BaseKeyAndMask
import numpy as np
import threading

def run():  
    p.external_devices.run_forever()
    
def stop():
    input("Press Enter to stop ...")
    print("STOPPING ...")
    p.external_devices.request_stop()
  
if __name__ == "__main__":

    ################ USER INPUT ####################
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

    # Using first SpiNNaker Link. Change to 1 if using the other one.
    SPINNAKER_LINK_ID = 0

    # no need to change these
    RETINA_KEY  = 0xFEFE0000
    RETINA_MASK = 0xFFFF0000

    # UDP port for live output (output spikes going to ROS live_output node)
    # must be the same as the one define in the ROS code
    RECEIVER_PORT = 56789

    ################################################
    RETINA_POP_XY_WEIGHT = 5/ssa_xy_weight_denominator
    RETINA_POP_XY_DELAY = 1

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

    #######################################################################################################
    connections_retina_pop_xy = []
    for n in range(RETINA_DIM*RETINA_DIM*2):
        ret_y = (n & 0x7F00) >> 8
        ret_x = (n & 0x00FE) >> 1
        ret_p = n & 0x0001

        if (ret_x >= L) and (ret_x <= RETINA_DIM - L - 1):    
            retina_pop_xy_weight_mod = RETINA_POP_XY_WEIGHT * G
        else:
            retina_pop_xy_weight_mod = RETINA_POP_XY_WEIGHT

        tup = (n, 
            ret_x//SCALEDOWN + X_NUM*(ret_y//SCALEDOWN) + X_NUM*Y_NUM*ret_p,
            retina_pop_xy_weight_mod,
            RETINA_POP_XY_DELAY)
        connections_retina_pop_xy.append(tup)

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
    print('length connections_ssa_xy:', len(connections_retina_pop_xy))
    print('length connections_xy_planar:', len(connections_xy_planar))
    print('length connections_planar_planar_inh:', len(connections_planar_planar_inh))

    #######################################################################################################
    class MyFPGADevice(ApplicationSpiNNakerLinkVertex):
        def __init__(self, n_neurons=RETINA_DIM*RETINA_DIM*2, spinnaker_link_id=SPINNAKER_LINK_ID, \
                    board_address=None, label=None):
                super().__init__(
                    n_atoms=n_neurons,
                    spinnaker_link_id=spinnaker_link_id,
                    label=label,
                    max_atoms_per_core=n_neurons,
                    board_address=board_address,
                    constraints=[FixedKeyAndMaskConstraint([BaseKeyAndMask(RETINA_KEY, RETINA_MASK)])])


    #######################################################################################################
    p.setup(1.0)

    retina_pop = p.Population(None, MyFPGADevice)

    pop_xy = p.Population(X_NUM*Y_NUM*P_NUM,
                        p.IF_curr_exp,
                        label = 'pop_xy')

    pop_planar = p.Population(THETA_NUM*R_NUM*P_NUM,
                            p.IF_curr_exp,
                            label = 'pop_output')

    p.Projection(retina_pop, 
                pop_xy, 
                p.FromListConnector(connections_retina_pop_xy),
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

    p.external_devices.activate_live_output_for(pop_planar, database_notify_port_num=RECEIVER_PORT)

    #######################################################################################################
    run_thread = threading.Thread(target=run)
    stop_thread = threading.Thread(target=stop)
    
    stop_thread.start()
    run_thread.start()
  
    stop_thread.join()
    run_thread.join()
    
    p.end()
    print("STOPPED!")