import sys
import numpy as np
import os
import math
import matplotlib.pyplot as plt
from pathlib import Path
import time
from datetime import datetime
import csv

def main(argv):
    num = int(argv[1])
    # Placing the plots in the plane
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)

    # csv format: 
    # 0. "time"
    # 1. "computation_time"
    # 2. "speed" 
    # 3. "total_distance"
    # 4. "collision_detection"

    for id in range(num):
        time = [] 
        speed = []
        dist = []
        compute = []
        collision = []

        print("printing drone" + str(id))
        dir = str(Path.home()) + "/Documents" + "/drone" + str(id) + "_log_file.csv"
        with open(dir,'r') as csvfile:
            lines = csv.reader(csvfile, delimiter=';')
            # lines = csv.reader(csvfile, delimiter=',')
            count = 0
            for row in lines:
                if count == 0:
                    count = count + 1
                    continue
                # ts_epoch = row[0]
                # ts = datetime.fromtimestamp(float(ts_epoch)).strftime('%Y-%m-%d %H:%M:%S')
                ts = float(row[0])
                com = float(row[1])
                dst = float(row[3])

                time.append(float("{:.3f}".format(ts)))
                dist.append(float("{:.3f}".format(dst)))
                compute.append(float("{:.3f}".format(com)))
                speed.append(float(row[2]))
                collision.append(int(row[4]))

        ax1.plot(time, speed, linestyle = 'dashed',
                marker = 'o',label = "drone" + str(id), markersize=1)
        ax2.plot(time, dist, linestyle = 'dashed',
                marker = 'o',label = "drone" + str(id), markersize=1)
        ax3.plot(time, compute, linestyle = 'dashed',
                marker = 'o',label = "drone" + str(id), markersize=1)
        ax4.plot(time, collision, linestyle = 'dashed',
                marker = 'o',label = "drone" + str(id), markersize=1)
    
    # ax1.set(xlabel='time', ylabel='speed')
    # ax1.legend(loc="upper right", prop={'size': 4})
    ax1.set_title('speed against time', fontsize = 8)
    ax1.grid()
    # ax1.set_yticks(np.arange(0, 10, 10))

    # ax2.set(xlabel='time', ylabel='distance')
    # ax2.legend(loc="upper right", prop={'size': 4})
    ax2.set_title('distance against time', fontsize = 8)
    ax2.grid()

    # ax3.set(xlabel='time', ylabel='compute_time in sec')
    # ax3.legend(loc="upper right", prop={'size': 4})
    ax3.set_title('compute_time against time', fontsize = 8)
    ax3.grid()

    # ax4.set(xlabel='time', ylabel='collision')
    # ax4.legend(loc="upper right", prop={'size': 4})
    ax4.set_title('collision against time', fontsize = 8)
    ax4.grid()

    # Packing all the plots and displaying them
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
	main(sys.argv)