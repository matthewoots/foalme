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
    fig, (ax1, ax2) = plt.subplots(2, 1)

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
                dst = float(row[3])
                time.append(float("{:.2f}".format(ts)))
                dist.append(float("{:.2f}".format(dst)))
                speed.append(float(row[2]))
        ax1.plot(time, speed, linestyle = 'dashed',
                marker = 'o',label = "drone" + str(id), markersize=2)
        ax2.plot(time, dist, linestyle = 'dashed',
                marker = 'o',label = "drone" + str(id), markersize=2)
    
    ax1.set(xlabel='time', ylabel='speed')
    ax1.legend(loc="upper right", prop={'size': 6})
    ax1.set_title('speed against time', fontsize = 8)
    # ax1.set_yticks(np.arange(0, 10, 10))

    ax2.set(xlabel='time', ylabel='distance')
    ax2.legend(loc="upper right", prop={'size': 6})
    ax2.set_title('distance against time', fontsize = 8)

    # Packing all the plots and displaying them
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
	main(sys.argv)