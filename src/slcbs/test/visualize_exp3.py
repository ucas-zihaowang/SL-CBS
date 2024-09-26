#!/usr/bin/env python3

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle, Rectangle, Arrow, FancyArrow
from matplotlib.collections import PatchCollection
from matplotlib import cm
from matplotlib.colors import ListedColormap  

import os
import argparse
import numpy as np
import yaml

import math

# matplotlib.use("Qt5Agg")
matplotlib.use( "Agg" )

PLOTLINE = True
YAW = True

UAVRadius = 1.0
OBSRadius = 0.5
ArrowLength = UAVRadius * 1.2

class Animation:
    def __init__( self, map, schedule ):
        self.map = map
        self.schedule = schedule

        aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
        self.fig = plt.figure( frameon=False, figsize=(8 * aspect, 8), facecolor='white' )
        self.ax = self.fig.add_subplot( 111, aspect='equal', facecolor='white' )
        self.fig.subplots_adjust( left=0, right=1, bottom=0, top=1, wspace=None, hspace=None )
        xmin = -1
        ymin = -1
        xmax = map["map"]["dimensions"][0] + 0.5
        ymax = map["map"]["dimensions"][1] + 0.5
        plt.xlim( xmin, xmax )
        plt.ylim( ymin, ymax )
        self.ax.axis('off')


        self.patches = []
        self.patches.append( Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red') )

        for o in map["map"]["obstacles"]:
            x, y = o[0], o[1]
            self.patches.append( Circle((x, y), OBSRadius, facecolor='grey', edgecolor='grey') )

        # cmap = plt.cm.get_cmap( 'hsv', len(map["agents"])+1 )
        # cmap = plt.colormaps['hsv'](np.linspace(0, 1, len(map["agents"]) + 1))
        cmap = ListedColormap(plt.colormaps['hsv'](np.linspace(0, 1, len(map["agents"]) + 1)) )


        for d, i in zip( map["agents"], range( 0,len(map["agents"]) ) ):
            self.patches.append( 
                Circle( (d["goal"][0], d["goal"][1]), 1.2 * UAVRadius,  facecolor='none', edgecolor= cmap(i+1),  alpha=0.7, lw=1, linestyle=":")
            )
            # self.patches.append( 
            #     FancyArrow( d["goal"][0], d["goal"][1], ArrowLength * math.cos(d["goal"][2]), ArrowLength * math.sin(d["goal"][2]), width=0.05, head_width= 5 * 0.05, facecolor='black', edgecolor='black')
            # )
            
        
        self.agents = dict()
        self.agent_names = dict()
        self.agents_arrow = dict()
        self.artists = []
        for d, i in zip( map["agents"], range(0, len(map["agents"])) ):
            name = d["name"]
            self.agents[name] = Circle( (d["start"][0], d["start"][1]), UAVRadius, edgecolor='black', alpha=0.7 )
            self.agents[name].original_face_color = cmap(i+1)
            self.patches.append( self.agents[name] )

            if YAW:
                self.agents_arrow[name]= FancyArrow( d["start"][0], d["start"][1], ArrowLength * math.cos(d["start"][2]), ArrowLength * math.sin(d["start"][2]), width=0.05, head_width= 5 * 0.05, facecolor='black', edgecolor='black')
                self.patches.append( self.agents_arrow[name] )
            
            self.agent_names[name] = self.ax.text( d["start"][0], d["start"][1], name.replace('agent', ''), fontsize=6 )
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append( self.agent_names[name] )


        self.lines = []
        self.list_xdata = [ [] for _ in range(0, len(map["agents"])) ]
        self.list_ydata = [ [] for _ in range(0, len(map["agents"])) ]
        for d, i in zip( map["agents"], range( 0,len(map["agents"]) ) ):
            self.list_xdata[i].append( d["start"][0] )
            self.list_ydata[i].append( d["start"][1] )
            line, = self.ax.plot( self.list_xdata[i], self.list_ydata[i], color=cmap(i+1),  alpha=0.3, lw=2, linestyle="-." )
            self.lines.append( line )


        self.T = 0
        for d, i in zip( map["agents"], range(0, len(map["agents"])) ):
            name = d["name"]
            self.T = max( self.T, len(schedule["schedule"][name]) )
            
        self.anim = animation.FuncAnimation( self.fig, self.animate_func, init_func = self.init_func, 
                                            frames=int(self.T+1), interval = 100, repeat=False, blit=True )

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists
    
    def animate_func( self, i ):
        for agent_name in self.schedule["schedule"]:
            agent = schedule["schedule"][agent_name]
            pos = self.getState( i , agent )
            pos_tuple = ( pos[0], pos[1] )
            self.agents[agent_name].center = pos_tuple

            if YAW:
                self.agents_arrow[agent_name].set_data( x=pos[0], y=pos[1], dx=ArrowLength * math.cos(pos[2]), dy=ArrowLength * math.sin(pos[2]) )

            self.agent_names[agent_name].set_position( pos_tuple )
            if PLOTLINE:
                self.list_xdata[int(agent_name.replace("agent", ""))].append(pos[0])
                self.list_ydata[int(agent_name.replace("agent", ""))].append(pos[1])
                self.lines[int(agent_name.replace("agent", ""))].set_data(
                    self.list_xdata[int(agent_name.replace("agent", ""))], self.list_ydata[int(agent_name.replace("agent", ""))]
                    )
        for _, agent in self.agents.items():
            agent.set_facecolor( agent.original_face_color )
        return self.patches + self.artists + self.lines

    def getState( self, t, d ):
        if t < len(d):
            return np.array( [ float(d[t]["x"]), float(d[t]["y"]),  float(d[t]["yaw"]) ] )
        else:
            return np.array( [ float(d[-1]["x"]), float(d[-1]["y"]) , float(d[-1]["yaw"]) ] )

    def show(self):
        plt.show()
    
    def save(self, file_name, speed):
        self.anim.save( file_name, "ffmpeg", fps=10 * speed, dpi=300, savefig_kwargs={'facecolor':'white'} )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument( "-m", "--map", default=None, help="input file containing map" )
    parser.add_argument( "-s", "--schedule", default=None, help="schedule for agents")
    parser.add_argument( "-v", "--video", dest='video', default=None, help="output video file (or leave empty to show on screen)")
    parser.add_argument( "--speed", type=int, default=1, help="speedup-factor" )
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.schedule) as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)
    
    animation = Animation( map, schedule )

    gifpath = args.schedule.replace(".yaml", ".mp4")
    

    if 1:
        matplotlib.use( "Agg" )
        animation.save( gifpath, args.speed )
    else:
        matplotlib.use( "Qt5Agg" )
        animation.show()
