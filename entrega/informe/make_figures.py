from pathlib import Path
import json, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
import yaml
ROOT=Path(__file__).resolve().parent
DATA=ROOT/'data'
OUT=ROOT/'figures'
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.bbox':'tight'})
fig,axs=plt.subplots(1,2,figsize=(10,4.5),sharex=True,sharey=True)
for ax,case,title in zip(axs,['mapping_slow','mapping_fast'],['Límite 0,20 m/s','Límite 0,50 m/s']):
    m=yaml.safe_load((DATA/(case+'.yaml')).read_text()); a=np.array(Image.open(DATA/(case+'.pgm')))
    x,y,_=m['origin']; r=m['resolution']; h,w=a.shape
    ax.imshow(a,cmap='gray',vmin=0,vmax=255,extent=[x,x+w*r,y,y+h*r],interpolation='nearest')
    ax.add_patch(Rectangle((-3.9,-3.9),7.8,7.8,fill=False,edgecolor='#e37722',lw=1,ls='--'))
    ax.set(xlim=(-6,6),ylim=(-6,6),xlabel='x [m]',ylabel='y [m]',title=title,aspect='equal')
fig.suptitle('Mapas guardados por SLAM Toolbox · resolución 0,05 m/celda')
fig.tight_layout();fig.savefig(OUT/'mapas.pdf');plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(9,6),sharex=True)
for case,label,color,ls in [('correct_low','Correcta / σ=0,01 m','#176b9b','-'),('correct_high','Correcta / σ=0,05 m','#176b9b','--'),('wrong_low','Incorrecta / σ=0,01 m','#bf5b14','-'),('wrong_high','Incorrecta / σ=0,05 m','#bf5b14','--')]:
    a=np.genfromtxt(DATA/('amcl_'+case+'.csv'),delimiter=',',names=True)
    axs[0].plot(a['sim_s'],a['position_error_m'],label=label,color=color,ls=ls)
    axs[1].plot(a['sim_s'],a['yaw_error_rad'],color=color,ls=ls)
axs[0].axhline(.2,color='gray',ls=':',label='Umbral de posición')
axs[1].axhline(.15,color='gray',ls=':')
axs[0].set(ylabel='Error de posición [m]',title='AMCL frente a la pose física de Gazebo');axs[0].legend(ncol=2,fontsize=9)
axs[1].set(xlabel='Tiempo de simulación desde pose inicial [s]',ylabel='Error angular [rad]')
for ax in axs: ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(OUT/'amcl.pdf');plt.close(fig)
fig,ax=plt.subplots(figsize=(8,3.6))
labels=['Ruedas\n(caso A)','EKF + IMU\n(caso A)','EKF sin IMU\n(caso B)','ros2_control +\nEKF + IMU (C)']
vals=[abs(2.5-2.6623),abs(2.6149-2.6623),abs(2.499-2.6621),abs(2.5813-2.6621)]
bars=ax.bar(labels,vals,color=['#8996a4','#176b9b','#bf5b14','#3b9c9a'])
ax.bar_label(bars,labels=[f'{v:.4f}' for v in vals],padding=3)
ax.set(ylabel='Error absoluto de yaw [rad]',ylim=(0,.2),title='Giro final: mediciones documentadas por Persona A')
fig.tight_layout();fig.savefig(OUT/'ekf.pdf');plt.close(fig)
fig,ax=plt.subplots(figsize=(6,4))
pts=np.array([[0,0],[0,-.7],[-.7,-.7],[-.7,0],[0,0]])
ax.plot(pts[:,0],pts[:,1],'o-',color='#176b9b')
for xy,label in zip(pts[1:],['1 · sur','2 · suroeste','3 · oeste','4 · origen']):ax.annotate(label,xy,xytext=(8,8),textcoords='offset points')
ax.set(xlabel='x [m]',ylabel='y [m]',title='Objetivos de misión en el frame map',xlim=(-1,.35),ylim=(-1,.3),aspect='equal');ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(OUT/'waypoints.pdf');plt.close(fig)
print('4 figuras generadas con datos medidos/configurados')
