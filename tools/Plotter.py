import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
import os
import sys

#Syntax is "python Plotter.py n" where n is the nth last edited csv file.
#That means if you wanted the last edited file, you'd do "python Plotter.py 1"

files = os.listdir(os.getcwd())
files.sort(key=os.path.getmtime, reverse=True)
fileName=''
iter=0
for name in files:
    if 'TestingData' in name:
        fileName=name
        iter+=1
    if iter==int(sys.argv[1]):
        break
print fileName
data = np.genfromtxt(fileName, delimiter=',', names=True)

Time=data['Time']
Time = Time[~np.isnan(Time)]
LeftError=data['LeftError']
LeftError = LeftError[~np.isnan(LeftError)]
RightError=data['RightError']
RightError = RightError[~np.isnan(RightError)]
LeftPosition=data['LeftPosition']
LeftPosition = LeftPosition[~np.isnan(LeftPosition)]
RightPosition=data['RightPosition']
RightPosition = RightPosition[~np.isnan(RightPosition)]
LeftVelocity=data['LeftVelocity']
LeftVelocity = LeftVelocity[~np.isnan(LeftVelocity)]
RightVelocity=data['RightVelocity']
RightVelocity = RightVelocity[~np.isnan(RightVelocity)]
LeftSetpoint=data['LeftSetpoint']
LeftSetpoint = LeftSetpoint[~np.isnan(LeftSetpoint)]
RightSetpoint=data['RightSetpoint']
RightSetpoint = RightSetpoint[~np.isnan(RightSetpoint)]
WeightedLeftError=data['WeightedLeftError']
WeightedLeftError = WeightedLeftError[~np.isnan(WeightedLeftError)]
WeightedRightError=data['WeightedRightError']
WeightedRightError = WeightedRightError[~np.isnan(WeightedRightError)]
WeightedLeftPosition=data['WeightedLeftPosition']
WeightedLeftPosition = WeightedLeftPosition[~np.isnan(WeightedLeftPosition)]
WeightedRightPosition=data['WeightedRightPosition']
WeightedRightPosition = WeightedRightPosition[~np.isnan(WeightedRightPosition)]
WeightedLeftVelocity=data['WeightedLeftVelocity']
WeightedLeftVelocity = WeightedLeftVelocity[~np.isnan(WeightedLeftVelocity)]
WeightedRightVelocity=data['WeightedRightVelocity']
WeightedRightVelocity = WeightedRightVelocity[~np.isnan(WeightedRightVelocity)]
WeightedLeftSetpoint=data['WeightedLeftSetpoint']
WeightedLeftSetpoint = WeightedLeftSetpoint[~np.isnan(WeightedLeftSetpoint)]
WeightedRightSetpoint=data['WeightedRightSetpoint']
WeightedRightSetpoint = WeightedRightSetpoint[~np.isnan(WeightedRightSetpoint)]
kP=data['kP']
kP = kP[~np.isnan(kP)]
kI=data['kI']
kI = kI[~np.isnan(kI)]
kD=data['kD']
kD = kD[~np.isnan(kD)]
kFL=data['kFL']
kFL = kFL[~np.isnan(kFL)]
kFR=data['kFR']
kFR = kFR[~np.isnan(kFR)]
kR=data['kR']
kR = kR[~np.isnan(kR)]
Voltage=data['Voltage']
Voltage = Voltage[~np.isnan(Voltage)]
Heading=data['Heading']
Heading = Heading[~np.isnan(Heading)]

fig = plt.figure()
avgError = (np.array(LeftError)+np.array(RightError))/2
avgSetpoint = (np.array(LeftSetpoint)+np.array(RightSetpoint))/2
pv=str(np.nanmax(kP))
iv=str(np.nanmax(kI))
dv=str(np.nanmax(kD))
fv1=str(np.nanmax(kFL))
fv2=str(np.nanmax(kFR))
rv=str(np.nanmax(kR))

ax1 = fig.add_subplot(331)

ax1.set_title("Left Error by time")    
ax1.set_xlabel('Time')
ax1.set_ylabel('Error')
ax1.plot(Time, LeftError, color='g', label='Left Error')
ax1.plot(Time, LeftSetpoint, color='r', label='Left Setpoint')

ax2 = fig.add_subplot(332)

ax2.set_title("Average Error by time")    
ax2.set_xlabel('Time')
ax2.set_ylabel('Error')
ax2.plot(Time, avgSetpoint, color='r', label='avg setpoint')
ax2.plot(Time, avgError, color='g', label='avg error')

ax3 = fig.add_subplot(333)

ax3.set_title("Right Error by time")
ax3.set_xlabel('Time')
ax3.set_ylabel('Error')
ax3.plot(Time, RightError, color='g', label='Right Error')
ax3.plot(Time, RightSetpoint, color='r', label='Right Setpoint')

ax4 = fig.add_subplot(334)

ax4.set_title("Left Position by time")    
ax4.set_xlabel('Time')
ax4.set_ylabel('Position')
ax4.plot(Time, LeftPosition, color='g', label='Left Position')
ax4.plot(Time, LeftSetpoint, color='r', label='Left Setpoint')

ax5 = fig.add_subplot(335)

highestE = np.maximum(LeftError, RightError)
highestP = np.maximum(LeftPosition, RightPosition)
ep = np.divide(highestE, highestP)
highestSetpoint = np.maximum(LeftSetpoint, RightSetpoint)
ax5.set_title("Voltage by time")
ax5.set_xlabel('Time')
ax5.set_ylabel('Voltage')
ax5.plot(Time, Voltage, color='g', label='Volts')
ax5.plot(Time, np.array(np.add(LeftError, RightError))*12.5/1000.0, color='r', label='Total error')

ax6 = fig.add_subplot(336)

ax6.set_title("Right Position by time")    
ax6.set_xlabel('Time')
ax6.set_ylabel('Position')
ax6.plot(Time, RightPosition, color='g', label='Right Position')
ax6.plot(Time, RightSetpoint, color='r', label='Right Setpoint')

ax7 = fig.add_subplot(337)

ax7.set_title("Left Velocity by time")    
ax7.set_xlabel('Time')
ax7.set_ylabel('Velocity')
ax7.plot(Time, LeftVelocity, color='g', label='Left Velocity')
ax7.plot(Time, LeftSetpoint, color='r', label='Left Setpoint')

ax8 = fig.add_subplot(338)

highestE = np.maximum(LeftError, RightError)
highestV = np.maximum(LeftVelocity, RightVelocity)
ev = np.divide(highestE, highestV)
highestSetpoint = np.maximum(LeftSetpoint, RightSetpoint)
ax8.set_title("Heading and velocities by time")
ax8.set_xlabel('Time')
ax8.set_ylabel('Heading')
ax8.plot(Time, Heading, color='g', label='Heading')
ax8.plot(Time, LeftVelocity, color='b', label='Left Velocity')
ax8.plot(Time, RightVelocity, color='r', label='Right Velocity')

ax9 = fig.add_subplot(339)

ax9.set_title("Right Velocity by time")    
ax9.set_xlabel('Time')
ax9.set_ylabel('Velocity')
ax9.plot(Time, RightVelocity, color='g', label='Right Velocity')
ax9.plot(Time, RightSetpoint, color='r', label='Right Setpoint')



ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()
ax5.legend()
ax6.legend()
ax7.legend()
ax8.legend()
ax9.legend()
fig.text(0.5,0.95,"kP: "+pv+", kI: "+iv+", kD: "+dv+", kFL: "+fv1+", kFR: "+fv2+", kR: "+rv,fontsize=10)
plt.show()