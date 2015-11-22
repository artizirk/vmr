import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, strPort, maxLen):
      # open serial port
      self.ser = serial.Serial(strPort, 115200)
      self.ser.flush()
      self.ax = deque([0.0]*maxLen)
      self.ay = deque([0.0]*maxLen)
      self.az = deque([0.0]*maxLen)
      self.maxLen = maxLen

  # add to buffer
  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  # add data
  def add(self, data):
      assert(len(data) == 3)
      self.addToBuf(self.ax, data[0])
      self.addToBuf(self.ay, data[1])
      self.addToBuf(self.az, data[2])

  # update plot
  def update(self, frameNum, a0, a1, a2, ar):
      try:
          line = self.ser.readline().decode()
          data_pairs = [[x.strip() for x in val.split("=")] for val in line.split("\t")]
          #print(data_pairs)
          data = {}
          for pair in data_pairs:
              #print(pair)
              try:
                data[pair[0]] = int(pair[1])
              except:
                data[pair[0]] = pair[1]
          print(data)
          data = data.get("Vasak",0), data.get("Parem",0), data.get("Keskmine",0)
          if(len(data) == 3):
              self.add(data)
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
              a2.set_data(range(self.maxLen), self.az)
      except KeyboardInterrupt:
          print('exiting')
      except IndexError:
          print(line.strip())
      except Exception as e:
          print("garbage", e)
      
      return a0, 

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()    

# main() function
def main():
  # create parser
  parser = argparse.ArgumentParser(description="lanyrint debug parser")
  # add expected arguments
  parser.add_argument('--port', dest='port', required=True)

  # parse args
  args = parser.parse_args()
  
  #strPort = '/dev/tty.usbserial-A7006Yqh'
  strPort = args.port

  print('reading from serial port %s...' % strPort)

  # plot parameters
  analogPlot = AnalogPlot(strPort, 100)

  print('plotting data...')

  # set up animation
  fig = plt.figure()
  ax = plt.axes(xlim=(0, 100), ylim=(0, 2048))
  a0, = ax.plot([], [])
  a1, = ax.plot([], [])
  a3, = ax.plot([], [])
  ar = plt.arrow( 0.5, 0.8, 0.0, -0.2, fc="k", ec="k", head_width=0.05, head_length=0.1 )
  anim = animation.FuncAnimation(fig, analogPlot.update, 
                                 fargs=(a0, a1, a3, ar), 
                                 interval=5)


  #plt.subplot(111)
  #plt.arrow( 0.5, 0.8, 0.0, -0.2, fc="k", ec="k", head_width=0.05, head_length=0.1 )


  # show plot
  plt.show()
  
  # clean up
  analogPlot.close()

  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()

