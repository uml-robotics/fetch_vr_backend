#!/usr/bin/env python
from email import header
import time
import Tkinter as tk
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Header
import threading
from datetime import datetime
from datetime import timedelta

# CONSTANTS
DELAY_OPTIONS = [
90,
135,
180
]

SA1_OPTIONS = [
"Where is the obstacle closest to the robot?",
"What color is the obstacle closest to the robot?",
"Where is the robot on this map?",
"Which wall is closest to the robot?"
]

SA2_OPTIONS = [
"How many objectives has the robot reached thus far?",
"Where was the last objective the robot reached? If you have not yet reached an objective, where was the starting location?",
"Out of the three objectives, how many are remaining?",
"Where is the next objective?"
]

SA3_OPTIONS = [
"Based on where the robot is currently located on the grid, if the robot drives up 2 grid blocks, will it run into any obstacles?",
"Based on where the robot is currently located on the grid, if the robot drives down 2 grid blocks, will it run into any obstacles.",
"Based on where the robot is currently located on the grid, if the robot drives right 2 grid blocks, will it run into any obstacles.",
"In which direction would the robot need to turn toward to be at the next target?",
"In terms of distance, what percent of the way are you to your next target? Please use the slider."
]

# GLOBALS
isStarted = False
startTime = -1
isPaused = False
pauseStartTime = -1
totalPauseDuration = timedelta(0)
isSAStarted = False
saCallback = None
saCurrentDelay = -1
saStartTime = -1

# ROS
pause_pub = rospy.Publisher('pause', Bool, queue_size=10)
start_pub = rospy.Publisher('start', Bool, queue_size=10)
end_pub = rospy.Publisher('stop', Bool, queue_size=10)
time_pub = rospy.Publisher('runtime', Header, queue_size=10)
question_pub = rospy.Publisher('question', String, queue_size=10)
rospy.init_node('experimenter_ui', anonymous=True)
rate = rospy.Rate(10)

def update():
    if isStarted:
        totalElapsedTime = datetime.now()-startTime

        currentPauseElapsedTime = timedelta(0)
        if isPaused:
            currentPauseElapsedTime = datetime.now() - pauseStartTime

        elapsedTime = totalElapsedTime - currentPauseElapsedTime - totalPauseDuration

        timeLabel.config(text=str(elapsedTime).split(".")[0])
        timeMsg = Header()
        timeMsg.stamp.secs = elapsedTime.total_seconds()
        time_pub.publish(timeMsg)
        if isSAStarted:
            if(isPaused):
                elapsedSATime = timedelta(seconds=0)
            else:
                elapsedSATime = datetime.now() - saStartTime

            countdown = timedelta(seconds=saCurrentDelay) - elapsedSATime

            saTimeLabel.config(text=str(countdown).split(".")[0])
    window.after(100, lambda: update())

def onPausePressed():
    if not isStarted:
        return

    if isPaused:
        unPause()
    else:
        pause()

    pause_pub.publish(isPaused)
    rate.sleep()

def pause():
    global isPaused
    isPaused = True
    pauseBtn.configure(text="RESUME")
    global pauseStartTime
    pauseStartTime = datetime.now()
    if isSAStarted:
        pauseSATimer()

def unPause():
    global isPaused
    isPaused = False
    pauseBtn.configure(text="PAUSE")
    global totalPauseDuration
    global pauseStartTime
    totalPauseDuration += (datetime.now() - pauseStartTime)
    pauseStartTime = -1
    if isSAStarted:
        unPauseSATimer()

def onStartPressed():
    if isStarted:
        endRun()
    else:
        startRun()

def startRun():
    global isStarted
    isStarted = True
    global startTime
    startTime = datetime.now()

    startBtn.configure(text="END RUN")
    
    start_pub.publish(True)
    rate.sleep()

def endRun():
    global isStarted
    isStarted = False
    global startTime
    startTime = -1

    # RESET PAUSE
    if isPaused:
        unPause()
    global totalPauseDuration
    totalPauseDuration = timedelta(0)

    # RESET SA
    if isSAStarted:
        clearSATimer()        
            
    startBtn.configure(text="START RUN")
    timeLabel.configure(text='')
    end_pub.publish(True)

def onStartSAPressed():
    if not isStarted or isPaused:
        return

    if isSAStarted:
        clearSATimer()
    else:
        startSATimer()

def startSATimer():
    global isSAStarted
    isSAStarted = True
    saBtn.configure(text="CANCEL SA TIMER")
    global saCurrentDelay
    saCurrentDelay = delay.get()
    global saCallback
    saCallback = threading.Timer(saCurrentDelay, askSA)
    saCallback.start()
    global saStartTime
    saStartTime = datetime.now()

    delaySelect.pack_forget()
    sa1Select.pack_forget()
    sa2Select.pack_forget()
    sa3Select.pack_forget()

def pauseSATimer():
    global saCallback
    if(saCallback):
        saCallback.cancel()
    totalTimeRun = pauseStartTime - saStartTime
    global saCurrentDelay
    saCurrentDelay -= totalTimeRun.total_seconds()

def unPauseSATimer():
    global saCallback
    saCallback = threading.Timer(saCurrentDelay, askSA)
    saCallback.start()
    global saStartTime
    saStartTime = datetime.now()

def clearSATimer():
    global isSAStarted
    isSAStarted = False
    saBtn.configure(text="START SA TIMER")
    global saCurrentDelay
    saCurrentDelay = -1
    global saCallback
    if(saCallback):
        saCallback.cancel()
    global saStartTime
    saStartTime = -1

    saTimeLabel.config(text='')

    delaySelect.pack()
    sa1Select.pack()
    sa2Select.pack()
    sa3Select.pack()


def askSA():
    questionStr = sa1.get() + ";" + sa2.get() + ";" + sa3.get()

    clearSATimer()

    print("ASKING: " + questionStr)
    question_pub.publish(questionStr)
    rate.sleep()


# Main Loop for TK
window = tk.Tk()

for i in range(3):
    window.columnconfigure(i, weight=1, minsize=480)
    window.rowconfigure(i, weight=1, minsize=480)

frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)

frame.grid(row=0, column=0, padx=5, pady=5)
startBtn = tk.Button(master=frame, text="START", width=30,
                  command=onStartPressed)
startBtn.pack(padx=5, pady=5)
timeLabel = tk.Label(master=frame, text='')
timeLabel.pack(padx=5, pady=5)


frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)

frame.grid(row=0, column=2, padx=5, pady=5)
pauseBtn = tk.Button(master=frame, text="PAUSE", width=30,
                  command=onPausePressed)
pauseBtn.pack(padx=5, pady=5)

frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)

frame.grid(row=1, column=0, padx=5, pady=5)
saBtn = tk.Button(master=frame, text="START SA TIMER", width=30,
                  command=onStartSAPressed)
saBtn.pack(padx=5, pady=5)
saTimeLabel = tk.Label(master=frame, text='')
saTimeLabel.pack(padx=5, pady=5)

frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)

delay = tk.IntVar(frame)
delay.set(DELAY_OPTIONS[0])

sa1 = tk.StringVar(frame)
sa1.set(SA1_OPTIONS[0])

sa2 = tk.StringVar(frame)
sa2.set(SA2_OPTIONS[0])

sa3 = tk.StringVar(frame)
sa3.set(SA3_OPTIONS[0])

frame.grid(row=1, column=2, padx=5, pady=5)
delaySelect = tk.OptionMenu(frame, delay, *DELAY_OPTIONS)
delaySelect.pack()

sa1Select = tk.OptionMenu(frame, sa1, *SA1_OPTIONS)
sa1Select.pack()
sa2Select = tk.OptionMenu(frame, sa2, *SA2_OPTIONS)
sa2Select.pack()
sa3Select = tk.OptionMenu(frame, sa3, *SA3_OPTIONS)
sa3Select.pack()

update()
window.mainloop()
rospy.spin()
