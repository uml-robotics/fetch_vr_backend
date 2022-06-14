#!/usr/bin/env python
import Tkinter as tk
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import threading

def onPause():
    global isStarted
    global isPaused
    global pauseBtn

    if not isStarted:
        return

    isPaused = not (isPaused)
    if not isPaused:
        pauseBtn.configure(text="PAUSE")
    else:
        pauseBtn.configure(text="RESUME")

    pause_pub.publish(isPaused)
    rate.sleep()

def onStart():
    global isStarted
    global startBtn

    isStarted = not (isStarted)
    if not isStarted:
        startBtn.configure(text="START RUN")
        end_pub.publish(True)
    else:
        startBtn.configure(text="END RUN")
        start_pub.publish(True)

    rate.sleep()

def onStartSA():
    global isSAStarted
    global saBtn
    global saCallback
    global delay

    isSAStarted = not (isSAStarted)
    if not isSAStarted:
        saBtn.configure(text="START SA TIMER")
        if(saCallback):
            saCallback.cancel()
    else:
        saBtn.configure(text="CANCEL SA TIMER")
        saCallback = threading.Timer(delay.get(), askSA)
        saCallback.start()

    rate.sleep()

def askSA():
    global saBtn
    global isSAStarted
    print("ASKING RANDOM SA!");
    saBtn.configure(text="START SA TIMER")
    isSAStarted = False
    questionStr = sa1.get() + ";" + sa2.get() + ";" + sa3.get()
    print("ASKING: " + questionStr)
    question_pub.publish(questionStr)


pause_pub = rospy.Publisher('pause', Bool, queue_size=10)
start_pub = rospy.Publisher('start', Bool, queue_size=10)
end_pub = rospy.Publisher('stop', Bool, queue_size=10)
question_pub = rospy.Publisher('question', String, queue_size=10)
rospy.init_node('experimenter_ui', anonymous=True)
rate = rospy.Rate(10)

isPaused = False
isStarted = False
isSAStarted = False
saCallback = None

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
                  command=onStart)
startBtn.pack(padx=5, pady=5)


frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)

frame.grid(row=0, column=2, padx=5, pady=5)
pauseBtn = tk.Button(master=frame, text="PAUSE", width=30,
                  command=onPause)
pauseBtn.pack(padx=5, pady=5)

frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)

frame.grid(row=1, column=0, padx=5, pady=5)
saBtn = tk.Button(master=frame, text="START SA TIMER", width=30,
                  command=onStartSA)
saBtn.pack(padx=5, pady=5)
frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)


DELAY_OPTIONS = [
1,
2,
3
]
delay = tk.IntVar(frame)
delay.set(DELAY_OPTIONS[0])

SA1_OPTIONS = [
"Where is the obstacle closest to the robot?",
"What color is the obstacle closest to the robot?",
"Where is the robot on this map?",
"Which wall is closest to the robot?"
]
sa1 = tk.StringVar(frame)
sa1.set(SA1_OPTIONS[0])

SA2_OPTIONS = [
"How many objectives has the robot reached thus far?",
"Where was the last objective the robot reached? If you have not yet reached an objective, where was the starting location?",
"Out of the three objectives, how many are remaining?",
"Where is the next objective?"
]
sa2 = tk.StringVar(frame)
sa2.set(SA2_OPTIONS[0])

SA3_OPTIONS = [
"Based on where the robot is currently located on the grid, if the robot drives up 2 grid blocks, will it run into any obstacles?",
"Based on where the robot is currently located on the grid, if the robot drives down 2 grid blocks, will it run into any obstacles.",
"Based on where the robot is currently located on the grid, if the robot drives right 2 grid blocks, will it run into any obstacles.",
"In which direction would the robot need to turn toward to be at the next target?",
"In terms of distance, what percent of the way are you to your next target? Please use the slider."
]
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

window.mainloop()
rospy.spin()
